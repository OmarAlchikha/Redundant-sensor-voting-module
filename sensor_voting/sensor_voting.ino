// sensor_voting.ino
//
// Redundant sensor voting module for Arduino Mega 2560.
//
// Reads three analog sensors measuring the same physical quantity, runs a
// 2-out-of-3 mid-value-select vote (voting_core.h/.cpp), degrades gracefully
// as sensors fail, drives status LEDs, logs every voting decision as
// timestamped CSV over serial, and accepts serial commands to inject faults
// for demonstration.
//
// Serial: 115200 baud, newline-terminated commands. Type "help".
//
// All fault-detection/voting logic lives in voting_core.{h,cpp} (no Arduino
// dependencies) so it can be unit-tested on a host PC. This file only does
// I/O: ADC reads, unit conversion, fault injection, LEDs, and logging.

#include "voting_core.h"

// ---------------- build-time configuration ----------------

// Sensor type. TMP36: three TMP36 analog temperature sensors mounted
// together. POT: three potentiometer wipers as stand-in "sensors" so the
// whole demo works with nothing but three pots (twist one to create a
// disagreement by hand).
#define SENSOR_TMP36 1
#define SENSOR_POT 2
#ifndef SENSOR_MODE
#define SENSOR_MODE SENSOR_TMP36
#endif

static const uint8_t kSensorPins[voting::kNumChannels] = {A0, A1, A2};

// Status LEDs (each through ~330 ohm to GND).
static const uint8_t kLedGreen = 22;   // NORMAL: full 2oo3 protection
static const uint8_t kLedYellow = 24;  // DEGRADED: solid = 2ch, blink = 1ch
static const uint8_t kLedRed = 26;     // FAIL / disagreement, output invalid

static const uint32_t kLoopPeriodMs = 50;  // 20 Hz voting cycle
static const uint8_t kOversample = 5;      // raw reads per channel per cycle
static const uint32_t kSerialBaud = 115200;

#if SENSOR_MODE == SENSOR_TMP36
// TMP36 on 5.0 V: Vout = 0.5 V + 10 mV/degC  ->  T = (V - 0.5) * 100
// Plausibility band -10..60 degC covers any indoor demo; a short to GND
// reads -50 degC and a rail/open reads ~+450 degC, both far outside it.
// Disagreement threshold 8 degC: 2 x (+/-3 degC worst-case TMP36 accuracy)
// + 2 degC noise/gradient margin. See README for the stack-up.
static const voting::Config kVoterConfig = {
    -10.0f,  // min_valid  [degC]
    60.0f,   // max_valid  [degC]
    8.0f,    // disagree_threshold [degC]
    10,      // fault_persistence: 10 cycles @ 20 Hz = 500 ms
    40,      // reinstate_persistence: 2 s of clean data to readmit a dropout
    2000     // max_hold_ms for last-good output
};
static const char kUnits[] = "degC";
#else
// Pot mode: wiper voltage mapped to 0..100 "units". Valid band 2..98 so
// twisting a pot hard against either rail demonstrates an out-of-range
// (open/short-like) fault without any wiring changes.
static const voting::Config kVoterConfig = {
    2.0f, 98.0f, 8.0f, 10, 40, 2000};
static const char kUnits[] = "units";
#endif

// Log every Nth cycle (events always log immediately). 5 => 4 lines/s.
static uint8_t g_logDecimation = 5;

// ---------------- fault injection ----------------
//
// Faults are injected in software at the point that best mimics the real
// failure: OPEN/SHORT/STUCK replace the raw ADC count (electrical faults),
// OFFSET/NOISE corrupt the converted engineering value (calibration drift,
// interference), DROPOUT suppresses the sample entirely (dead sensor /
// broken comms). Injection happens BEFORE validation, so the full detection
// path is exercised exactly as it would be by a genuine failure.

enum class Inject : uint8_t { None, Open, Short, Stuck, Offset, Noise, Dropout };

struct InjectState {
  Inject mode = Inject::None;
  float param = 0.0f;  // offset size or noise amplitude, engineering units
  int stuck_raw = 0;   // ADC count frozen at injection time
};

static InjectState g_inject[voting::kNumChannels];

static const char* injectName(Inject m) {
  switch (m) {
    case Inject::None:    return "none";
    case Inject::Open:    return "open";
    case Inject::Short:   return "short";
    case Inject::Stuck:   return "stuck";
    case Inject::Offset:  return "offset";
    case Inject::Noise:   return "noise";
    case Inject::Dropout: return "dropout";
  }
  return "?";
}

// ---------------- sensor acquisition ----------------

static int readRawMedian(uint8_t pin) {
  // Median-of-5 raw reads: cheap spike rejection so a single ADC glitch
  // never looks like a channel disagreement.
  int s[kOversample];
  for (uint8_t i = 0; i < kOversample; i++) {
    s[i] = analogRead(pin);
  }
  // insertion sort (N=5)
  for (uint8_t i = 1; i < kOversample; i++) {
    int key = s[i];
    int8_t j = i - 1;
    while (j >= 0 && s[j] > key) {
      s[j + 1] = s[j];
      j--;
    }
    s[j + 1] = key;
  }
  return s[kOversample / 2];
}

static float convertToUnits(int raw) {
#if SENSOR_MODE == SENSOR_TMP36
  float volts = raw * (5.0f / 1023.0f);
  return (volts - 0.5f) * 100.0f;
#else
  return raw * (100.0f / 1023.0f);
#endif
}

static voting::ChannelInput readChannel(uint8_t i) {
  voting::ChannelInput in;
  const InjectState& inj = g_inject[i];

  if (inj.mode == Inject::Dropout) {
    in.value = 0.0f;
    in.sample_valid = false;
    return in;
  }

  int raw = readRawMedian(kSensorPins[i]);
  switch (inj.mode) {
    case Inject::Open:  raw = 1023; break;  // input pulled to rail
    case Inject::Short: raw = 0;    break;  // input shorted to ground
    case Inject::Stuck: raw = inj.stuck_raw; break;
    default: break;
  }

  float v = convertToUnits(raw);
  if (inj.mode == Inject::Offset) v += inj.param;
  if (inj.mode == Inject::Noise) {
    v += random(-1000, 1001) * (inj.param / 1000.0f);
  }

  in.value = v;
  in.sample_valid = true;
  return in;
}

// ---------------- voter, logging, LEDs ----------------

static voting::Voter g_voter(kVoterConfig);
static voting::ChannelState g_prevState[voting::kNumChannels];
static voting::SystemMode g_prevMode = voting::SystemMode::Normal3ch;
static uint32_t g_cycle = 0;

static void printFloat(float v) { Serial.print(v, 2); }

static void logCsvHeader() {
  Serial.print(F("t_ms,v1,v2,v3,s1,s2,s3,voted,valid,mode,used_mask,flags ("));
  Serial.print(kUnits);
  Serial.println(F(")"));
}

static void logCsvRow(uint32_t t, const voting::ChannelInput in[],
                      const voting::VoteResult& r) {
  Serial.print(t);
  for (uint8_t i = 0; i < voting::kNumChannels; i++) {
    Serial.print(',');
    if (in[i].sample_valid) printFloat(in[i].value);
    else Serial.print(F("--"));
  }
  for (uint8_t i = 0; i < voting::kNumChannels; i++) {
    Serial.print(',');
    Serial.print(voting::channelStateName(g_voter.channel(i).state));
  }
  Serial.print(',');
  printFloat(r.value);
  Serial.print(',');
  Serial.print(r.valid ? 1 : 0);
  Serial.print(',');
  Serial.print(voting::systemModeName(r.mode));
  Serial.print(',');
  Serial.print(r.used_mask, BIN);
  Serial.print(',');
  if (r.used_hold) Serial.print(F("HOLD "));
  if (r.outlier_pending) Serial.print(F("OUTLIER? "));
  if (r.three_way_split) Serial.print(F("SPLIT "));
  Serial.println();
}

// Event lines are prefixed with '#' so a CSV parser can skip them while a
// human reading the stream still sees every decision the instant it happens.
static void logEvents(uint32_t t, const voting::VoteResult& r) {
  for (uint8_t i = 0; i < voting::kNumChannels; i++) {
    voting::ChannelState s = g_voter.channel(i).state;
    if (s != g_prevState[i]) {
      Serial.print(F("# t="));
      Serial.print(t);
      Serial.print(F(" EVT ch"));
      Serial.print(i + 1);
      Serial.print(F(" "));
      Serial.print(voting::channelStateName(g_prevState[i]));
      Serial.print(F(" -> "));
      Serial.print(voting::channelStateName(s));
      Serial.print(F(" (last="));
      printFloat(g_voter.channel(i).last_value);
      Serial.println(F(")"));
      g_prevState[i] = s;
    }
  }
  if (r.mode != g_prevMode) {
    Serial.print(F("# t="));
    Serial.print(t);
    Serial.print(F(" EVT mode "));
    Serial.print(voting::systemModeName(g_prevMode));
    Serial.print(F(" -> "));
    Serial.print(voting::systemModeName(r.mode));
    Serial.print(F(" voted="));
    printFloat(r.value);
    Serial.print(F(" valid="));
    Serial.println(r.valid ? 1 : 0);
    g_prevMode = r.mode;
  }
}

static void updateLeds(const voting::VoteResult& r, uint32_t t) {
  bool green = false, yellow = false, red = false;
  const bool blink = ((t / 250) & 1) != 0;
  switch (r.mode) {
    case voting::SystemMode::Normal3ch:
      green = true;
      break;
    case voting::SystemMode::Degraded2ch:
      yellow = true;
      break;
    case voting::SystemMode::Degraded1ch:
      yellow = blink;
      break;
    case voting::SystemMode::DualDisagree:
    case voting::SystemMode::Failed:
      // While holding last-good the output is still usable: red + yellow.
      // Once the hold expires the output is invalid: red only.
      red = true;
      yellow = r.valid;
      break;
  }
  digitalWrite(kLedGreen, green ? HIGH : LOW);
  digitalWrite(kLedYellow, yellow ? HIGH : LOW);
  digitalWrite(kLedRed, red ? HIGH : LOW);
}

// ---------------- serial command interface ----------------

static char g_cmdBuf[64];
static uint8_t g_cmdLen = 0;

static void printHelp() {
  Serial.println(F("# Commands (channel = 1..3):"));
  Serial.println(F("#   inject <ch> open        signal pinned at +rail"));
  Serial.println(F("#   inject <ch> short       signal pinned at GND"));
  Serial.println(F("#   inject <ch> stuck       freeze at current reading"));
  Serial.println(F("#   inject <ch> offset <u>  add bias in eng. units"));
  Serial.println(F("#   inject <ch> noise <u>   add +/-u random noise"));
  Serial.println(F("#   inject <ch> dropout     stop delivering samples"));
  Serial.println(F("#   clear <ch>|all          remove injected fault(s)"));
  Serial.println(F("#   reset                   clear latched exclusions"));
  Serial.println(F("#   status                  print channel/system status"));
  Serial.println(F("#   log <n>                 print every nth cycle"));
  Serial.println(F("#   help"));
}

static void printStatus() {
  Serial.print(F("# status t="));
  Serial.println(millis());
  for (uint8_t i = 0; i < voting::kNumChannels; i++) {
    const voting::ChannelStatus& ch = g_voter.channel(i);
    Serial.print(F("#   ch"));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(voting::channelStateName(ch.state));
    Serial.print(F(" last="));
    printFloat(ch.last_value);
    Serial.print(F(" cnt(drop/rng/cmp)="));
    Serial.print(ch.dropout_count);
    Serial.print('/');
    Serial.print(ch.range_count);
    Serial.print('/');
    Serial.print(ch.miscompare_count);
    Serial.print(F(" inject="));
    Serial.println(injectName(g_inject[i].mode));
  }
}

static void handleCommand(char* line) {
  // lowercase in place
  for (char* p = line; *p; p++) {
    if (*p >= 'A' && *p <= 'Z') *p += 'a' - 'A';
  }
  char* tok = strtok(line, " \t");
  if (!tok) return;

  if (strcmp(tok, "help") == 0) {
    printHelp();
  } else if (strcmp(tok, "status") == 0) {
    printStatus();
  } else if (strcmp(tok, "reset") == 0) {
    g_voter.reset();
    Serial.println(F("# CMD reset: latched exclusions cleared"));
  } else if (strcmp(tok, "log") == 0) {
    char* nStr = strtok(NULL, " \t");
    long n = nStr ? atol(nStr) : 0;
    if (n >= 1 && n <= 255) {
      g_logDecimation = (uint8_t)n;
      Serial.print(F("# CMD log every "));
      Serial.print(n);
      Serial.println(F(" cycles"));
    } else {
      Serial.println(F("# ERR usage: log <1..255>"));
    }
  } else if (strcmp(tok, "clear") == 0) {
    char* chStr = strtok(NULL, " \t");
    if (chStr && strcmp(chStr, "all") == 0) {
      for (uint8_t i = 0; i < voting::kNumChannels; i++) {
        g_inject[i].mode = Inject::None;
      }
      Serial.println(F("# CMD cleared all injected faults"));
    } else {
      long ch = chStr ? atol(chStr) : 0;
      if (ch >= 1 && ch <= voting::kNumChannels) {
        g_inject[ch - 1].mode = Inject::None;
        Serial.print(F("# CMD cleared fault on ch"));
        Serial.println(ch);
      } else {
        Serial.println(F("# ERR usage: clear <1..3>|all"));
      }
    }
  } else if (strcmp(tok, "inject") == 0) {
    char* chStr = strtok(NULL, " \t");
    char* what = strtok(NULL, " \t");
    char* arg = strtok(NULL, " \t");
    long ch = chStr ? atol(chStr) : 0;
    if (ch < 1 || ch > voting::kNumChannels || !what) {
      Serial.println(F("# ERR usage: inject <1..3> <fault> [param]"));
      return;
    }
    InjectState& inj = g_inject[ch - 1];
    if (strcmp(what, "open") == 0) {
      inj.mode = Inject::Open;
    } else if (strcmp(what, "short") == 0) {
      inj.mode = Inject::Short;
    } else if (strcmp(what, "stuck") == 0) {
      inj.stuck_raw = readRawMedian(kSensorPins[ch - 1]);
      inj.mode = Inject::Stuck;
    } else if (strcmp(what, "offset") == 0) {
      inj.param = arg ? atof(arg) : 10.0f;
      inj.mode = Inject::Offset;
    } else if (strcmp(what, "noise") == 0) {
      inj.param = arg ? atof(arg) : 5.0f;
      inj.mode = Inject::Noise;
    } else if (strcmp(what, "dropout") == 0) {
      inj.mode = Inject::Dropout;
    } else {
      Serial.println(
          F("# ERR fault: open|short|stuck|offset|noise|dropout"));
      return;
    }
    Serial.print(F("# CMD inject ch"));
    Serial.print(ch);
    Serial.print(F(" "));
    Serial.print(injectName(inj.mode));
    if (inj.mode == Inject::Offset || inj.mode == Inject::Noise) {
      Serial.print(F(" "));
      printFloat(inj.param);
    }
    Serial.println();
  } else {
    Serial.println(F("# ERR unknown command, try: help"));
  }
}

static void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (g_cmdLen > 0) {
        g_cmdBuf[g_cmdLen] = '\0';
        handleCommand(g_cmdBuf);
        g_cmdLen = 0;
      }
    } else if (g_cmdLen < sizeof(g_cmdBuf) - 1) {
      g_cmdBuf[g_cmdLen++] = c;
    }
  }
}

// ---------------- main ----------------

void setup() {
  Serial.begin(kSerialBaud);
  pinMode(kLedGreen, OUTPUT);
  pinMode(kLedYellow, OUTPUT);
  pinMode(kLedRed, OUTPUT);
  randomSeed(analogRead(A7));  // unconnected pin as entropy for noise inject

  for (uint8_t i = 0; i < voting::kNumChannels; i++) {
    g_prevState[i] = voting::ChannelState::Healthy;
  }

  Serial.println(F("# Redundant sensor voting module (2oo3 mid-value select)"));
  Serial.print(F("# sensors="));
#if SENSOR_MODE == SENSOR_TMP36
  Serial.print(F("TMP36 x3"));
#else
  Serial.print(F("potentiometer x3"));
#endif
  Serial.print(F(" rate="));
  Serial.print(1000 / kLoopPeriodMs);
  Serial.print(F("Hz threshold="));
  Serial.print(kVoterConfig.disagree_threshold, 1);
  Serial.print(kUnits);
  Serial.println(F(" -- type 'help' for fault injection"));
  logCsvHeader();
}

void loop() {
  static uint32_t nextTick = 0;

  pollSerial();

  uint32_t now = millis();
  if ((int32_t)(now - nextTick) < 0) return;
  nextTick = now + kLoopPeriodMs;  // fixed-rate; skips forward after stalls

  voting::ChannelInput in[voting::kNumChannels];
  for (uint8_t i = 0; i < voting::kNumChannels; i++) {
    in[i] = readChannel(i);
  }

  voting::VoteResult r = g_voter.update(in, now);

  logEvents(now, r);
  if ((g_cycle % g_logDecimation) == 0) logCsvRow(now, in, r);
  updateLeds(r, now);
  g_cycle++;
}
