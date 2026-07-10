// sim_demo.cpp
//
// Hardware-free demonstration of the voting module. Runs a scripted fault
// scenario through the SAME voting_core the Arduino uses and prints the same
// timestamped CSV + event log the sketch prints over serial. Lets you see
// the voting behaviour (and copy sample output into docs) with no Mega,
// no sensors, and no serial terminal.
//
// Build/run:  sim/run_sim.sh   (or see run_sim.sh for the g++ line)
//
// The scenario, in order:
//   1. All three channels healthy and in agreement.
//   2. Channel 3 develops a slow +offset until it is voted out (2oo3 rejects
//      the outlier; system degrades 3ch -> 2ch).
//   3. Channel 2 drops out entirely (2ch -> 1ch, low confidence).
//   4. Channel 2's samples return; after the reinstate window it rejoins,
//      but channel 3 stays latched out -> back to 2ch, not 3ch.
//   5. Operator reset clears the latched fault -> full 3ch protection.

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "voting_core.h"

using namespace voting;

namespace {

// Same CSV format as the sketch's logCsvRow(), rendered to stdout.
void printRow(const Voter& v, uint32_t t, const ChannelInput in[3],
              const VoteResult& r) {
  std::printf("%lu", (unsigned long)t);
  for (int i = 0; i < 3; i++) {
    if (in[i].sample_valid) std::printf(",%.2f", in[i].value);
    else std::printf(",--");
  }
  for (int i = 0; i < 3; i++) {
    std::printf(",%s", channelStateName(v.channel(i).state));
  }
  std::printf(",%.2f,%d,%s,", r.value, r.valid ? 1 : 0,
              systemModeName(r.mode));
  for (int b = 2; b >= 0; b--) std::printf("%d", (r.used_mask >> b) & 1);
  std::printf(",");
  if (r.used_hold) std::printf("HOLD ");
  if (r.outlier_pending) std::printf("OUTLIER? ");
  if (r.three_way_split) std::printf("SPLIT ");
  std::printf("\n");
}

ChannelState g_prev[3];
SystemMode g_prevMode = SystemMode::Normal3ch;

void printEvents(const Voter& v, uint32_t t, const VoteResult& r) {
  for (int i = 0; i < 3; i++) {
    ChannelState s = v.channel(i).state;
    if (s != g_prev[i]) {
      std::printf("# t=%lu EVT ch%d %s -> %s (last=%.2f)\n",
                  (unsigned long)t, i + 1, channelStateName(g_prev[i]),
                  channelStateName(s), v.channel(i).last_value);
      g_prev[i] = s;
    }
  }
  if (r.mode != g_prevMode) {
    std::printf("# t=%lu EVT mode %s -> %s voted=%.2f valid=%d\n",
                (unsigned long)t, systemModeName(g_prevMode),
                systemModeName(r.mode), r.value, r.valid ? 1 : 0);
    g_prevMode = r.mode;
  }
}

ChannelInput mk(float v, bool valid = true) {
  ChannelInput in;
  in.value = v;
  in.sample_valid = valid;
  return in;
}

}  // namespace

int main() {
  // Matches the sketch's TMP36 config.
  Config cfg;
  cfg.min_valid = -10.0f;
  cfg.max_valid = 60.0f;
  cfg.disagree_threshold = 8.0f;
  cfg.fault_persistence = 10;
  cfg.reinstate_persistence = 40;
  cfg.max_hold_ms = 2000;

  Voter voter(cfg);
  for (int i = 0; i < 3; i++) g_prev[i] = ChannelState::Healthy;

  std::printf("t_ms,v1,v2,v3,s1,s2,s3,voted,valid,mode,used_mask,flags (degC)\n");

  const uint32_t dt = 50;  // 20 Hz, same as the sketch
  uint32_t t = 0;
  int decim = 0;

  auto step = [&](ChannelInput a, ChannelInput b, ChannelInput c) {
    ChannelInput in[3] = {a, b, c};
    VoteResult r = voter.update(in, t);
    printEvents(voter, t, r);
    if ((decim++ % 5) == 0) printRow(voter, t, in, r);
    t += dt;
  };

  // 1. Healthy agreement (~24.x-25.x degC), 1.5 s.
  std::printf("# --- phase 1: all healthy ---\n");
  for (int i = 0; i < 30; i++) step(mk(25.0f), mk(24.7f), mk(25.2f));

  // 2. ch3 drifts high until voted out. Ramp 25 -> 45 over ~1.5 s then hold.
  std::printf("# --- phase 2: ch3 slow +offset fault ---\n");
  for (int i = 0; i < 30; i++) {
    float bad = 25.2f + (20.0f * i / 30.0f);
    step(mk(25.0f), mk(24.7f), mk(bad));
  }
  for (int i = 0; i < 20; i++) step(mk(25.0f), mk(24.7f), mk(45.0f));

  // 3. ch2 drops out entirely -> only ch1 usable.
  std::printf("# --- phase 3: ch2 dropout ---\n");
  for (int i = 0; i < 30; i++) step(mk(25.0f), mk(0, false), mk(45.0f));

  // 4. ch2 returns; after reinstate window it rejoins (ch3 still latched).
  std::printf("# --- phase 4: ch2 recovers ---\n");
  for (int i = 0; i < 55; i++) step(mk(25.0f), mk(24.8f), mk(45.0f));

  // 5. Operator reset -> ch3 readmitted, full 3ch (it now reads sanely).
  std::printf("# --- phase 5: operator reset ---\n");
  voter.reset();
  for (int i = 0; i < 3; i++) g_prev[i] = voter.channel(i).state;
  g_prevMode = SystemMode::Normal3ch;
  for (int i = 0; i < 20; i++) step(mk(25.0f), mk(24.8f), mk(25.1f));

  std::printf("# --- end of scenario ---\n");
  return 0;
}
