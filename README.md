# Redundant Sensor Voting Module (Arduino Mega)

A small, self-contained demonstration of **triple-redundant sensor voting**
with **2-out-of-3 (2oo3) mid-value-select logic**, graceful degradation, fault
latching, timestamped decision logging, and a serial fault-injection interface
for demos. It runs on an Arduino Mega 2560 with three analog sensors, and the
voting logic also builds and runs on a plain PC for unit testing and a
hardware-free simulation.

The design borrows the *pattern* used in redundant instrumentation and flight
control systems — measure the same quantity with independent channels, cross-
compare them, reject the outlier, and degrade in a defined, annunciated way as
channels are lost — but it is built entirely from public, textbook engineering.
There is no proprietary, employer-specific, or safety-certified content here;
it is a learning/portfolio project, not flight-worthy software (see
[Scope & limitations](#scope--limitations)).

---

## Contents

```
sensor_voting/
  voting_core.h      voting/fault/degradation logic -- NO Arduino dependencies
  voting_core.cpp
  sensor_voting.ino  Arduino sketch: ADC read, unit conversion, fault
                     injection, LEDs, serial logging & command interface
test/
  test_voting.cpp    host-side unit tests (10 scenarios, 50 assertions)
  run_tests.sh
sim/
  sim_demo.cpp       hardware-free scripted scenario, prints the real log format
  run_sim.sh
```

---

## What it does

- Reads **three** sensors measuring the same physical quantity (temperature
  with TMP36s, or three potentiometers as a zero-extra-hardware stand-in).
- Runs a **2oo3 vote** every 50 ms (20 Hz): the output is the **median** of the
  healthy channels, which is provably immune to any *single* bad channel.
- **Detects and rejects** a failed/outlier sensor via three independent checks:
  range (open/short to a rail), dropout (no samples), and cross-channel
  miscompare (in-range but wrong).
- **Degrades gracefully**: `3ch → 2ch → 1ch → fail`, annunciated on three LEDs
  and in the log, with a bounded last-good-value hold when no fresh vote is
  possible.
- **Flags disagreement** it cannot arbitrate (two channels left, and they
  disagree) instead of silently picking one.
- **Injects faults on command** over serial for demos (`open`, `short`,
  `stuck`, `offset`, `noise`, `dropout`) without touching the wiring.
- **Logs every decision** as timestamped CSV, plus immediate `#`-prefixed event
  lines whenever a channel or the system changes state.

---

## Hardware

### Bill of materials

| Qty | Part | Notes |
|----:|------|-------|
| 1 | Arduino Mega 2560 | Any Uno/Nano works too; Mega chosen for pin headroom and a second UART for logging while debugging |
| 3 | TMP36 temperature sensors | Analog, 10 mV/°C, 0.5 V @ 0 °C. *Or* 3× 10 kΩ potentiometers for the pot demo mode |
| 3 | 330 Ω resistors | LED current limiting |
| 1 | Green / 1 Yellow / 1 Red LED | Status annunciation |
| — | 0.1 µF caps (optional) | One per TMP36 across supply, close to the sensor, for noise |

### Pin map

| Signal | Pin |
|--------|-----|
| Sensor 1 | A0 |
| Sensor 2 | A1 |
| Sensor 3 | A2 |
| Green LED ("3CH normal") | D22 |
| Yellow LED ("degraded") | D24 |
| Red LED ("fail / disagree") | D26 |
| Noise-seed (unconnected) | A7 |

### Wiring (TMP36 mode)

```
              +5V ───────┬───────────┬───────────┬────────
                         │           │           │
                       TMP36       TMP36       TMP36
                       (S1)        (S2)        (S3)
                    +Vs GND Vout ...        ...
                         │   │           (each sensor: +Vs->5V, GND->GND)
              GND ───────┴───┴───────────┴───────────┴────
                             │
              A0 <───────────┘ (S1 Vout)   A1 <── S2 Vout   A2 <── S3 Vout

   D22 ──[330Ω]──|>|── GND   (green)
   D24 ──[330Ω]──|>|── GND   (yellow)
   D26 ──[330Ω]──|>|── GND   (red)
```

For **pot mode**, wire each pot as a divider (ends to 5 V and GND, wiper to
A0/A1/A2) and set `SENSOR_MODE` to `SENSOR_POT` at the top of the sketch. You
can then create a disagreement by simply twisting one pot, and an out-of-range
fault by turning it hard against either end stop.

> **A note on "independent" redundancy:** wiring all three sensors to one
> Arduino, one 5 V rail, and one ADC reference means they share common-mode
> failure points (supply, reference, MCU). Real redundant systems use
> dissimilar power, separate references/ADCs, and sometimes dissimilar sensor
> technologies. This project demonstrates the *voting algorithm*; the shared
> front-end is called out honestly in the interview section below.

---

## Build & run

### On the Arduino

1. Open `sensor_voting/sensor_voting.ino` in the Arduino IDE (or `arduino-cli`).
2. Select **Arduino Mega 2560**, upload.
3. Open Serial Monitor at **115200 baud**, line ending = Newline.
4. You'll see a header, then streaming CSV. Type `help` for fault-injection
   commands.

### On a PC (no hardware needed)

The voting core is deliberately free of any Arduino dependency, so it compiles
with a stock C++ compiler.

```bash
# Unit tests (exit 0 = pass; suitable for CI)
cd test && ./run_tests.sh

# Scripted end-to-end simulation that prints the real log format
cd sim && ./run_sim.sh
```

Both scripts just invoke `g++ -std=c++11`. No external libraries.

---

## Architecture: why the logic is split from the I/O

`voting_core.{h,cpp}` contains **all** the voting, fault-detection, and
degradation logic and has **zero** Arduino dependencies — no `Arduino.h`, no
`Serial`, no `millis()`. Time is passed in explicitly on every `update()` call.
The `.ino` is a thin shell that does only I/O: ADC reads, unit conversion,
fault injection, LEDs, and logging.

This split is the single most important design decision in the project, and it
buys three things:

1. **Testability.** The exact code that runs on the Mega is exercised by 50
   host-side assertions across 10 scenarios, deterministically, in
   milliseconds — no hardware, no timing flakiness. You cannot unit-test logic
   that is tangled up with `analogRead()` and `Serial.print()`.
2. **Determinism.** Passing `now_ms` in as a parameter (instead of the logic
   reaching for `millis()`) means every time-dependent behaviour — the last-good
   hold window especially — is reproducible and testable.
3. **Portability.** The same core drives the Arduino, the unit tests, and the
   simulation, so all three agree by construction.

---

## The voting algorithm

### Mid-value select (median), not average

With three healthy channels the output is the **median**. This is the standard
choice for redundant analog voting because the median of three is bounded by the
two *non-selected* values: **a single channel can be arbitrarily wrong — even
railed — and it can never move the output beyond the two good channels.** An
average has no such property; one wild reading drags the mean with it.

When degraded to two agreeing channels, the output is their average (best
linear estimate of two trustworthy readings). With one channel, the output is
that channel passed through, flagged low-confidence.

### Three independent fault checks

Each channel is screened every cycle by three checks, in order:

| Check | Trigger | Meaning | Latches? |
|-------|---------|---------|----------|
| **Range** | sample outside the plausibility band | open/short to a rail; physically impossible reading | **Yes** |
| **Dropout** | no sample delivered | dead sensor / broken link | No (auto-reinstates) |
| **Miscompare** | in-range but disagrees with the agreeing pair | drift, decalibration, stuck value | **Yes** |

An out-of-range sample is **never** allowed into the vote, even on the first
cycle before it latches — it is known-garbage immediately.

### Persistence filtering (debounce)

A fault must be observed for `fault_persistence` **consecutive** cycles before a
channel is excluded (default 10 cycles = 500 ms at 20 Hz). This prevents a
single noise spike or a one-cycle ADC glitch from nuisance-tripping a good
channel. During accumulation the channel is marked `SUS` (suspect) and the log
raises an `OUTLIER?` flag, but the channel keeps voting until the fault is
confirmed.

### Latching vs. auto-reinstatement

- **Range** and **miscompare** faults **latch**: once confirmed, the channel
  stays excluded until an operator `reset`. Rationale: an in-range-but-wrong or
  previously-railed sensor that *appears* to recover is most likely
  intermittent, and an intermittent sensor that silently rejoins the vote is
  more dangerous than one that stays out. Make a human look at it.
- **Dropout** does **not** latch: it auto-reinstates after
  `reinstate_persistence` consecutive good cycles (default 40 = 2 s). Dropouts
  are frequently transient (connector seat, harness flex), and the recovery
  requirement is deliberately longer than the trip requirement — slow to trust,
  quick to distrust.

---

## Graceful degradation

| Usable channels | Mode | Output | LEDs | Confidence |
|---:|------|--------|------|-----------|
| 3 | `3CH` | median | Green solid | Full 2oo3 protection |
| 2 (agree) | `2CH` | average of the two | Yellow solid | Miscompare *monitor* only — can detect a further split but can't arbitrate it |
| 2 (disagree) | `DISAGREE` | last-good (held, bounded) then invalid | Red + Yellow while held, Red-only after | Cannot arbitrate — **flagged, not guessed** |
| 1 | `1CH` | pass-through | Yellow blink | Low — no cross-check possible |
| 0 | `FAIL` | last-good then invalid | Red solid | None |

The key honesty point is the **`DISAGREE`** row: with only two channels left and
no referee, the module refuses to guess which one is right. It holds the last
agreed value for a bounded window (`max_hold_ms`, default 2 s) so a downstream
consumer has time to react, then marks the output **invalid**. It never
fabricates a "winner."

---

## Fault injection (demo interface)

Type these into the Serial Monitor (channel = 1..3):

| Command | Effect | Mimics |
|---------|--------|--------|
| `inject <ch> open` | reading pinned to +rail | wire open / pull-up fault |
| `inject <ch> short` | reading pinned to GND | short to ground |
| `inject <ch> stuck` | freeze at current reading | frozen/hung sensor |
| `inject <ch> offset <u>` | add a fixed bias (units) | decalibration / drift |
| `inject <ch> noise <u>` | add ±u random noise | EMI / bad ground |
| `inject <ch> dropout` | stop delivering samples | dead sensor / lost comms |
| `clear <ch>` \| `clear all` | remove injected fault(s) | — |
| `reset` | clear **latched** exclusions | operator maintenance action |
| `status` | print per-channel + system status | — |
| `log <n>` | print every *n*th cycle | throttle the stream |
| `help` | list commands | — |

Faults are injected **before** the validation path — `open`/`short`/`stuck`
replace the raw ADC count, `offset`/`noise` corrupt the converted value,
`dropout` suppresses the sample — so injected faults exercise exactly the same
detection code a real failure would.

---

## Log format

CSV columns:

```
t_ms, v1, v2, v3, s1, s2, s3, voted, valid, mode, used_mask, flags
```

- `t_ms` — `millis()` timestamp of the cycle.
- `v1..v3` — per-channel reading (`--` if that channel dropped its sample).
- `s1..s3` — per-channel state: `OK`, `SUS`, `X-RNG`, `X-DROP`, `X-CMP`.
- `voted` — the voted output value.
- `valid` — `1` if the output may be trusted, `0` if not.
- `mode` — `3CH`, `2CH`, `1CH`, `DISAGREE`, `FAIL`.
- `used_mask` — bitmask (ch3 ch2 ch1) of which channels fed the output.
- `flags` — `HOLD` (stale held output), `OUTLIER?` (fault accumulating, not yet
  excluded), `SPLIT` (all three mutually disagree).

Lines beginning with `#` are **event/annotation** lines (state changes, command
acknowledgements, banners). A CSV parser can drop every `#` line and keep the
clean data table; a human watching the stream sees every decision the instant
it happens.

### Sample output (from `sim/run_sim.sh`, abridged)

```
t_ms,v1,v2,v3,s1,s2,s3,voted,valid,mode,used_mask,flags (degC)
0,25.00,24.70,25.20,OK,OK,OK,25.00,1,3CH,111,
# --- phase 2: ch3 slow +offset fault ---
2250,25.00,24.70,35.20,OK,OK,SUS,25.00,1,3CH,111,OUTLIER?
# t=2550 EVT ch3 SUS -> X-CMP (last=39.20)
# t=2600 EVT mode 3CH -> 2CH voted=24.85 valid=1
2750,25.00,24.70,41.87,OK,OK,X-CMP,24.85,1,2CH,011,
# --- phase 3: ch2 dropout ---
# t=4000 EVT mode 2CH -> 1CH voted=25.00 valid=1
4500,25.00,--,45.00,OK,X-DROP,X-CMP,25.00,1,1CH,001,
# --- phase 4: ch2 recovers ---
# t=7450 EVT ch2 X-DROP -> OK (last=24.80)
# t=7450 EVT mode 1CH -> 2CH voted=24.90 valid=1
7500,25.00,24.80,45.00,OK,OK,X-CMP,24.90,1,2CH,011,
# --- phase 5: operator reset ---
8250,25.00,24.80,25.10,OK,OK,OK,25.00,1,3CH,111,
```

Notice: while ch3 drifts up it is caught (`OUTLIER?`) but keeps voting until the
fault is confirmed at t=2550; the output value `voted` never follows ch3 up,
because the median rejects it from the first cycle. That is the whole point.

---

## Design decisions and rationale (the non-obvious ones)

- **Median for 3, average for 2, pass-through for 1.** Median gives the
  single-fault-immunity guarantee; average is the best estimator once you're
  down to two trusted channels; pass-through is all that's left at one, and it's
  explicitly de-rated to low confidence.
- **Logic separated from I/O (`voting_core` has no `Arduino.h`).** Enables host
  unit testing and deterministic time — see [Architecture](#architecture-why-the-logic-is-split-from-the-io).
- **Time injected as a parameter, not read from `millis()` inside the logic.**
  Makes the last-good hold window testable and reproducible.
- **Persistence/debounce before exclusion.** One glitch should not remove a good
  sensor; real signals are noisy. 500 ms default is a balance between nuisance
  immunity and reaction time; it's a single named constant, easy to retune.
- **Range and miscompare latch; dropout does not.** Encodes a trust policy:
  a sensor that was electrically impossible or provably wrong must be
  human-cleared; a sensor that merely went quiet may re-earn trust, but slowly
  (reinstate window > trip window).
- **Out-of-range samples are excluded from the vote immediately**, before they
  latch — known-garbage data never influences the output for even one cycle.
- **Median-of-5 oversampling per channel per cycle** in the sketch, so a single
  ADC spike doesn't masquerade as a channel disagreement. This is
  *within*-channel filtering; the voting is the *cross*-channel filtering.
- **Two-channel disagreement is flagged, not resolved.** With no referee, any
  "pick one" rule is a coin flip that can select the failed sensor. Holding
  last-good for a bounded time and then declaring invalid is the safe behaviour.
- **Bounded last-good hold with an explicit expiry.** An indefinitely-held stale
  value is a classic latent hazard (the number looks alive but isn't). The hold
  exists only to give a consumer time to react, then the output self-invalidates.
- **Three-way split blames no one.** If all three mutually disagree there is no
  evidence to convict any single channel; the median is still bracketed by two
  of the readings, so it's kept as the least-wrong output and flagged `SPLIT`.
- **Fault injection happens before validation**, so demo faults travel the exact
  detection path a real fault would — the demo isn't a special case.
- **`#`-prefixed event lines interleaved with CSV.** One stream serves both a
  machine parser (drop `#`) and a human watching live (see events immediately).
- **Saturating counters.** Persistence counters clamp at 255 instead of wrapping
  — a wrap could momentarily un-trip a confirmed fault.
- **Fixed-rate loop that skips forward after a stall** (`nextTick = now + period`
  with a signed-delta comparison) rather than free-running, so logging or a
  serial burst can't slew the effective sample rate or break the `millis()`
  rollover-safe arithmetic.

---

## Testing

`test/test_voting.cpp` runs 10 scenarios / 50 assertions against the core:

- all-agree median; single outlier rejected from the value immediately;
- miscompare latches and survives the bad channel "recovering"; `reset` clears it;
- out-of-range never used, even pre-latch;
- dropout excludes then auto-reinstates after the (longer) recovery window;
- degradation all the way to 1 channel and to total failure;
- dual-disagreement holds last-good then goes invalid at the hold expiry;
- three-way split keeps the median and blames no channel;
- a transient one-cycle glitch does **not** latch.

```
$ cd test && ./run_tests.sh
...
50 checks, 0 failures
ALL TESTS PASSED
```

The tests build with `-Wall -Wextra -Werror`; the sketch is additionally
compile-checked against a minimal Arduino shim under the same warning flags.

---

## How this would be challenged in an interview

**1. "Your three sensors share one Arduino, one 5 V rail, and one ADC reference.
Isn't that redundancy theatre? A supply or reference fault takes out all three
at once."**

Largely fair, and it's the most important limitation. Voting only protects
against *independent* channel faults; it does nothing for common-mode failures
in the shared front-end. Real redundant instrumentation attacks this with
dissimilar/segregated power, independent references and ADCs (often separate
MCUs entirely), physical separation of harness routing, and sometimes dissimilar
sensor technologies so a single environmental cause can't fail them identically.
What this project *does* correctly demonstrate is the voting/degradation
*algorithm*, which is necessary but not sufficient. One partial mitigation even
on shared hardware: a **ratiometric plausibility check** — because a TMP36's
output and the ADC reference both scale with a sagging 5 V rail, a supply droop
shows up as all three channels moving together in a way that a real temperature
change wouldn't, which you can screen for. I'd also monitor the reference with a
known precision divider on a spare channel. But I wouldn't claim this box is
fault-tolerant against supply loss; it isn't, and pretending otherwise is worse
than stating the boundary.

**2. "You debounce faults over 500 ms before excluding a channel. During those
500 ms a channel is failing but still voting. Justify that number — and what
happens if the transient is a real, fast excursion you needed to catch?"**

It's a classic detection-latency vs. false-alarm trade. Trip instantly and every
noise spike nuisance-removes a healthy sensor, which *reduces* availability and
can walk you down to a fragile single-channel state for no reason. Wait too long
and you carry a bad channel into the vote. The 500 ms figure is tied to the
20 Hz rate (10 cycles) and to the idea that the physical quantity here
(temperature) can't change meaningfully that fast, so anything that does is
noise or a fault. Crucially, **the median already protects the output during the
debounce window** — while ch3 is drifting up but not yet excluded, the voted
value never follows it, because a single outlier can't move a median. So the
debounce delays *exclusion/annunciation*, not *output protection*. For a
fast-slewing quantity I'd shorten persistence, raise the sample rate, or make
the threshold rate-aware (compare slew rates, not just levels). The constant is
a single named parameter precisely so it can be tuned per signal.

**3. "Two channels left and they disagree. You hold the last value for two
seconds and then output 'invalid.' A control loop downstream still needs a
number. What should actually happen, and is holding stale data safe?"**

Holding stale data is a genuine hazard — a value that looks live but is frozen
is how you get latent failures — which is exactly why the hold is *bounded* and
the output carries an explicit `valid` flag rather than being silently trusted.
The module's job is to tell the truth about its own confidence, not to
manufacture a number; deciding what to *do* with "I can no longer arbitrate"
belongs to the system layer, not the voter. Reasonable system responses:
fail-over to an independent estimate or model, hand off to the operator/pilot,
enter a defined safe state, or fall back to a different control law that doesn't
need this input. What you must *not* do is pick one of the two disagreeing
channels by a fixed rule — with no third opinion that's a coin flip that can
select the failed sensor. If continuous operation on two channels were a hard
requirement, the honest answer is that two channels aren't enough to *arbitrate*
and the architecture needs a third dissimilar source (analytical redundancy /
a model-based virtual sensor) to vote against.

**4. "Walk me through your failure-mode coverage. Which sensor failures does
this actually catch, and which slip through?"**

Caught: hard opens/shorts (range check), dead sensors and lost links (dropout),
and drift/decalibration/stuck-in-band values once they exceed the disagreement
threshold against the other two (miscompare). Slips through or is limited:
(a) a **slow common-mode drift** where all three age together in the same
direction stays inside the disagreement band and is invisible to voting — you'd
need an absolute reference or dissimilar sensors; (b) an **in-band stuck value**
that happens to sit within threshold of the real value contributes no error
until the real value moves away, so detection is delayed; (c) **simultaneous
same-direction faults on two channels** can out-vote the one healthy channel —
2oo3 assumes at most one fault at a time, which is why fault *rate* and
*coverage* matter, not just the vote; (d) anything in the **shared front-end**
(see Q1). I'd characterise this honestly with an FMEA rather than claim blanket
coverage, and I'd add per-channel diagnostics (e.g. injected test stimulus,
reasonableness/rate checks) to shrink the in-band-undetected region.

**5. "This is a demo. What separates it from software you could actually deploy
in a safety-related system, and what would you change first?"**

The gap is mostly process and rigor, not cleverness. First: this uses `float`
and non-deterministic-ish library calls and hasn't been analysed for worst-case
execution time or determinism; safety code favors fixed-point or carefully
bounded math, static allocation (already true here), and proven timing. Second:
there's no requirements-to-test traceability, no coverage evidence (MC/DC),
no coding-standard conformance (MISRA-style), and no independent review — all of
which a standard like DO-178C or IEC 61508 would demand and which are what
actually earn trust, more than the algorithm. Third: the hardware needs the
independence work from Q1 and a proper hazard analysis to set the thresholds and
the safe-state behaviour from evidence rather than judgement. Fourth: I'd add
input sanity on the config itself, watchdog/brown-out handling, and power-on
self-test. What I'd defend is the *structure* — logic isolated from I/O,
deterministic time, everything unit-tested, faults exercising real paths,
confidence stated honestly — because that structure is the foundation you'd
build the rigor on top of, rather than something you'd have to tear out.

---

## Scope & limitations

- **Not safety-certified, not flight code.** A learning/portfolio demonstration
  of the voting *pattern* built from public, textbook engineering only.
- **Shared front-end** (one MCU/supply/reference) — no protection against
  common-mode failures; see interview Q1.
- **Single-fault assumption** inherent to 2oo3 — concurrent multi-channel faults
  can defeat the vote.
- **Thresholds are illustrative** and tuned for a benign indoor demo; a real
  deployment sets them from sensor datasheets and a hazard analysis.
- Contains no proprietary, employer-specific, or otherwise non-public material
  of any kind.

## License

MIT — do whatever you like; no warranty.
