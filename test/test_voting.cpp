// test_voting.cpp
//
// Host-side unit tests for the voting core. Compiles and runs on a PC with
// no Arduino toolchain -- the whole point of keeping voting_core free of
// Arduino dependencies. Build/run with test/run_tests.sh (or see below).
//
//   g++ -std=c++11 -Wall -Wextra -I../sensor_voting
//       test_voting.cpp ../sensor_voting/voting_core.cpp -o run_tests
//   ./run_tests
//
// Exit code 0 = all pass, 1 = a failure (usable in CI).

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "voting_core.h"

using namespace voting;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    g_checks++;                                                        \
    if (!(cond)) {                                                     \
      g_failures++;                                                    \
      std::printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
    }                                                                  \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                          \
  do {                                                                 \
    g_checks++;                                                        \
    if (std::fabs((a) - (b)) > (tol)) {                                \
      g_failures++;                                                    \
      std::printf("  FAIL %s:%d: |%.3f - %.3f| > %.3f\n", __FILE__,    \
                  __LINE__, (double)(a), (double)(b), (double)(tol));  \
    }                                                                  \
  } while (0)

static Config makeConfig() {
  // Mirrors the TMP36 config in the sketch but with a short persistence so
  // tests stay readable. min/max -10..60, threshold 8, persistence 3,
  // reinstate 4, hold 2000 ms.
  Config c;
  c.min_valid = -10.0f;
  c.max_valid = 60.0f;
  c.disagree_threshold = 8.0f;
  c.fault_persistence = 3;
  c.reinstate_persistence = 4;
  c.max_hold_ms = 2000;
  return c;
}

static ChannelInput mk(float v, bool valid = true) {
  ChannelInput in;
  in.value = v;
  in.sample_valid = valid;
  return in;
}

// Drive the voter for `cycles` steps with the same three inputs, advancing
// the clock by 50 ms each step. Returns the final result.
static VoteResult run(Voter& v, ChannelInput a, ChannelInput b, ChannelInput c,
                      int cycles, uint32_t* clock) {
  ChannelInput in[3];
  VoteResult r;
  for (int i = 0; i < cycles; i++) {
    in[0] = a;
    in[1] = b;
    in[2] = c;
    r = v.update(in, *clock);
    *clock += 50;
  }
  return r;
}

static void test_all_agree() {
  std::printf("test_all_agree\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  VoteResult r = run(v, mk(25.0f), mk(25.2f), mk(24.8f), 1, &t);
  CHECK(r.valid);
  CHECK(r.mode == SystemMode::Normal3ch);
  CHECK_NEAR(r.value, 25.0f, 0.001f);  // median of the three
  CHECK(r.used_count == 3);
  CHECK(!r.outlier_pending);
}

static void test_single_outlier_rejected() {
  std::printf("test_single_outlier_rejected\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  // ch2 reads 40 while ch0/ch1 agree near 25. Median rejects the outlier
  // immediately in the value; exclusion latches after persistence cycles.
  VoteResult r = run(v, mk(25.0f), mk(40.0f), mk(24.5f), 1, &t);
  CHECK_NEAR(r.value, 25.0f, 0.001f);  // outlier never reaches the output
  CHECK(r.valid);
  CHECK(r.outlier_pending);
  CHECK(v.channel(1).state == ChannelState::Suspect);

  // Keep the fault present until it latches (persistence = 3).
  r = run(v, mk(25.0f), mk(40.0f), mk(24.5f), 3, &t);
  CHECK(v.channel(1).state == ChannelState::ExcludedMiscompare);
  CHECK(r.mode == SystemMode::Degraded2ch);  // dropped to 2 good channels
  CHECK_NEAR(r.value, 24.75f, 0.001f);       // average of the two survivors
}

static void test_miscompare_latches() {
  std::printf("test_miscompare_latches\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  run(v, mk(25.0f), mk(40.0f), mk(24.5f), 4, &t);
  CHECK(v.channel(1).state == ChannelState::ExcludedMiscompare);
  // Even if the bad channel starts agreeing again, it stays excluded.
  VoteResult r = run(v, mk(25.0f), mk(25.0f), mk(24.5f), 5, &t);
  CHECK(v.channel(1).state == ChannelState::ExcludedMiscompare);
  CHECK(r.mode == SystemMode::Degraded2ch);
  // Reset readmits it.
  v.reset();
  r = run(v, mk(25.0f), mk(25.0f), mk(24.5f), 1, &t);
  CHECK(v.channel(1).state == ChannelState::Healthy);
  CHECK(r.mode == SystemMode::Normal3ch);
}

static void test_out_of_range_never_used() {
  std::printf("test_out_of_range_never_used\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  // ch0 shorted low (-50, below -10 min). Must be excluded from the value
  // even on the very first cycle, before latching.
  VoteResult r = run(v, mk(-50.0f), mk(25.0f), mk(25.3f), 1, &t);
  CHECK(r.mode == SystemMode::Degraded2ch);
  CHECK_NEAR(r.value, 25.15f, 0.001f);
  CHECK((r.used_mask & 0x1) == 0);  // ch0 not used
  r = run(v, mk(-50.0f), mk(25.0f), mk(25.3f), 3, &t);
  CHECK(v.channel(0).state == ChannelState::ExcludedRange);
}

static void test_dropout_and_reinstate() {
  std::printf("test_dropout_and_reinstate\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  // ch2 stops sending samples. After persistence it is excluded (dropout).
  VoteResult r = run(v, mk(25.0f), mk(25.1f), mk(0, false), 3, &t);
  CHECK(v.channel(2).state == ChannelState::ExcludedDropout);
  CHECK(r.mode == SystemMode::Degraded2ch);
  // Samples return: must NOT be used until reinstate persistence met.
  r = run(v, mk(25.0f), mk(25.1f), mk(25.0f), 3, &t);
  CHECK(v.channel(2).state == ChannelState::ExcludedDropout);
  CHECK(r.mode == SystemMode::Degraded2ch);
  // One more clean cycle (total 4 = reinstate) readmits it.
  r = run(v, mk(25.0f), mk(25.1f), mk(25.0f), 1, &t);
  CHECK(v.channel(2).state == ChannelState::Healthy);
  CHECK(r.mode == SystemMode::Normal3ch);
}

static void test_degrade_to_one_channel() {
  std::printf("test_degrade_to_one_channel\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  // ch1 dropout, ch2 shorted -> only ch0 usable.
  VoteResult r = run(v, mk(25.0f), mk(0, false), mk(-50.0f), 4, &t);
  CHECK(r.mode == SystemMode::Degraded1ch);
  CHECK(r.valid);  // usable but low confidence
  CHECK_NEAR(r.value, 25.0f, 0.001f);
  CHECK(r.used_count == 1);
}

static void test_dual_disagree_holds_then_invalid() {
  std::printf("test_dual_disagree_holds_then_invalid\n");
  Config c = makeConfig();
  Voter v(c);
  uint32_t t = 1000;
  // Establish a good last-value with all three, then drop one and make the
  // remaining two disagree. No referee -> hold last-good, then invalid.
  run(v, mk(25.0f), mk(25.0f), mk(25.0f), 2, &t);
  // ch2 dropout so only ch0/ch1 remain; make them disagree by > threshold.
  VoteResult r = run(v, mk(20.0f), mk(35.0f), mk(0, false), 1, &t);
  CHECK(r.mode == SystemMode::DualDisagree);
  CHECK(r.used_hold);
  CHECK(r.valid);  // still within hold window
  CHECK_NEAR(r.value, 25.0f, 0.001f);  // held value, not a guess

  // Advance past max_hold_ms (2000 ms => 40 cycles of 50 ms).
  r = run(v, mk(20.0f), mk(35.0f), mk(0, false), 45, &t);
  CHECK(r.mode == SystemMode::DualDisagree);
  CHECK(!r.valid);  // hold expired -> output must not be trusted
}

static void test_total_failure() {
  std::printf("test_total_failure\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  VoteResult r =
      run(v, mk(0, false), mk(0, false), mk(0, false), 4, &t);
  CHECK(r.mode == SystemMode::Failed);
  CHECK(!r.valid);
  CHECK(r.used_count == 0);
}

static void test_three_way_split_no_blame() {
  std::printf("test_three_way_split_no_blame\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  // All three mutually disagree (spread > threshold each way): 10/25/40.
  VoteResult r = run(v, mk(10.0f), mk(25.0f), mk(40.0f), 5, &t);
  CHECK(r.three_way_split);
  CHECK(r.mode == SystemMode::Normal3ch);
  CHECK_NEAR(r.value, 25.0f, 0.001f);  // median still least-wrong
  // No channel should be blamed/excluded on a pure split.
  CHECK(v.channel(0).state != ChannelState::ExcludedMiscompare);
  CHECK(v.channel(1).state != ChannelState::ExcludedMiscompare);
  CHECK(v.channel(2).state != ChannelState::ExcludedMiscompare);
}

static void test_transient_glitch_no_trip() {
  std::printf("test_transient_glitch_no_trip\n");
  Voter v(makeConfig());
  uint32_t t = 1000;
  // Single-cycle outlier then recovery must NOT latch (persistence = 3).
  run(v, mk(25.0f), mk(25.0f), mk(25.0f), 2, &t);
  VoteResult r = run(v, mk(25.0f), mk(45.0f), mk(25.0f), 1, &t);  // 1 glitch
  CHECK(r.outlier_pending);
  CHECK(v.channel(1).state == ChannelState::Suspect);
  r = run(v, mk(25.0f), mk(25.0f), mk(25.0f), 2, &t);  // recovers
  CHECK(v.channel(1).state == ChannelState::Healthy);
  CHECK(r.mode == SystemMode::Normal3ch);
}

int main() {
  test_all_agree();
  test_single_outlier_rejected();
  test_miscompare_latches();
  test_out_of_range_never_used();
  test_dropout_and_reinstate();
  test_degrade_to_one_channel();
  test_dual_disagree_holds_then_invalid();
  test_total_failure();
  test_three_way_split_no_blame();
  test_transient_glitch_no_trip();

  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("TESTS FAILED\n");
  return 1;
}
