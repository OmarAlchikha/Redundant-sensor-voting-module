// voting_core.h
//
// Hardware-independent 2-out-of-3 redundant sensor voting core.
//
// This module contains all voting, fault-detection, and degradation logic.
// It has NO Arduino dependencies (no Arduino.h, no Serial, no millis()) so
// the exact same code that runs on the Mega can be compiled and unit-tested
// on a host PC. Time is passed in explicitly on every update.
//
// Responsibilities:
//   * per-channel signal validation (range check, dropout detection)
//   * cross-channel comparison and outlier identification (2oo3 miscompare)
//   * persistence filtering (a fault must be seen for N consecutive cycles
//     before a channel is excluded -- no nuisance trips on single glitches)
//   * latching of confirmed hard faults until an explicit operator reset
//   * graceful degradation: 3 -> 2 -> 1 -> 0 usable channels, with a
//     time-limited last-good-value hold when no fresh vote is possible
//
// It deliberately does NOT know how sensors are read, converted, or logged.

#ifndef VOTING_CORE_H
#define VOTING_CORE_H

#include <stdint.h>

namespace voting {

constexpr uint8_t kNumChannels = 3;

// Per-channel health state.
enum class ChannelState : uint8_t {
  Healthy = 0,
  // Fault evidence is accumulating (counter > 0) but the persistence
  // threshold has not been reached. Channel may still be used if its
  // current sample is individually valid.
  Suspect,
  // No samples arriving. NOT latched: auto-reinstates after
  // reinstate_persistence consecutive good cycles, because dropouts are
  // often transient (connector, harness flex) rather than sensor death.
  ExcludedDropout,
  // Signal outside the physically plausible band (open/short to rail).
  // Latched until reset(): a sensor that railed once is not trusted again
  // without operator action.
  ExcludedRange,
  // Voted out by the other two channels (persistent miscompare).
  // Latched until reset(): an in-range-but-wrong sensor is the most
  // dangerous failure mode, and one that "comes back" is likely
  // intermittent -- worse than one that stays dead.
  ExcludedMiscompare,
};

// Overall system mode, in decreasing order of confidence.
enum class SystemMode : uint8_t {
  Normal3ch = 0,   // 3 usable channels, full 2oo3 protection
  Degraded2ch,     // 2 usable channels agree; miscompare monitor only
  Degraded1ch,     // 1 usable channel; output is unvalidated, low confidence
  DualDisagree,    // 2 usable channels disagree; cannot arbitrate
  Failed,          // no usable channels
};

// One channel's input for a single cycle, in engineering units.
struct ChannelInput {
  float value;
  bool sample_valid;  // false = no sample this cycle (dropout / timeout)
};

struct Config {
  float min_valid;               // engineering-unit plausibility band
  float max_valid;
  float disagree_threshold;      // max legitimate channel-to-channel delta
  uint8_t fault_persistence;     // consecutive bad cycles before exclusion
  uint8_t reinstate_persistence; // consecutive good cycles before a
                                 // dropped-out channel is readmitted
  uint32_t max_hold_ms;          // max age of a held last-good output
};

struct ChannelStatus {
  ChannelState state;
  uint8_t dropout_count;    // consecutive missing samples
  uint8_t range_count;      // consecutive out-of-range samples
  uint8_t miscompare_count; // consecutive cycles voted against by the pair
  uint8_t good_count;       // consecutive good cycles (reinstatement)
  float last_value;         // last received value (for logging even when bad)
  bool used_in_vote;        // contributed to this cycle's output
};

struct VoteResult {
  float value;
  bool valid;            // false = output must not be trusted
  SystemMode mode;
  bool used_hold;        // value is a held (stale) last-good output
  bool outlier_pending;  // an outlier is being counted but not yet excluded
  bool three_way_split;  // all three usable channels mutually disagree
  uint8_t used_mask;     // bit i set = channel i contributed to the value
  uint8_t used_count;
};

class Voter {
 public:
  explicit Voter(const Config& cfg);

  // Run one voting cycle. Call at a fixed rate; now_ms is the current
  // timestamp (monotonic, e.g. millis() on target).
  VoteResult update(const ChannelInput inputs[kNumChannels], uint32_t now_ms);

  const ChannelStatus& channel(uint8_t i) const { return st_[i]; }

  // Clear latched exclusions and all counters (operator maintenance action).
  void reset();

 private:
  // Returns true if the channel's sample is usable in this cycle's vote.
  bool updateChannelHealth(uint8_t i, const ChannelInput& in);
  void holdLastGood(VoteResult& r, uint32_t now_ms) const;
  void recordFreshOutput(const VoteResult& r, uint32_t now_ms);

  Config cfg_;
  ChannelStatus st_[kNumChannels];
  float last_good_value_;
  uint32_t last_good_ms_;
  bool has_last_good_;
};

// Short human-readable codes for logging (fit in a CSV field).
const char* channelStateName(ChannelState s);
const char* systemModeName(SystemMode m);

}  // namespace voting

#endif  // VOTING_CORE_H
