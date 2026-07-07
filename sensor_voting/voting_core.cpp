// voting_core.cpp -- see voting_core.h for design notes.

#include "voting_core.h"

#include <math.h>

namespace voting {

namespace {

float median3(float a, float b, float c) {
  // max(min(a,b), min(max(a,b), c)) -- branchless-ish median of three
  float lo = (a < b) ? a : b;
  float hi = (a < b) ? b : a;
  float m = (hi < c) ? hi : c;
  return (lo > m) ? lo : m;
}

void bumpSaturating(uint8_t& counter) {
  if (counter < 255) counter++;
}

}  // namespace

Voter::Voter(const Config& cfg) : cfg_(cfg) { reset(); }

void Voter::reset() {
  for (uint8_t i = 0; i < kNumChannels; i++) {
    st_[i].state = ChannelState::Healthy;
    st_[i].dropout_count = 0;
    st_[i].range_count = 0;
    st_[i].miscompare_count = 0;
    st_[i].good_count = 0;
    st_[i].last_value = 0.0f;
    st_[i].used_in_vote = false;
  }
  last_good_value_ = 0.0f;
  last_good_ms_ = 0;
  has_last_good_ = false;
}

bool Voter::updateChannelHealth(uint8_t i, const ChannelInput& in) {
  ChannelStatus& ch = st_[i];
  const bool latched = (ch.state == ChannelState::ExcludedRange ||
                        ch.state == ChannelState::ExcludedMiscompare);

  if (!in.sample_valid) {
    if (!latched) {
      bumpSaturating(ch.dropout_count);
      ch.good_count = 0;
      if (ch.state != ChannelState::ExcludedDropout) {
        if (ch.dropout_count >= cfg_.fault_persistence) {
          ch.state = ChannelState::ExcludedDropout;
        } else if (ch.state == ChannelState::Healthy) {
          ch.state = ChannelState::Suspect;
        }
      }
    }
    return false;
  }

  // Keep tracking the raw value even on excluded channels so logs show
  // what the bad sensor is doing.
  ch.last_value = in.value;

  if (latched) return false;

  const bool in_range =
      (in.value >= cfg_.min_valid) && (in.value <= cfg_.max_valid);

  if (!in_range) {
    bumpSaturating(ch.range_count);
    ch.good_count = 0;
    if (ch.range_count >= cfg_.fault_persistence) {
      ch.state = ChannelState::ExcludedRange;  // latched
    } else if (ch.state == ChannelState::Healthy) {
      ch.state = ChannelState::Suspect;
    }
    // Even before exclusion, an out-of-range sample is garbage: never let
    // it into the vote.
    return false;
  }

  // Valid, in-range sample from here on.
  ch.range_count = 0;

  if (ch.state == ChannelState::ExcludedDropout) {
    // Reinstatement path: require sustained recovery before readmission.
    bumpSaturating(ch.good_count);
    if (ch.good_count < cfg_.reinstate_persistence) {
      return false;  // still on probation, not used yet
    }
    // Probation cleared: this sample is valid and in-range, so readmit and
    // use it in the vote this same cycle.
    ch.state = ChannelState::Healthy;
    ch.dropout_count = 0;
    ch.miscompare_count = 0;
    ch.good_count = 0;
  }

  ch.dropout_count = 0;
  if (ch.state == ChannelState::Suspect && ch.miscompare_count == 0) {
    ch.state = ChannelState::Healthy;  // transient glitch cleared
  }
  return true;
}

void Voter::holdLastGood(VoteResult& r, uint32_t now_ms) const {
  r.value = last_good_value_;
  r.used_hold = has_last_good_;
  r.valid = has_last_good_ &&
            (uint32_t)(now_ms - last_good_ms_) <= cfg_.max_hold_ms;
}

void Voter::recordFreshOutput(const VoteResult& r, uint32_t now_ms) {
  last_good_value_ = r.value;
  last_good_ms_ = now_ms;
  has_last_good_ = true;
}

VoteResult Voter::update(const ChannelInput inputs[kNumChannels],
                         uint32_t now_ms) {
  uint8_t idx[kNumChannels];
  uint8_t n = 0;
  for (uint8_t i = 0; i < kNumChannels; i++) {
    st_[i].used_in_vote = false;
    if (updateChannelHealth(i, inputs[i])) idx[n++] = i;
  }

  VoteResult r;
  r.value = 0.0f;
  r.valid = false;
  r.mode = SystemMode::Failed;
  r.used_hold = false;
  r.outlier_pending = false;
  r.three_way_split = false;
  r.used_mask = 0;
  r.used_count = n;

  const float thr = cfg_.disagree_threshold;

  if (n == 3) {
    const float v0 = inputs[idx[0]].value;
    const float v1 = inputs[idx[1]].value;
    const float v2 = inputs[idx[2]].value;
    const bool a01 = fabsf(v0 - v1) <= thr;
    const bool a02 = fabsf(v0 - v2) <= thr;
    const bool a12 = fabsf(v1 - v2) <= thr;

    // A channel is the outlier only if the OTHER pair agrees and it
    // disagrees with both of them. Anything more ambiguous produces no
    // blame -- the median is still bounded by the two non-selected values.
    int8_t outlier = -1;
    if (a12 && !a01 && !a02) outlier = 0;
    else if (a02 && !a01 && !a12) outlier = 1;
    else if (a01 && !a02 && !a12) outlier = 2;

    if (!a01 && !a02 && !a12) r.three_way_split = true;

    if (outlier >= 0) {
      ChannelStatus& bad = st_[idx[outlier]];
      bumpSaturating(bad.miscompare_count);
      if (bad.miscompare_count >= cfg_.fault_persistence) {
        bad.state = ChannelState::ExcludedMiscompare;  // latched
      } else {
        bad.state = ChannelState::Suspect;
        r.outlier_pending = true;
      }
      // The agreeing pair is vindicated this cycle.
      for (uint8_t k = 0; k < 3; k++) {
        if ((int8_t)k == outlier) continue;
        ChannelStatus& good = st_[idx[k]];
        good.miscompare_count = 0;
        if (good.state == ChannelState::Suspect && good.range_count == 0 &&
            good.dropout_count == 0) {
          good.state = ChannelState::Healthy;
        }
      }
    } else if (!r.three_way_split) {
      // Consensus (or ambiguous-but-chained agreement): nobody is blamed.
      for (uint8_t k = 0; k < 3; k++) {
        ChannelStatus& ch = st_[idx[k]];
        ch.miscompare_count = 0;
        if (ch.state == ChannelState::Suspect && ch.range_count == 0 &&
            ch.dropout_count == 0) {
          ch.state = ChannelState::Healthy;
        }
      }
    }
    // On a three-way split: no counters move. There is no evidence to
    // blame any single channel, and the mid value is still bracketed by
    // the other two readings, so it remains the least-wrong output.

    r.value = median3(v0, v1, v2);
    r.valid = true;
    r.mode = SystemMode::Normal3ch;
    for (uint8_t k = 0; k < 3; k++) {
      r.used_mask |= (uint8_t)(1u << idx[k]);
      st_[idx[k]].used_in_vote = true;
    }
    recordFreshOutput(r, now_ms);

  } else if (n == 2) {
    const float va = inputs[idx[0]].value;
    const float vb = inputs[idx[1]].value;
    if (fabsf(va - vb) <= thr) {
      r.value = 0.5f * (va + vb);
      r.valid = true;
      r.mode = SystemMode::Degraded2ch;
      r.used_mask = (uint8_t)((1u << idx[0]) | (1u << idx[1]));
      st_[idx[0]].used_in_vote = true;
      st_[idx[1]].used_in_vote = true;
      recordFreshOutput(r, now_ms);
    } else {
      // Two channels, no referee: we deliberately do NOT guess which one
      // is right. Hold the last good output for a bounded time, then
      // declare the output invalid.
      r.mode = SystemMode::DualDisagree;
      holdLastGood(r, now_ms);
    }

  } else if (n == 1) {
    r.value = inputs[idx[0]].value;
    r.valid = true;  // usable, but consumer must treat mode as low confidence
    r.mode = SystemMode::Degraded1ch;
    r.used_mask = (uint8_t)(1u << idx[0]);
    st_[idx[0]].used_in_vote = true;
    recordFreshOutput(r, now_ms);

  } else {
    r.mode = SystemMode::Failed;
    holdLastGood(r, now_ms);
  }

  return r;
}

const char* channelStateName(ChannelState s) {
  switch (s) {
    case ChannelState::Healthy:            return "OK";
    case ChannelState::Suspect:            return "SUS";
    case ChannelState::ExcludedDropout:    return "X-DROP";
    case ChannelState::ExcludedRange:      return "X-RNG";
    case ChannelState::ExcludedMiscompare: return "X-CMP";
  }
  return "?";
}

const char* systemModeName(SystemMode m) {
  switch (m) {
    case SystemMode::Normal3ch:    return "3CH";
    case SystemMode::Degraded2ch:  return "2CH";
    case SystemMode::Degraded1ch:  return "1CH";
    case SystemMode::DualDisagree: return "DISAGREE";
    case SystemMode::Failed:       return "FAIL";
  }
  return "?";
}

}  // namespace voting
