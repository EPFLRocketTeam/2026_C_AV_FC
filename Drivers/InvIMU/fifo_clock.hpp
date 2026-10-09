#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace Drivers::InvIMU {

// Map the IMU's 16-bit, nominal-1us counter to the application's monotonic
// clock. The observation belongs to FIFO_COUNT, NOT an earlier DRDY interrupt
// or the end of the SPI transfer. FIFO reads remove the oldest packets, so
// unread packets (including the erratum's retained last packet) must be aged
// out of that observation. FSYNC tagging is not an external sampling clock.
//
// A bounded alpha/beta servo estimates both phase and oscillator rate. Phase
// noise from polling is at most roughly one sample period; a half-period
// midpoint avoids giving every sensor a different positive timestamp bias.
// This helper has no HAL dependency and is exercised by deterministic tests.
class FifoClock {
 public:
  struct Burst {
    uint64_t first_us;
    float tick_us;
  };

  void reset(float period_ticks) {
    *this = FifoClock{};
    period_ticks_ = period_ticks;
  }

  Burst map(uint16_t first_raw, uint16_t last_raw, uint32_t span_ticks,
            uint16_t unread_frames, uint64_t count_time_us) {
    const auto age = rounded((static_cast<float>(unread_frames) + 0.5f) *
                             period_ticks_ * tick_us_);
    const uint64_t observed_end = count_time_us > age ? count_time_us - age : 0;
    if (!initialized_) {
      initialized_ = true;
      const uint64_t span_us = rounded(static_cast<float>(span_ticks) * tick_us_);
      last_us_ = std::max(observed_end, span_us + 1);
      last_raw_ = last_raw;
      return {last_us_ - span_us, tick_us_};
    }

    uint64_t gap_ticks = static_cast<uint16_t>(first_raw - last_raw_);
    // A normal wrap is already handled by uint16 subtraction. Only recover
    // additional complete wraps when host time proves that packets were lost
    // across a long service interruption. Account for this burst's span too.
    const uint64_t elapsed_ticks = gap_ticks + span_ticks;
    if (observed_end > last_us_) {
      const float expected_ticks = static_cast<float>(observed_end - last_us_) / tick_us_;
      if (expected_ticks > static_cast<float>(elapsed_ticks) + 32768.0f) {
        const auto wraps = static_cast<uint32_t>(
            (expected_ticks - static_cast<float>(elapsed_ticks) + 32768.0f) / 65536.0f);
        gap_ticks += static_cast<uint64_t>(wraps) * 65536u;
        recovered_wraps_ += wraps;
      }
    }

    const float elapsed_us = static_cast<float>(gap_ticks + span_ticks) * tick_us_;
    const uint64_t predicted_end = last_us_ + rounded(elapsed_us);
    const int64_t innovation = static_cast<int64_t>(observed_end) -
                               static_cast<int64_t>(predicted_end);
    error_us_ = static_cast<int32_t>(std::clamp<int64_t>(innovation, INT32_MIN, INT32_MAX));
    const float old_tick_us = tick_us_;
    int64_t correction;
    if (gap_ticks * tick_us_ >= 32768.0f) {
      // There is a genuine gap, not adjacent samples: phase reacquisition is
      // safe without compressing/reversing a normal sample interval. Don't
      // train oscillator rate on an ambiguous/multi-wrap interruption.
      correction = innovation;
      ++reacquisitions_;
    } else {
      const int64_t limit = std::max<int64_t>(1, std::min<int64_t>(32,
          rounded(static_cast<float>(gap_ticks) * tick_us_ * 0.25f)));
      correction = std::clamp<int64_t>(innovation / 8, -limit, limit);
      if (elapsed_us >= 1000.0f) {
        tick_us_ = std::clamp(tick_us_ + static_cast<float>(innovation) /
                             (256.0f * elapsed_us), 0.9f, 1.1f);
      }
    }

    const uint64_t span_us = rounded(static_cast<float>(span_ticks) * old_tick_us);
    const int64_t proposed_first = static_cast<int64_t>(predicted_end) +
                                   correction - static_cast<int64_t>(span_us);
    // Counter/configuration faults must never move the application's time
    // backwards. The driver separately flags this repair for diagnostics.
    const uint64_t first_us = static_cast<uint64_t>(
        std::max<int64_t>(proposed_first, static_cast<int64_t>(last_us_) + 1));
    last_us_ = first_us + span_us;
    last_raw_ = last_raw;
    return {first_us, old_tick_us};
  }

  int32_t errorUs() const { return error_us_; }
  int32_t scalePpm() const { return static_cast<int32_t>((tick_us_ - 1.0f) * 1e6f); }
  uint32_t recoveredWraps() const { return recovered_wraps_; }
  uint32_t reacquisitions() const { return reacquisitions_; }

 private:
  static uint64_t rounded(float value) { return static_cast<uint64_t>(value + 0.5f); }
  float period_ticks_ = 156.25f;
  float tick_us_ = 1.0f;
  bool initialized_ = false;
  uint16_t last_raw_ = 0;
  uint64_t last_us_ = 0;
  int32_t error_us_ = 0;
  uint32_t recovered_wraps_ = 0;
  uint32_t reacquisitions_ = 0;
};
}  // namespace Drivers::InvIMU
