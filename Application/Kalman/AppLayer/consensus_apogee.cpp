#include "Application/Kalman/AppLayer/apogee_algorithm.hpp"

#include <cmath>
#include <memory>

namespace app {

namespace consensus {

constexpr float kHighAccelLockoutMps2 = 5.0f * 9.81f;
constexpr uint32_t kMinBurnDurationMs = 2000;
constexpr uint32_t kShadowTimeoutMs = 1500;
constexpr uint32_t kShadowClearResetMs = 500;
constexpr float kScenarioDNearZeroThresholdMps = -2.0f;
constexpr float kDivergedNearZeroThresholdMps = -20.0f;
constexpr float kDivergedDirectDetectThresholdMps = 1.0f;

}  // namespace consensus

class ConsensusApogeeDetector : public IApogeeAlgorithm {
 public:
  ConsensusApogeeDetector() = default;

  void reset() override {
    armed_ = false;
    latched_ = false;
    liftoff_ms_ = 0;
    shadow_timer_start_ms_ = 0;
    shadow_timer_active_ = false;
    shadow_clear_start_ms_ = 0;
    shadow_clear_pending_ = false;
    diverged_timer_mode_ = false;
  }

  void arm(uint32_t liftoff_ms) override {
    reset();
    liftoff_ms_ = liftoff_ms;
    armed_ = true;
  }

  bool armed() const override { return armed_; }

  ApogeeVerdict update(uint32_t now_ms, const ApogeeInput& input) override {
    if (!armed_ || latched_) {
      return {};
    }

    if (!isDeploymentAllowed(input)) {
      resetShadowTimerNow();
      return {};
    }

    const bool eskf_apogee = input.eskf_valid && (input.eskf_velocity_down_mps > 0);
    const bool shadow_apogee = input.shadow_valid && (input.shadow_velocity_down_mps > 0);

    if (input.eskf_diverged || !input.eskf_valid) {
      if (input.shadow_valid &&
          input.shadow_velocity_down_mps > consensus::kDivergedDirectDetectThresholdMps) {
        resetShadowTimerNow();
        return trigger(now_ms, input, "ESKF diverged, Shadow-only");
      }
      if (input.shadow_velocity_down_mps <=
          consensus::kDivergedNearZeroThresholdMps) {
        resetShadowTimerNow();
        return {};
      }
      if (!shadow_timer_active_) {
        diverged_timer_mode_ = true;
        shadow_timer_start_ms_ = now_ms;
        shadow_timer_active_ = true;
        return waiting(now_ms, input, "ESKF diverged, near-zero timer started");
      }
      if (!diverged_timer_mode_) {
        diverged_timer_mode_ = true;
        shadow_timer_start_ms_ = now_ms;
        return waiting(now_ms, input, "ESKF diverged, near-zero timer started");
      }
      if ((now_ms - shadow_timer_start_ms_) >= consensus::kShadowTimeoutMs) {
        return trigger(now_ms, input, "ESKF diverged, near-zero timeout");
      }
      return waiting(now_ms, input, "ESKF diverged, waiting near-zero timeout");
    }

    if (eskf_apogee && shadow_apogee) {
      resetShadowTimerNow();
      return trigger(now_ms, input, "Consensus");
    }

    if (eskf_apogee && !shadow_apogee) {
      resetShadowTimerNow();
      if (input.shadow_velocity_down_mps > consensus::kScenarioDNearZeroThresholdMps) {
        return trigger(now_ms, input, "ESKF early, Shadow near-zero");
      }
      return vetoed(now_ms, input, "ESKF early, Shadow veto");
    }

    if (shadow_apogee && !eskf_apogee) {
      shadow_clear_pending_ = false;
      return handleShadowTimer(now_ms, input);
    }

    maybeResetShadowTimerOnSustainedClear(now_ms);
    return {};
  }

  const char* name() const override { return "Consensus"; }

 private:
  bool armed_ = false;
  bool latched_ = false;
  uint32_t liftoff_ms_ = 0;

  uint32_t shadow_timer_start_ms_ = 0;
  bool shadow_timer_active_ = false;
  uint32_t shadow_clear_start_ms_ = 0;
  bool shadow_clear_pending_ = false;
  bool diverged_timer_mode_ = false;

  bool isDeploymentAllowed(const ApogeeInput& input) const {
    if (std::fabs(input.body_accel_x_mps2) > consensus::kHighAccelLockoutMps2) {
      return false;
    }
    if (!input.is_coast_phase) {
      return false;
    }
    if (input.time_since_liftoff_ms < consensus::kMinBurnDurationMs) {
      return false;
    }
    return true;
  }

  ApogeeVerdict handleShadowTimer(uint32_t now_ms, const ApogeeInput& input) {
    if (!shadow_timer_active_ || diverged_timer_mode_) {
      shadow_timer_start_ms_ = now_ms;
      shadow_timer_active_ = true;
      diverged_timer_mode_ = false;
      return waiting(now_ms, input, "Shadow early, timer started");
    }

    const uint32_t elapsed = now_ms - shadow_timer_start_ms_;
    if (elapsed >= consensus::kShadowTimeoutMs) {
      return trigger(now_ms, input, "Shadow timeout override");
    }

    return waiting(now_ms, input, "Shadow early, waiting for ESKF");
  }

  void maybeResetShadowTimerOnSustainedClear(uint32_t now_ms) {
    if (!shadow_timer_active_) {
      shadow_clear_pending_ = false;
      return;
    }
    if (!shadow_clear_pending_) {
      shadow_clear_start_ms_ = now_ms;
      shadow_clear_pending_ = true;
      return;
    }
    const uint32_t clear_elapsed = now_ms - shadow_clear_start_ms_;
    if (clear_elapsed >= consensus::kShadowClearResetMs) {
      resetShadowTimerNow();
    }
  }

  void resetShadowTimerNow() {
    shadow_timer_active_ = false;
    shadow_timer_start_ms_ = 0;
    shadow_clear_start_ms_ = 0;
    shadow_clear_pending_ = false;
    diverged_timer_mode_ = false;
  }

  ApogeeVerdict trigger(uint32_t now_ms, const ApogeeInput& input,
                        const char* reason) {
    latched_ = true;
    armed_ = false;
    return ApogeeVerdict{ApogeeVerdict::Result::Detected,
                         now_ms,
                         input.altitude_m,
                         input.eskf_valid ? input.eskf_velocity_down_mps
                                          : input.shadow_velocity_down_mps,
                         reason};
  }

  ApogeeVerdict vetoed(uint32_t now_ms, const ApogeeInput& input,
                       const char* reason) {
    return ApogeeVerdict{ApogeeVerdict::Result::Vetoed,
                         now_ms,
                         input.altitude_m,
                         input.shadow_velocity_down_mps,
                         reason};
  }

  ApogeeVerdict waiting(uint32_t now_ms, const ApogeeInput& input,
                        const char* reason) {
    return ApogeeVerdict{ApogeeVerdict::Result::WaitingTimer,
                         now_ms,
                         input.altitude_m,
                         input.shadow_velocity_down_mps,
                         reason};
  }
};

std::unique_ptr<IApogeeAlgorithm> createConsensusApogeeDetector() {
  return std::make_unique<ConsensusApogeeDetector>();
}

}  // namespace app
