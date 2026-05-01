#pragma once

#include <stdint.h>

namespace app {

struct ApogeeInput {
  float eskf_velocity_down_mps = 0.0f;
  bool eskf_valid = false;
  bool eskf_diverged = false;

  float shadow_velocity_down_mps = 0.0f;
  bool shadow_valid = false;

  float body_accel_x_mps2 = 0.0f;
  float altitude_m = 0.0f;

  bool is_coast_phase = false;
  uint32_t time_since_liftoff_ms = 0;
};

struct ApogeeVerdict {
  enum class Result : uint8_t {
    NoDetection = 0,
    Detected = 1,
    Vetoed = 2,
    WaitingTimer = 3,
  };

  Result result = Result::NoDetection;
  uint32_t event_ms = 0;
  float altitude_m = 0.0f;
  float velocity_mps = 0.0f;
  const char* reason = nullptr;
};

class IApogeeAlgorithm {
 public:
  virtual ~IApogeeAlgorithm() = default;
  virtual void reset() = 0;
  virtual void arm(uint32_t liftoff_ms) = 0;
  virtual bool armed() const = 0;
  virtual ApogeeVerdict update(uint32_t now_ms, const ApogeeInput& input) = 0;
  virtual const char* name() const = 0;
};

}  // namespace app
