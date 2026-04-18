#pragma once

#include <stdint.h>
#include "Application/Data/fsm.hpp"
#include "Application/Kalman/AppLayer/kalman_sensor_types.hpp"

namespace app {

struct EstimatorOutput {
  bool altitude_valid = false;
  bool velocity_valid = false;
  bool eskf_valid = false;
  bool shadow_valid = false;
  float altitude_m = 0.0f;
  float vertical_velocity_mps = 0.0f;
  float shadow_velocity_down_mps = 0.0f;
  float body_accel_x_mps2 = 0.0f;
  uint32_t last_update_ms = 0;

  bool attitude_valid = false;
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float position_ned[3] = {0.0f, 0.0f, 0.0f};
  float velocity_ned[3] = {0.0f, 0.0f, 0.0f};
};

class IStateEstimator {
 public:
  virtual ~IStateEstimator() = default;
  virtual void reset() = 0;
  virtual void onLiftoff(uint32_t liftoff_ms) = 0;
  virtual void processImuBatch(const ImuBatch& batch) = 0;
  virtual void processBaroBatch(const BaroBatch& batch) = 0;
  virtual EstimatorOutput output() const = 0;

  virtual void processGpsSample(const sensors::gnss::GnssSample& sample) {
    (void)sample;
  }
  virtual void processMagSample(const sensors::mmc5983ma::MmcSample& sample) {
    (void)sample;
  }
  virtual void onTick(uint64_t now_us) { (void)now_us; }
  virtual void onFlightStateChange(flight_computer::State state,
                                   uint8_t reason = 0) {
    (void)state;
    (void)reason;
  }
};

class NullEstimator : public IStateEstimator {
 public:
  void reset() override;
  void onLiftoff(uint32_t liftoff_ms) override;
  void processImuBatch(const ImuBatch& batch) override;
  void processBaroBatch(const BaroBatch& batch) override;
  EstimatorOutput output() const override;

 private:
  static float pressureToAltitude(float pressure_pa);

  EstimatorOutput out_{};
  float last_altitude_m_ = 0.0f;
  uint64_t last_altitude_us_ = 0;
  uint32_t liftoff_ms_ = 0;
  bool liftoff_seen_ = false;
};

}  // namespace app
