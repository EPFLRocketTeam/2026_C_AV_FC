#include "Application/Kalman/AppLayer/state_estimator.hpp"

#include <math.h>

namespace app {

void NullEstimator::reset() {
  out_ = {};
  last_altitude_m_ = 0.0f;
  last_altitude_us_ = 0;
  liftoff_ms_ = 0;
  liftoff_seen_ = false;
}

void NullEstimator::onLiftoff(uint32_t liftoff_ms) {
  liftoff_ms_ = liftoff_ms;
  liftoff_seen_ = true;
}

void NullEstimator::processImuBatch(const ImuBatch& batch) {
  (void)batch;
}

void NullEstimator::processBaroBatch(const BaroBatch& batch) {
  if (!batch.data || batch.count == 0) return;

  const uint32_t dt_us = batch.dt_us ? batch.dt_us : 1000;
  for (size_t i = 0; i < batch.count; ++i) {
    const uint64_t sample_time =
        batch.t0_us + static_cast<uint64_t>(i) * static_cast<uint64_t>(dt_us);
    const float altitude_m = pressureToAltitude(batch.data[i].pressurePa);
    out_.altitude_m = altitude_m;
    out_.altitude_valid = true;

    if (last_altitude_us_ != 0 && sample_time > last_altitude_us_) {
      const float dt_s =
          static_cast<float>(sample_time - last_altitude_us_) / 1e6f;
      if (dt_s > 0.0f) {
        out_.vertical_velocity_mps = (altitude_m - last_altitude_m_) / dt_s;
        out_.velocity_valid = true;
      }
    }

    last_altitude_m_ = altitude_m;
    last_altitude_us_ = sample_time;
    out_.last_update_ms = static_cast<uint32_t>(sample_time / 1000ULL);
  }

  if (liftoff_seen_ && out_.last_update_ms < liftoff_ms_) {
    out_.last_update_ms = liftoff_ms_;
  }
}

EstimatorOutput NullEstimator::output() const { return out_; }

float NullEstimator::pressureToAltitude(float pressure_pa) {
  const float safe_pressure = pressure_pa > 1.0f ? pressure_pa : 101325.0f;
  const float ratio = safe_pressure / 101325.0f;
  return 44330.0f * (1.0f - powf(ratio, 0.1903f));
}

}  // namespace app
