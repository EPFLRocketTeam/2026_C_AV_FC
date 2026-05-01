// Virtual IMU Preprocessing Pipeline Implementation - Multi-Sensor Voting
// Part of Phase 2: ESKF Pre-Processing Layer
//
// See Kalman_filter_planification.md Section 2.2 for algorithm details.

#include "virtual_imu.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace eskf {

static_assert(ESKF_CENTRAL_DIFF_ORDER == 7,
              "VirtualImu currently implements only a 7-point central-difference kernel");

namespace {

inline eskf_scalar vectorNorm3(const eskf_scalar v[3]) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

} // namespace

// ============================================================
// Configuration & Reset
// ============================================================

void VirtualImu::configure(const VirtualImuConfig &cfg) {
  cfg_ = cfg;

  // Pre-compute PCB center (centroid of all enabled sensor positions)
  // This is I1 optimization: compute once instead of every sample
  pcb_center_[0] = pcb_center_[1] = pcb_center_[2] = 0;
  size_t enabled_count = 0;
  for (size_t i = 0; i < cfg_.imu_count; ++i) {
    if (cfg_.imus[i].enabled) {
      for (int j = 0; j < 3; ++j) {
        pcb_center_[j] += cfg_.imus[i].position[j];
      }
      enabled_count++;
    }
  }
  if (enabled_count > 0) {
    for (int j = 0; j < 3; ++j) {
      pcb_center_[j] /= enabled_count;
    }
  }

  reset();
}

void VirtualImu::setRuntimePolicy(const VirtualImuRuntimePolicy &policy) {
  runtime_policy_ = policy;
  if (runtime_policy_.soft_threshold_scale < 1.0) {
    runtime_policy_.soft_threshold_scale = 1.0;
  }
}

void VirtualImu::reset() {
  history_count_ = 0;
  std::memset(omega_history_, 0, sizeof(omega_history_));
  std::memset(accel_history_, 0, sizeof(accel_history_));
  std::memset(imu_accel_body_history_, 0, sizeof(imu_accel_body_history_));
  std::memset(imu_gyro_body_history_, 0, sizeof(imu_gyro_body_history_));
  std::memset(timestamp_history_, 0, sizeof(timestamp_history_));
  std::memset(status_history_, 0, sizeof(status_history_));
  std::memset(health_history_, 0, sizeof(health_history_));
  std::memset(hard_fault_counter_history_, 0,
              sizeof(hard_fault_counter_history_));
  std::memset(degraded_history_, 0, sizeof(degraded_history_));
  std::memset(salvage_history_, 0, sizeof(salvage_history_));
  std::memset(saturation_history_, 0, sizeof(saturation_history_));
  std::memset(prev_omega_, 0, sizeof(prev_omega_));
  std::memset(health_state_, 0, sizeof(health_state_));
  std::memset(hard_fault_counter_, 0, sizeof(hard_fault_counter_));
  std::memset(healthy_counter_, 0, sizeof(healthy_counter_));
  std::memset(saturation_counter_, 0, sizeof(saturation_counter_));
  std::memset(stale_counter_, 0, sizeof(stale_counter_));
  std::memset(has_prev_sample_, 0, sizeof(has_prev_sample_));
  std::memset(prev_accel_body_, 0, sizeof(prev_accel_body_));
  std::memset(prev_gyro_body_, 0, sizeof(prev_gyro_body_));
  std::memset(tare_window_count_, 0, sizeof(tare_window_count_));
  std::memset(tare_window_gyro_sum_, 0, sizeof(tare_window_gyro_sum_));
  std::memset(tare_window_accel_sum_, 0, sizeof(tare_window_accel_sum_));
  std::memset(gyro_tare_body_, 0, sizeof(gyro_tare_body_));
  std::memset(accel_tare_body_, 0, sizeof(accel_tare_body_));
  has_prev_omega_ = false;
  tare_frozen_ = false;
  runtime_policy_ = VirtualImuRuntimePolicy{};
}

size_t VirtualImu::lookAheadSamples() const {
  return cfg_.use_central_diff ? (kHistorySize / 2) : 0;
}

// ============================================================
// Helper Functions
// ============================================================

void VirtualImu::getCurrentCG(uint64_t timestamp_us, eskf_scalar cg_out[3]) const {
  if (cfg_.cg_callback) {
    cfg_.cg_callback(cfg_.cg_user_data, timestamp_us, cg_out);
  } else {
    cg_out[0] = cfg_.static_cg[0];
    cg_out[1] = cfg_.static_cg[1];
    cg_out[2] = cfg_.static_cg[2];
  }
}

void VirtualImu::rotateToBody(eskf_scalar out[3], const eskf_scalar R[3][3],
                              const eskf_scalar sensor[3]) const {
  for (int i = 0; i < 3; ++i) {
    out[i] = R[i][0] * sensor[0] + R[i][1] * sensor[1] + R[i][2] * sensor[2];
  }
}

void VirtualImu::computeMedian3D(eskf_scalar median_out[3],
                                 const eskf_scalar data[][3],
                                 const bool valid[], size_t count) const {
  // For each axis, find median of valid values
  for (int axis = 0; axis < 3; ++axis) {
    eskf_scalar values[ESKF_MAX_IMUS];
    size_t n = 0;

    for (size_t i = 0; i < count; ++i) {
      if (valid[i]) {
        values[n++] = data[i][axis];
      }
    }

    if (n == 0) {
      median_out[axis] = 0;
    } else if (n == 1) {
      median_out[axis] = values[0];
    } else {
      // Sort and take middle
      std::sort(values, values + n);
      if (n % 2 == 1) {
        median_out[axis] = values[n / 2];
      } else {
        median_out[axis] = (values[n / 2 - 1] + values[n / 2]) / 2;
      }
    }
  }
}

// ============================================================
// Voting Functions
// ============================================================

void VirtualImu::voteGyro(eskf_scalar fused_gyro[3],
                          const eskf_scalar gyro_body[][3],
                          SensorStatus statuses[], size_t count) const {
  // Build valid mask (exclude HARD_FAIL)
  bool valid[ESKF_MAX_IMUS];
  size_t valid_count = 0;
  for (size_t i = 0; i < count; ++i) {
    valid[i] = (statuses[i] == SensorStatus::OK);
    if (valid[i])
      valid_count++;
  }

  if (valid_count == 0) {
    fused_gyro[0] = fused_gyro[1] = fused_gyro[2] = 0;
    return;
  }

  // If voting enabled and >2 valid sensors, compute median and reject outliers
  // (with only 2 sensors, median is just average - outlier detection is
  // degenerate)
  if (cfg_.voting_enabled && valid_count > 2) {
    // Compute median
    eskf_scalar median[3];
    computeMedian3D(median, gyro_body, valid, count);

    // Reject outliers
    for (size_t i = 0; i < count; ++i) {
      if (valid[i]) {
        // Compute distance from median (Euclidean)
        eskf_scalar dist = 0;
        for (int j = 0; j < 3; ++j) {
          eskf_scalar d = gyro_body[i][j] - median[j];
          dist += d * d;
        }
        dist = std::sqrt(dist);

        if (dist > cfg_.gyro_voting_threshold) {
          statuses[i] = SensorStatus::SOFT_FAIL;
          valid[i] = false;
          valid_count--;
        }
      }
    }
  }

  // Fuse: average of valid sensors
  fused_gyro[0] = fused_gyro[1] = fused_gyro[2] = 0;
  size_t n = 0;
  for (size_t i = 0; i < count; ++i) {
    if (valid[i]) {
      for (int j = 0; j < 3; ++j) {
        fused_gyro[j] += gyro_body[i][j];
      }
      n++;
    }
  }
  if (n > 0) {
    for (int j = 0; j < 3; ++j) {
      fused_gyro[j] /= n;
    }
  }
}

void VirtualImu::voteAccel(const eskf_scalar accel_body[][3],
                           const eskf_scalar omega[3],
                           const eskf_scalar omega_dot_rough[3],
                           SensorStatus statuses[], size_t count) const {
  if (!cfg_.voting_enabled)
    return;

  // Build valid mask and count
  bool valid[ESKF_MAX_IMUS];
  size_t valid_count = 0;
  for (size_t i = 0; i < count; ++i) {
    valid[i] = (statuses[i] == SensorStatus::OK);
    if (valid[i])
      valid_count++;
  }

  if (valid_count <= 1)
    return; // No voting with 0 or 1 sensor

  // Use pre-computed PCB center (computed in configure())
  // Project each accel to PCB center using rough omega_dot
  eskf_scalar accel_test[ESKF_MAX_IMUS][3];
  for (size_t i = 0; i < count; ++i) {
    if (!valid[i])
      continue;

    // Lever arm from sensor i to PCB center
    eskf_scalar r[3];
    for (int j = 0; j < 3; ++j) {
      r[j] = pcb_center_[j] - cfg_.imus[i].position[j];
    }

    // ω × r
    eskf_scalar omega_cross_r[3];
    omega_cross_r[0] = omega[1] * r[2] - omega[2] * r[1];
    omega_cross_r[1] = omega[2] * r[0] - omega[0] * r[2];
    omega_cross_r[2] = omega[0] * r[1] - omega[1] * r[0];

    // ω × (ω × r) - centripetal
    eskf_scalar centripetal[3];
    centripetal[0] = omega[1] * omega_cross_r[2] - omega[2] * omega_cross_r[1];
    centripetal[1] = omega[2] * omega_cross_r[0] - omega[0] * omega_cross_r[2];
    centripetal[2] = omega[0] * omega_cross_r[1] - omega[1] * omega_cross_r[0];

    // ω̇ × r - tangential
    eskf_scalar tangential[3];
    tangential[0] = omega_dot_rough[1] * r[2] - omega_dot_rough[2] * r[1];
    tangential[1] = omega_dot_rough[2] * r[0] - omega_dot_rough[0] * r[2];
    tangential[2] = omega_dot_rough[0] * r[1] - omega_dot_rough[1] * r[0];

    // Project: a_test = a_raw - tangential - centripetal
    for (int j = 0; j < 3; ++j) {
      accel_test[i][j] = accel_body[i][j] - tangential[j] - centripetal[j];
    }
  }

  // Compute median of projected values
  eskf_scalar median[3];
  computeMedian3D(median, accel_test, valid, count);

  // Reject outliers
  for (size_t i = 0; i < count; ++i) {
    if (valid[i]) {
      eskf_scalar dist = 0;
      for (int j = 0; j < 3; ++j) {
        eskf_scalar d = accel_test[i][j] - median[j];
        dist += d * d;
      }
      dist = std::sqrt(dist);

      if (dist > cfg_.accel_voting_threshold) {
        statuses[i] = SensorStatus::SOFT_FAIL;
      }
    }
  }

  // Note: accel_test values are DISCARDED - they contain noise from rough
  // derivative
}

// ============================================================
// Centroid & Averaging
// ============================================================

void VirtualImu::computeEffectiveCentroid(eskf_scalar centroid_out[3],
                                          const SensorStatus statuses[],
                                          size_t count) const {
  centroid_out[0] = centroid_out[1] = centroid_out[2] = 0;
  size_t n = 0;

  for (size_t i = 0; i < count; ++i) {
    if (statuses[i] == SensorStatus::OK && cfg_.imus[i].enabled) {
      for (int j = 0; j < 3; ++j) {
        centroid_out[j] += cfg_.imus[i].position[j];
      }
      n++;
    }
  }

  if (n > 0) {
    for (int j = 0; j < 3; ++j) {
      centroid_out[j] /= n;
    }
  }
}

void VirtualImu::averageValidSensors(eskf_scalar avg_accel[3],
                                     eskf_scalar avg_gyro[3],
                                     const eskf_scalar accel_body[][3],
                                     const eskf_scalar gyro_body[][3],
                                     const SensorStatus statuses[],
                                     size_t count) const {
  avg_accel[0] = avg_accel[1] = avg_accel[2] = 0;
  avg_gyro[0] = avg_gyro[1] = avg_gyro[2] = 0;
  size_t n = 0;

  for (size_t i = 0; i < count; ++i) {
    if (statuses[i] == SensorStatus::OK) {
      for (int j = 0; j < 3; ++j) {
        avg_accel[j] += accel_body[i][j];
        avg_gyro[j] += gyro_body[i][j];
      }
      n++;
    }
  }

  if (n > 0) {
    for (int j = 0; j < 3; ++j) {
      avg_accel[j] /= n;
      avg_gyro[j] /= n;
    }
  }
}

// ============================================================
// Derivative Computation
// ============================================================

void VirtualImu::computeOmegaDotRough(eskf_scalar omega_dot[3],
                                      const eskf_scalar omega_current[3],
                                      eskf_scalar dt) {
  if (has_prev_omega_ && dt > 0) {
    for (int i = 0; i < 3; ++i) {
      omega_dot[i] = (omega_current[i] - prev_omega_[i]) / dt;
    }
  } else {
    omega_dot[0] = omega_dot[1] = omega_dot[2] = 0;
  }

  // Store for next iteration
  for (int i = 0; i < 3; ++i) {
    prev_omega_[i] = omega_current[i];
  }
  has_prev_omega_ = true;
}

void VirtualImu::computeOmegaDotSmooth(eskf_scalar omega_dot[3],
                                       eskf_scalar dt) const {
  constexpr eskf_scalar kMinDt = 1e-6; // 1 microsecond
  if (cfg_.use_central_diff) {
    // 7-point centered Savitzky-Golay first-derivative (window=7, poly=3)
    // Produces derivative at the center sample (lag = 3 samples).
    // Coefficients: [-3, -2, -1, 0, 1, 2, 3] / (28 * dt)
    if (history_count_ >= kHistorySize && dt > kMinDt) {
      static constexpr eskf_scalar kCoeff[7] = {-3, -2, -1, 0, 1, 2, 3};
      const eskf_scalar scale =
          static_cast<eskf_scalar>(1.0) / (static_cast<eskf_scalar>(28.0) * dt);
      for (int i = 0; i < 3; ++i) {
        eskf_scalar acc = 0;
        for (size_t k = 0; k < kHistorySize; ++k) {
          acc += kCoeff[k] * omega_history_[k][i];
        }
        omega_dot[i] = acc * scale;
      }
    } else {
      omega_dot[0] = omega_dot[1] = omega_dot[2] = 0;
    }
  } else {
    // Backward difference: (ω[n] - ω[n-2]) / (2*dt)
    if (history_count_ >= 3 && dt > kMinDt) {
      const size_t last = history_count_ - 1;
      const eskf_scalar dt2 = static_cast<eskf_scalar>(2.0) * dt;
      for (int i = 0; i < 3; ++i) {
        omega_dot[i] =
            (omega_history_[last][i] - omega_history_[last - 2][i]) / dt2;
      }
    } else {
      omega_dot[0] = omega_dot[1] = omega_dot[2] = 0;
    }
  }
}

// ============================================================
// Lever-Arm Correction
// ============================================================

void VirtualImu::applyLeverArmCorrection(eskf_scalar accel_cg[3],
                                         const eskf_scalar accel_sens[3],
                                         const eskf_scalar omega[3],
                                         const eskf_scalar omega_dot[3],
                                         const eskf_scalar lever_arm[3],
                                         eskf_scalar tangential_out[3],
                                         eskf_scalar centripetal_out[3],
                                         eskf_scalar correction_unclamped_out[3],
                                         eskf_scalar correction_applied_out[3],
                                         bool *correction_clipped) const {
  // a_cg = a_sens - (ω̇ × r + ω × (ω × r))

  // ω × r
  eskf_scalar omega_cross_r[3];
  omega_cross_r[0] = omega[1] * lever_arm[2] - omega[2] * lever_arm[1];
  omega_cross_r[1] = omega[2] * lever_arm[0] - omega[0] * lever_arm[2];
  omega_cross_r[2] = omega[0] * lever_arm[1] - omega[1] * lever_arm[0];

  // ω × (ω × r) - centripetal
  eskf_scalar centripetal[3];
  centripetal[0] = omega[1] * omega_cross_r[2] - omega[2] * omega_cross_r[1];
  centripetal[1] = omega[2] * omega_cross_r[0] - omega[0] * omega_cross_r[2];
  centripetal[2] = omega[0] * omega_cross_r[1] - omega[1] * omega_cross_r[0];

  // ω̇ × r - tangential
  eskf_scalar tangential[3];
  tangential[0] = omega_dot[1] * lever_arm[2] - omega_dot[2] * lever_arm[1];
  tangential[1] = omega_dot[2] * lever_arm[0] - omega_dot[0] * lever_arm[2];
  tangential[2] = omega_dot[0] * lever_arm[1] - omega_dot[1] * lever_arm[0];

  eskf_scalar correction_unclamped[3];
  eskf_scalar correction_applied[3];
  for (int i = 0; i < 3; ++i) {
    correction_unclamped[i] = tangential[i] + centripetal[i];
    correction_applied[i] = correction_unclamped[i];
  }

  bool clipped = false;
  const eskf_scalar max_corr_norm = cfg_.lever_arm_correction_max_norm;
  if (std::isfinite(max_corr_norm) && max_corr_norm > 0) {
    const eskf_scalar corr_norm = vectorNorm3(correction_applied);
    if (std::isfinite(corr_norm) && corr_norm > max_corr_norm &&
        corr_norm > static_cast<eskf_scalar>(1e-9)) {
      const eskf_scalar scale = max_corr_norm / corr_norm;
      for (int i = 0; i < 3; ++i) {
        correction_applied[i] *= scale;
      }
      clipped = true;
    }
  }

  // Apply correction
  for (int i = 0; i < 3; ++i) {
    accel_cg[i] = accel_sens[i] - correction_applied[i];
  }

  if (tangential_out) {
    for (int i = 0; i < 3; ++i) {
      tangential_out[i] = tangential[i];
    }
  }
  if (centripetal_out) {
    for (int i = 0; i < 3; ++i) {
      centripetal_out[i] = centripetal[i];
    }
  }
  if (correction_unclamped_out) {
    for (int i = 0; i < 3; ++i) {
      correction_unclamped_out[i] = correction_unclamped[i];
    }
  }
  if (correction_applied_out) {
    for (int i = 0; i < 3; ++i) {
      correction_applied_out[i] = correction_applied[i];
    }
  }
  if (correction_clipped) {
    *correction_clipped = clipped;
  }
}

// ============================================================
// Main Processing
// ============================================================

size_t VirtualImu::process(const eskf_sensor_t *const accel_data[ESKF_MAX_IMUS],
                           const eskf_sensor_t *const gyro_data[ESKF_MAX_IMUS],
                           const eskf_scalar *const temp_data[ESKF_MAX_IMUS],
                           const SensorStatus statuses_in[ESKF_MAX_IMUS],
                           size_t count, uint64_t t0_us, VirtualImuOutput *out,
                           size_t out_cap, uint32_t sample_dt_us,
                           const eskf_scalar *gyro_bias_body) {

  if (count == 0 || out_cap == 0 || cfg_.imu_count == 0)
    return 0;

  // Compute dt from provided sample period (calculated by caller from actual
  // timestamps)
  const eskf_scalar dt = static_cast<eskf_scalar>(sample_dt_us) * 1e-6;
  const eskf_scalar soft_scale =
      (runtime_policy_.soft_threshold_scale > 1.0)
          ? runtime_policy_.soft_threshold_scale
          : static_cast<eskf_scalar>(1.0);
  const eskf_scalar soft_gyro_threshold = cfg_.gyro_voting_threshold * soft_scale;
  const eskf_scalar soft_accel_threshold =
      cfg_.accel_voting_threshold * soft_scale;
  const eskf_scalar accel_stale_eps =
      (std::isfinite(cfg_.stale_accel_delta_threshold) &&
       cfg_.stale_accel_delta_threshold > 0)
        ? cfg_.stale_accel_delta_threshold
        : static_cast<eskf_scalar>(0);
  const eskf_scalar gyro_stale_eps =
      (std::isfinite(cfg_.stale_gyro_delta_threshold) &&
       cfg_.stale_gyro_delta_threshold > 0)
        ? cfg_.stale_gyro_delta_threshold
        : static_cast<eskf_scalar>(0);
  const eskf_scalar accel_stale_eps_sq = accel_stale_eps * accel_stale_eps;
  const eskf_scalar gyro_stale_eps_sq = gyro_stale_eps * gyro_stale_eps;
  const bool tare_enabled =
      cfg_.enable_preflight_tare && (cfg_.tare_window_samples > 0);
  const eskf_scalar tare_gravity =
      (std::isfinite(cfg_.tare_accel_gravity) && cfg_.tare_accel_gravity > 0)
          ? cfg_.tare_accel_gravity
          : constants::kGravityLocal;

  if (tare_enabled && runtime_policy_.in_flight) {
    tare_frozen_ = true;
  }

  size_t out_count = 0;

  for (size_t n = 0; n < count && out_count < out_cap; ++n) {
    // Copy statuses (we modify during voting and health-state transitions)
    SensorStatus statuses[ESKF_MAX_IMUS];
    bool enabled_mask[ESKF_MAX_IMUS] = {};
    bool has_data_mask[ESKF_MAX_IMUS] = {};
    bool input_hard_mask[ESKF_MAX_IMUS] = {};
    bool soft_reject_mask[ESKF_MAX_IMUS] = {};
    bool hard_fault_mask[ESKF_MAX_IMUS] = {};
    bool pre_vote_valid[ESKF_MAX_IMUS] = {};
    bool fused_mask[ESKF_MAX_IMUS] = {};
    bool saturation_mask[ESKF_MAX_IMUS] = {};
    bool stale_fault_mask[ESKF_MAX_IMUS] = {};
    eskf_scalar gyro_dist[ESKF_MAX_IMUS] = {};
    eskf_scalar accel_dist[ESKF_MAX_IMUS] = {};

    for (size_t i = 0; i < cfg_.imu_count; ++i) {
      statuses[i] = SensorStatus::HARD_FAIL;
      enabled_mask[i] = cfg_.imus[i].enabled;
      input_hard_mask[i] = true;
    }

    for (size_t i = cfg_.imu_count; i < ESKF_MAX_IMUS; ++i) {
      statuses[i] = SensorStatus::HARD_FAIL;
    }

    // 1. Read raw data, apply calibration, rotate to body frame
    eskf_scalar gyro_body[ESKF_MAX_IMUS][3] = {};
    eskf_scalar accel_body[ESKF_MAX_IMUS][3] = {};

    for (size_t i = 0; i < cfg_.imu_count; ++i) {
      if (enabled_mask[i] && accel_data[i] && gyro_data[i]) {
        has_data_mask[i] = true;
        eskf_scalar a_raw[3] = {accel_data[i][n * 3], accel_data[i][n * 3 + 1],
                                accel_data[i][n * 3 + 2]};
        eskf_scalar g_raw[3] = {gyro_data[i][n * 3], gyro_data[i][n * 3 + 1],
                                gyro_data[i][n * 3 + 2]};

        eskf_scalar a_cal[3], g_cal[3];

        // Apply calibration if available
        if (cfg_.calibration) {
          const ImuCalibration &cal = cfg_.calibration[i];

          // Temperature delta for thermal compensation. Clamp to the
          // calibration-supported range when bounds are provided.
          eskf_scalar dT = 0;
          if (temp_data && temp_data[i]) {
            dT = temp_data[i][n] - cal.reference_temp_k;
          }
          dT = clampThermalDeltaTempToCalibrationBounds(cal, dT);

          // --- Accel Calibration ---
          // Step 1: Thermal compensation
          // Compute temperature-dependent bias using cubic polynomial
          eskf_scalar a_thermal_bias[3] = {0, 0, 0};
          if (cal.thermal_calibrated) {
            for (int j = 0; j < 3; ++j) {
              a_thermal_bias[j] = evalThermalPoly(cal.accel_thermal[j], dT);
            }
          }

          // Step 2: Remove static bias + thermal bias
          eskf_scalar a_unbiased[3];
          for (int j = 0; j < 3; ++j) {
            a_unbiased[j] = a_raw[j] - cal.accel_bias[j] - a_thermal_bias[j];
          }

          // Step 3: Apply transform matrix (scale + skew from ellipsoid
          // fitting)
          applyMatrix3(cal.accel_transform, a_unbiased, a_cal);

          // --- Gyro Calibration ---
          // Step 1: Thermal compensation for gyro
          eskf_scalar g_thermal_bias[3] = {0, 0, 0};
          if (cal.thermal_calibrated) {
            // Use same bounded dT computed for accel.
            for (int j = 0; j < 3; ++j) {
              g_thermal_bias[j] = evalThermalPoly(cal.gyro_thermal[j], dT);
            }
          }

          // Step 2: Remove constant bias + thermal bias, then apply scale
          // factor
          for (int j = 0; j < 3; ++j) {
            g_cal[j] = (g_raw[j] - cal.gyro_bias[j] - g_thermal_bias[j]) *
                       cal.gyro_scale[j];
          }

          // Step 3: Compensate gyro g-sensitivity using calibrated accel.
          // omega_corr = omega_meas - G * a_cal
          eskf_scalar g_sens_corr[3];
          applyMatrix3(cal.gyro_g_sensitivity, a_cal, g_sens_corr);
          for (int j = 0; j < 3; ++j) {
            g_cal[j] -= g_sens_corr[j];
          }
        } else {
          // No calibration - passthrough
          for (int j = 0; j < 3; ++j) {
            a_cal[j] = a_raw[j];
            g_cal[j] = g_raw[j];
          }
        }

        // Rotate calibrated values to body frame
        rotateToBody(accel_body[i], cfg_.imus[i].to_body, a_cal);
        rotateToBody(gyro_body[i], cfg_.imus[i].to_body, g_cal);

        // Input status contributes only to hard-fault observations, not to
        // soft voting thresholds.
        input_hard_mask[i] = (statuses_in[i] != SensorStatus::OK);
      } else {
        std::memset(accel_body[i], 0, sizeof(accel_body[i]));
        std::memset(gyro_body[i], 0, sizeof(gyro_body[i]));
        has_data_mask[i] = false;
        input_hard_mask[i] = true;
      }
    }

    // 1a. Preflight per-IMU windowed tare.
    // Applied after calibration/rotation and before stale detection/voting.
    if (tare_enabled) {
      if (!tare_frozen_) {
        for (size_t i = 0; i < cfg_.imu_count; ++i) {
          const bool tare_sample_ok =
              enabled_mask[i] && has_data_mask[i] && !input_hard_mask[i];
          if (!tare_sample_ok) {
            tare_window_count_[i] = 0;
            for (int axis = 0; axis < 3; ++axis) {
              tare_window_gyro_sum_[i][axis] = 0;
              tare_window_accel_sum_[i][axis] = 0;
            }
            continue;
          }

          eskf_scalar accel_bias_sample[3] = {0, 0, 0};
          eskf_scalar accel_norm_sq = 0;
          for (int axis = 0; axis < 3; ++axis) {
            accel_norm_sq += accel_body[i][axis] * accel_body[i][axis];
          }
          if (accel_norm_sq > static_cast<eskf_scalar>(1e-12)) {
            const eskf_scalar accel_norm = std::sqrt(accel_norm_sq);
            const eskf_scalar scale =
                (accel_norm - tare_gravity) / accel_norm;
            for (int axis = 0; axis < 3; ++axis) {
              accel_bias_sample[axis] = accel_body[i][axis] * scale;
            }
          }

          for (int axis = 0; axis < 3; ++axis) {
            tare_window_gyro_sum_[i][axis] += gyro_body[i][axis];
            tare_window_accel_sum_[i][axis] += accel_bias_sample[axis];
          }
          if (tare_window_count_[i] < std::numeric_limits<uint16_t>::max()) {
            tare_window_count_[i]++;
          }

          if (tare_window_count_[i] >= cfg_.tare_window_samples) {
            const eskf_scalar inv_n =
                static_cast<eskf_scalar>(1.0) /
                static_cast<eskf_scalar>(tare_window_count_[i]);
            for (int axis = 0; axis < 3; ++axis) {
              gyro_tare_body_[i][axis] = tare_window_gyro_sum_[i][axis] * inv_n;
              accel_tare_body_[i][axis] =
                  tare_window_accel_sum_[i][axis] * inv_n;
              tare_window_gyro_sum_[i][axis] = 0;
              tare_window_accel_sum_[i][axis] = 0;
            }
            tare_window_count_[i] = 0;
          }
        }
      }

      for (size_t i = 0; i < cfg_.imu_count; ++i) {
        if (!(enabled_mask[i] && has_data_mask[i])) {
          continue;
        }
        for (int axis = 0; axis < 3; ++axis) {
          accel_body[i][axis] -= accel_tare_body_[i][axis];
          gyro_body[i][axis] -= gyro_tare_body_[i][axis];
        }
      }
    }

    // 1b. Per-sensor stale/frozen measurement detection.
    // Uses calibrated + body-frame values to detect stuck-at faults.
    if (cfg_.stale_persistence_samples > 0) {
      for (size_t i = 0; i < cfg_.imu_count; ++i) {
        if (!(enabled_mask[i] && has_data_mask[i] && !input_hard_mask[i])) {
          if (!has_data_mask[i]) {
            stale_counter_[i] = 0;
            has_prev_sample_[i] = false;
          }
          continue;
        }

        bool stale_sample = false;
        if (has_prev_sample_[i]) {
          eskf_scalar accel_delta_sq = 0;
          eskf_scalar gyro_delta_sq = 0;
          for (int axis = 0; axis < 3; ++axis) {
            const eskf_scalar da =
                accel_body[i][axis] - prev_accel_body_[i][axis];
            const eskf_scalar dg = gyro_body[i][axis] - prev_gyro_body_[i][axis];
            accel_delta_sq += da * da;
            gyro_delta_sq += dg * dg;
          }
          stale_sample = (accel_delta_sq <= accel_stale_eps_sq) &&
                         (gyro_delta_sq <= gyro_stale_eps_sq);
        }

        for (int axis = 0; axis < 3; ++axis) {
          prev_accel_body_[i][axis] = accel_body[i][axis];
          prev_gyro_body_[i][axis] = gyro_body[i][axis];
        }
        has_prev_sample_[i] = true;

        if (stale_sample) {
          if (stale_counter_[i] < std::numeric_limits<uint16_t>::max()) {
            stale_counter_[i]++;
          }
          if (stale_counter_[i] >= cfg_.stale_persistence_samples) {
            stale_fault_mask[i] = true;
            hard_fault_mask[i] = true;
          }
        } else {
          stale_counter_[i] = 0;
        }
      }
    } else {
      for (size_t i = 0; i < cfg_.imu_count; ++i) {
        stale_counter_[i] = 0;
      }
    }

    // 2. Build voting eligibility from persistent health state.
    size_t pre_vote_count = 0;
    for (size_t i = 0; i < cfg_.imu_count; ++i) {
      const bool health_blocked = (health_state_[i] == SensorHealthState::INHIBITED ||
                                   health_state_[i] == SensorHealthState::RECOVERING);
      pre_vote_valid[i] = enabled_mask[i] && has_data_mask[i] && !input_hard_mask[i] &&
                          !health_blocked;
      if (pre_vote_valid[i]) {
        statuses[i] = SensorStatus::OK;
        pre_vote_count++;
      } else if (health_state_[i] == SensorHealthState::INHIBITED) {
        statuses[i] = SensorStatus::INHIBITED;
      } else if (health_state_[i] == SensorHealthState::RECOVERING) {
        statuses[i] = SensorStatus::RECOVERING;
      } else if (enabled_mask[i]) {
        statuses[i] = SensorStatus::HARD_FAIL;
      }
    }

    // 3. Gyro soft/hard distance checks against robust median.
    if (pre_vote_count > 0) {
      // Relative median-distance hard-faulting is only observable with >=3
      // valid sensors. With 2 sensors, equal disagreement can hard-fault both.
      const bool relative_hard_fault_observable = pre_vote_count > 2;
      eskf_scalar gyro_median[3] = {};
      computeMedian3D(gyro_median, gyro_body, pre_vote_valid, cfg_.imu_count);
      for (size_t i = 0; i < cfg_.imu_count; ++i) {
        if (!pre_vote_valid[i]) {
          continue;
        }
        eskf_scalar d2 = 0;
        for (int j = 0; j < 3; ++j) {
          const eskf_scalar d = gyro_body[i][j] - gyro_median[j];
          d2 += d * d;
        }
        gyro_dist[i] = std::sqrt(d2);
        if (cfg_.voting_enabled && pre_vote_count > 2 &&
            soft_gyro_threshold > 0 && gyro_dist[i] > soft_gyro_threshold) {
          soft_reject_mask[i] = true;
        }
        if (relative_hard_fault_observable &&
            cfg_.gyro_hard_fault_threshold > 0 &&
            gyro_dist[i] > cfg_.gyro_hard_fault_threshold) {
          hard_fault_mask[i] = true;
        }
      }
    }

    // 4. Gyro fusion before rough derivative and accel voting projection.
    eskf_scalar fused_gyro[3];
    fused_gyro[0] = fused_gyro[1] = fused_gyro[2] = 0;
    size_t fused_n = 0;
    for (size_t i = 0; i < cfg_.imu_count; ++i) {
      fused_mask[i] = pre_vote_valid[i] && !soft_reject_mask[i] &&
                      !hard_fault_mask[i];
      if (fused_mask[i]) {
        for (int j = 0; j < 3; ++j) {
          fused_gyro[j] += gyro_body[i][j];
        }
        fused_n++;
      }
    }
    if (fused_n > 0) {
      for (int j = 0; j < 3; ++j) {
        fused_gyro[j] /= fused_n;
      }
    }

    // Bias-corrected gyro for lever-arm projection (optional)
    eskf_scalar fused_gyro_unbiased[3];
    for (int j = 0; j < 3; ++j) {
      fused_gyro_unbiased[j] =
          fused_gyro[j] - (gyro_bias_body ? gyro_bias_body[j] : 0);
    }

    // 3. Compute rough ω̇ (for accel voting only)
    eskf_scalar omega_dot_rough[3];
    computeOmegaDotRough(omega_dot_rough, fused_gyro_unbiased, dt);

    // 5. Accel soft/hard distance checks (projected to fixed PCB center).
    bool accel_vote_mask[ESKF_MAX_IMUS] = {};
    size_t accel_vote_count = 0;
    eskf_scalar accel_test[ESKF_MAX_IMUS][3] = {};
    for (size_t i = 0; i < cfg_.imu_count; ++i) {
      accel_vote_mask[i] = pre_vote_valid[i] && !hard_fault_mask[i];
      if (!accel_vote_mask[i]) {
        continue;
      }
      accel_vote_count++;

      eskf_scalar r[3];
      for (int j = 0; j < 3; ++j) {
        r[j] = pcb_center_[j] - cfg_.imus[i].position[j];
      }

      eskf_scalar omega_cross_r[3];
      omega_cross_r[0] = fused_gyro_unbiased[1] * r[2] - fused_gyro_unbiased[2] * r[1];
      omega_cross_r[1] = fused_gyro_unbiased[2] * r[0] - fused_gyro_unbiased[0] * r[2];
      omega_cross_r[2] = fused_gyro_unbiased[0] * r[1] - fused_gyro_unbiased[1] * r[0];

      eskf_scalar centripetal[3];
      centripetal[0] = fused_gyro_unbiased[1] * omega_cross_r[2] -
                       fused_gyro_unbiased[2] * omega_cross_r[1];
      centripetal[1] = fused_gyro_unbiased[2] * omega_cross_r[0] -
                       fused_gyro_unbiased[0] * omega_cross_r[2];
      centripetal[2] = fused_gyro_unbiased[0] * omega_cross_r[1] -
                       fused_gyro_unbiased[1] * omega_cross_r[0];

      eskf_scalar tangential[3];
      tangential[0] = omega_dot_rough[1] * r[2] - omega_dot_rough[2] * r[1];
      tangential[1] = omega_dot_rough[2] * r[0] - omega_dot_rough[0] * r[2];
      tangential[2] = omega_dot_rough[0] * r[1] - omega_dot_rough[1] * r[0];

      for (int j = 0; j < 3; ++j) {
        accel_test[i][j] = accel_body[i][j] - tangential[j] - centripetal[j];
      }
    }

    if (accel_vote_count > 0) {
      eskf_scalar accel_median[3] = {};
      computeMedian3D(accel_median, accel_test, accel_vote_mask, cfg_.imu_count);
      for (size_t i = 0; i < cfg_.imu_count; ++i) {
        if (!accel_vote_mask[i]) {
          continue;
        }
        eskf_scalar d2 = 0;
        for (int j = 0; j < 3; ++j) {
          const eskf_scalar d = accel_test[i][j] - accel_median[j];
          d2 += d * d;
        }
        accel_dist[i] = std::sqrt(d2);
        if (cfg_.voting_enabled && accel_vote_count > 1 &&
            soft_accel_threshold > 0 &&
            accel_dist[i] > soft_accel_threshold) {
          soft_reject_mask[i] = true;
        }
        if (accel_vote_count > 2 && cfg_.accel_hard_fault_threshold > 0 &&
            accel_dist[i] > cfg_.accel_hard_fault_threshold) {
          hard_fault_mask[i] = true;
        }
      }
    }

    // 6. Phase-aware clipping/saturation policy.
    bool frame_saturation = false;
    for (size_t i = 0; i < cfg_.imu_count; ++i) {
      if (!(enabled_mask[i] && has_data_mask[i])) {
        continue;
      }
      uint8_t axis_count = 0;
      for (int axis = 0; axis < 3; ++axis) {
        if (cfg_.accel_saturation_threshold > 0 &&
            std::abs(accel_body[i][axis]) >= cfg_.accel_saturation_threshold) {
          axis_count++;
        }
        if (cfg_.gyro_saturation_threshold > 0 &&
            std::abs(gyro_body[i][axis]) >= cfg_.gyro_saturation_threshold) {
          axis_count++;
        }
      }
      saturation_mask[i] = axis_count > 0;
      frame_saturation = frame_saturation || saturation_mask[i];

      if (saturation_mask[i]) {
        const bool allow_boost_degraded = runtime_policy_.boost_phase &&
                                          axis_count < cfg_.saturation_multi_axis_limit;
        if (!allow_boost_degraded) {
          if (saturation_counter_[i] < std::numeric_limits<uint16_t>::max()) {
            saturation_counter_[i]++;
          }
          if (saturation_counter_[i] >=
              cfg_.saturation_hard_fault_persistence_samples) {
            hard_fault_mask[i] = true;
          }
        }
      } else if (saturation_counter_[i] > 0) {
        saturation_counter_[i]--;
      }
    }

    // 7. Update persistent health lifecycle from sample observations.
    for (size_t i = 0; i < cfg_.imu_count; ++i) {
      // Only treat upstream input status as a hard sample when data exists for
      // this frame. Missing-source frames are expected in asynchronous replay
      // grouping and must not poison persistent health counters.
      const bool input_hard_sample = has_data_mask[i] && input_hard_mask[i];
      const bool hard_sample =
          input_hard_sample || hard_fault_mask[i] || stale_fault_mask[i];
      const bool soft_sample = soft_reject_mask[i] || saturation_mask[i];
      SensorHealthState state = health_state_[i];

      switch (state) {
      case SensorHealthState::INHIBITED:
        if (!has_data_mask[i]) {
          // No sample for this source in this frame: keep inhibited state and
          // recovery counters unchanged.
          break;
        }
        if (!hard_sample && !soft_sample) {
          if (healthy_counter_[i] < std::numeric_limits<uint16_t>::max()) {
            healthy_counter_[i]++;
          }
          if (healthy_counter_[i] >= cfg_.recovery_cooldown_samples) {
            state = SensorHealthState::RECOVERING;
            healthy_counter_[i] = 0;
          }
        } else {
          healthy_counter_[i] = 0;
        }
        break;
      case SensorHealthState::RECOVERING:
        if (!has_data_mask[i]) {
          // Missing-source frame: do not force regress to inhibited.
          break;
        }
        if (hard_sample) {
          state = SensorHealthState::INHIBITED;
          healthy_counter_[i] = 0;
        } else if (!soft_sample) {
          if (healthy_counter_[i] < std::numeric_limits<uint16_t>::max()) {
            healthy_counter_[i]++;
          }
          if (healthy_counter_[i] >= cfg_.recovery_confirm_samples) {
            state = SensorHealthState::ACTIVE;
            healthy_counter_[i] = 0;
            hard_fault_counter_[i] = 0;
            saturation_counter_[i] = 0;
          }
        } else {
          healthy_counter_[i] = 0;
        }
        break;
      case SensorHealthState::ACTIVE:
      case SensorHealthState::SUSPECT:
      default:
        if (!has_data_mask[i]) {
          // No new observation: hold state/counters.
          break;
        }
        if (hard_sample) {
          if (hard_fault_counter_[i] < std::numeric_limits<uint16_t>::max()) {
            hard_fault_counter_[i]++;
          }
          healthy_counter_[i] = 0;
        } else {
          if (hard_fault_counter_[i] > 0) {
            hard_fault_counter_[i]--;
          }
          if (state == SensorHealthState::SUSPECT && !soft_sample) {
            if (healthy_counter_[i] < std::numeric_limits<uint16_t>::max()) {
              healthy_counter_[i]++;
            }
            if (healthy_counter_[i] >= cfg_.recovery_confirm_samples) {
              state = SensorHealthState::ACTIVE;
              healthy_counter_[i] = 0;
            }
          } else {
            healthy_counter_[i] = 0;
          }
        }

        if (hard_fault_counter_[i] >= cfg_.hard_fault_persistence_samples) {
          state = SensorHealthState::INHIBITED;
          healthy_counter_[i] = 0;
        } else if (hard_fault_counter_[i] >= cfg_.hard_fault_suspect_samples) {
          state = SensorHealthState::SUSPECT;
        }
        break;
      }

      health_state_[i] = state;
      if (state == SensorHealthState::INHIBITED) {
        statuses[i] = SensorStatus::INHIBITED;
      } else if (state == SensorHealthState::RECOVERING) {
        statuses[i] = SensorStatus::RECOVERING;
      } else if (hard_sample) {
        statuses[i] = SensorStatus::HARD_FAIL;
      } else if (soft_reject_mask[i]) {
        statuses[i] = SensorStatus::SOFT_FAIL;
      } else if (enabled_mask[i] && has_data_mask[i] && !input_hard_mask[i]) {
        statuses[i] = SensorStatus::OK;
      } else {
        statuses[i] = SensorStatus::HARD_FAIL;
      }
    }

    // 8. Deterministic all-soft-reject continuity salvage.
    bool continuity_salvage_used = false;
    bool degraded_output = frame_saturation;
    size_t valid_now = 0;
    for (size_t i = 0; i < cfg_.imu_count; ++i) {
      if (statuses[i] == SensorStatus::OK) {
        valid_now++;
      }
    }
    if (valid_now == 0 && cfg_.enable_all_soft_reject_salvage) {
      size_t best_idx = ESKF_MAX_IMUS;
      eskf_scalar best_score = std::numeric_limits<eskf_scalar>::infinity();
      for (size_t i = 0; i < cfg_.imu_count; ++i) {
        const bool health_ok =
            (health_state_[i] == SensorHealthState::ACTIVE ||
             health_state_[i] == SensorHealthState::SUSPECT);
        if (!(health_ok && enabled_mask[i] && has_data_mask[i] &&
              !input_hard_mask[i] && !hard_fault_mask[i])) {
          continue;
        }
        if (!soft_reject_mask[i]) {
          continue;
        }
        eskf_scalar score = 0;
        if (soft_gyro_threshold > 1e-6) {
          score += gyro_dist[i] / soft_gyro_threshold;
        }
        if (soft_accel_threshold > 1e-6) {
          score += accel_dist[i] / soft_accel_threshold;
        }
        if (!std::isfinite(score)) {
          score = static_cast<eskf_scalar>(1e9);
        }
        if (score < best_score ||
            (std::abs(score - best_score) < 1e-9 && i < best_idx)) {
          best_score = score;
          best_idx = i;
        }
      }
      if (best_idx < cfg_.imu_count) {
        statuses[best_idx] = SensorStatus::OK;
        continuity_salvage_used = true;
        degraded_output = true;
      }
    }

    // 9. Average valid sensors (clean path using raw body data)
    eskf_scalar avg_accel[3], avg_gyro[3];
    averageValidSensors(avg_accel, avg_gyro, accel_body, gyro_body, statuses,
                        cfg_.imu_count);

    // 10. Update history for smooth derivative and status flags.
    if (history_count_ < kHistorySize) {
      for (int i = 0; i < 3; ++i) {
        omega_history_[history_count_][i] = avg_gyro[i];
        accel_history_[history_count_][i] = avg_accel[i];
      }
      for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
        for (int j = 0; j < 3; ++j) {
          imu_accel_body_history_[history_count_][i][j] = accel_body[i][j];
          imu_gyro_body_history_[history_count_][i][j] = gyro_body[i][j];
        }
      }
      timestamp_history_[history_count_] = t0_us + n * sample_dt_us;
      for (size_t i = 0; i < cfg_.imu_count; ++i) {
        status_history_[history_count_][i] = statuses[i];
        health_history_[history_count_][i] = health_state_[i];
        hard_fault_counter_history_[history_count_][i] = hard_fault_counter_[i];
      }
      for (size_t i = cfg_.imu_count; i < ESKF_MAX_IMUS; ++i) {
        status_history_[history_count_][i] = SensorStatus::HARD_FAIL;
        health_history_[history_count_][i] = SensorHealthState::INHIBITED;
        hard_fault_counter_history_[history_count_][i] = 0;
      }
      degraded_history_[history_count_] = degraded_output;
      salvage_history_[history_count_] = continuity_salvage_used;
      saturation_history_[history_count_] = frame_saturation;
      history_count_++;
    } else {
      // Shift history
      for (size_t h = 0; h < kHistorySize - 1; ++h) {
        for (int i = 0; i < 3; ++i) {
          omega_history_[h][i] = omega_history_[h + 1][i];
          accel_history_[h][i] = accel_history_[h + 1][i];
        }
        for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
          for (int j = 0; j < 3; ++j) {
            imu_accel_body_history_[h][i][j] =
                imu_accel_body_history_[h + 1][i][j];
            imu_gyro_body_history_[h][i][j] =
                imu_gyro_body_history_[h + 1][i][j];
          }
        }
        timestamp_history_[h] = timestamp_history_[h + 1];
        for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
          status_history_[h][i] = status_history_[h + 1][i];
          health_history_[h][i] = health_history_[h + 1][i];
          hard_fault_counter_history_[h][i] =
              hard_fault_counter_history_[h + 1][i];
        }
        degraded_history_[h] = degraded_history_[h + 1];
        salvage_history_[h] = salvage_history_[h + 1];
        saturation_history_[h] = saturation_history_[h + 1];
      }
      for (int i = 0; i < 3; ++i) {
        omega_history_[kHistorySize - 1][i] = avg_gyro[i];
        accel_history_[kHistorySize - 1][i] = avg_accel[i];
      }
      for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
        for (int j = 0; j < 3; ++j) {
          imu_accel_body_history_[kHistorySize - 1][i][j] = accel_body[i][j];
          imu_gyro_body_history_[kHistorySize - 1][i][j] = gyro_body[i][j];
        }
      }
      timestamp_history_[kHistorySize - 1] = t0_us + n * sample_dt_us;
      for (size_t i = 0; i < cfg_.imu_count; ++i) {
        status_history_[kHistorySize - 1][i] = statuses[i];
        health_history_[kHistorySize - 1][i] = health_state_[i];
        hard_fault_counter_history_[kHistorySize - 1][i] =
            hard_fault_counter_[i];
      }
      for (size_t i = cfg_.imu_count; i < ESKF_MAX_IMUS; ++i) {
        status_history_[kHistorySize - 1][i] = SensorStatus::HARD_FAIL;
        health_history_[kHistorySize - 1][i] = SensorHealthState::INHIBITED;
        hard_fault_counter_history_[kHistorySize - 1][i] = 0;
      }
      degraded_history_[kHistorySize - 1] = degraded_output;
      salvage_history_[kHistorySize - 1] = continuity_salvage_used;
      saturation_history_[kHistorySize - 1] = frame_saturation;
    }

    if (cfg_.use_central_diff && history_count_ < kHistorySize) {
      continue;
    }

    // 11. Compute smooth ω̇ (for navigation) at center sample (lag = 3)
    eskf_scalar omega_dot_smooth[3];
    computeOmegaDotSmooth(omega_dot_smooth, dt);
    eskf_scalar omega_dot_unclamped[3];
    bool omega_dot_clipped = false;
    for (int i = 0; i < 3; ++i) {
      omega_dot_unclamped[i] = omega_dot_smooth[i];
      if (!std::isfinite(omega_dot_smooth[i])) {
        omega_dot_unclamped[i] = 0;
        omega_dot_smooth[i] = 0;
        omega_dot_clipped = true;
      }
    }
    const eskf_scalar omega_dot_unclamped_norm = vectorNorm3(omega_dot_unclamped);
    const eskf_scalar max_omega_dot_norm = cfg_.omega_dot_max_norm;
    if (std::isfinite(max_omega_dot_norm) && max_omega_dot_norm > 0) {
      const eskf_scalar omega_dot_norm = vectorNorm3(omega_dot_smooth);
      if (std::isfinite(omega_dot_norm) && omega_dot_norm > max_omega_dot_norm &&
          omega_dot_norm > static_cast<eskf_scalar>(1e-9)) {
        const eskf_scalar scale = max_omega_dot_norm / omega_dot_norm;
        for (int i = 0; i < 3; ++i) {
          omega_dot_smooth[i] *= scale;
        }
        omega_dot_clipped = true;
      }
    }
    const eskf_scalar omega_dot_norm = vectorNorm3(omega_dot_smooth);

    eskf_scalar nav_accel[3];
    eskf_scalar nav_gyro[3];
    const SensorStatus *center_status = nullptr;
    const SensorHealthState *center_health = nullptr;
    const uint16_t *center_hard_counter = nullptr;
    bool center_degraded = false;
    bool center_salvage = false;
    bool center_saturation = false;
    if (cfg_.use_central_diff) {
      const size_t center_idx = kHistorySize / 2;
      for (int i = 0; i < 3; ++i) {
        nav_accel[i] = accel_history_[center_idx][i];
        nav_gyro[i] = omega_history_[center_idx][i];
      }
      center_status = status_history_[center_idx];
      center_health = health_history_[center_idx];
      center_hard_counter = hard_fault_counter_history_[center_idx];
      center_degraded = degraded_history_[center_idx];
      center_salvage = salvage_history_[center_idx];
      center_saturation = saturation_history_[center_idx];
    } else {
      const size_t last_idx = history_count_ - 1;
      for (int i = 0; i < 3; ++i) {
        nav_accel[i] = accel_history_[last_idx][i];
        nav_gyro[i] = omega_history_[last_idx][i];
      }
      center_status = status_history_[last_idx];
      center_health = health_history_[last_idx];
      center_hard_counter = hard_fault_counter_history_[last_idx];
      center_degraded = degraded_history_[last_idx];
      center_salvage = salvage_history_[last_idx];
      center_saturation = saturation_history_[last_idx];
    }

    // 12. Compute effective centroid of valid sensors.
    eskf_scalar centroid[3];
    computeEffectiveCentroid(centroid, center_status, cfg_.imu_count);

    // 13. Get current CG and compute dynamic lever arm.
    eskf_scalar cg[3];
    uint64_t cg_ts_us = cfg_.use_central_diff
                ? timestamp_history_[kHistorySize / 2]
                : timestamp_history_[history_count_ - 1];
    getCurrentCG(cg_ts_us, cg);

    eskf_scalar lever_arm[3];
    for (int i = 0; i < 3; ++i) {
      lever_arm[i] = centroid[i] - cg[i];
    }

    // 14. Final lever-arm correction (single projection using smooth
    // derivative)
    eskf_scalar accel_cg[3];
    eskf_scalar tangential_correction[3] = {};
    eskf_scalar centripetal_correction[3] = {};
    eskf_scalar lever_arm_correction_unclamped[3] = {};
    eskf_scalar lever_arm_correction[3] = {};
    bool lever_arm_correction_clipped = false;
    eskf_scalar nav_gyro_unbiased[3];
    for (int i = 0; i < 3; ++i) {
      nav_gyro_unbiased[i] =
          nav_gyro[i] - (gyro_bias_body ? gyro_bias_body[i] : 0);
    }
    applyLeverArmCorrection(accel_cg, nav_accel, nav_gyro_unbiased,
                omega_dot_smooth, lever_arm,
                tangential_correction, centripetal_correction,
                lever_arm_correction_unclamped,
                lever_arm_correction,
                &lever_arm_correction_clipped);
    const eskf_scalar lever_arm_correction_norm =
      vectorNorm3(lever_arm_correction);
    const eskf_scalar lever_arm_correction_unclamped_norm =
      vectorNorm3(lever_arm_correction_unclamped);

    // 15. Write output.
    for (int i = 0; i < 3; ++i) {
      out[out_count].frame.accel[i] = accel_cg[i];
      out[out_count].frame.gyro[i] = nav_gyro[i];
    }
    if (cfg_.use_central_diff) {
      const size_t center_idx = kHistorySize / 2;
      out[out_count].frame.timestamp_us = timestamp_history_[center_idx];
      for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
        for (int j = 0; j < 3; ++j) {
          out[out_count].imu_accel_body[i][j] =
              imu_accel_body_history_[center_idx][i][j];
          out[out_count].imu_gyro_body[i][j] =
              imu_gyro_body_history_[center_idx][i][j];
        }
      }
    } else {
      out[out_count].frame.timestamp_us =
          timestamp_history_[history_count_ - 1];
      const size_t last_idx = history_count_ - 1;
      for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
        for (int j = 0; j < 3; ++j) {
          out[out_count].imu_accel_body[i][j] =
              imu_accel_body_history_[last_idx][i][j];
          out[out_count].imu_gyro_body[i][j] =
              imu_gyro_body_history_[last_idx][i][j];
        }
      }
    }

    // Copy statuses and compute valid count
    size_t valid_count = 0;
    for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
      out[out_count].imu_status[i] = center_status[i];
      out[out_count].imu_health[i] = center_health[i];
      out[out_count].imu_hard_fault_counter[i] = center_hard_counter[i];
      if (i < cfg_.imu_count && center_status[i] == SensorStatus::OK)
        valid_count++;
    }
    out[out_count].valid_imu_count = valid_count;
    out[out_count].degraded_output = center_degraded;
    out[out_count].continuity_salvage_used = center_salvage;
    out[out_count].saturation_detected = center_saturation;
    for (int i = 0; i < 3; ++i) {
      out[out_count].effective_centroid[i] = centroid[i];
      out[out_count].nav_accel[i] = nav_accel[i];
      out[out_count].nav_gyro[i] = nav_gyro[i];
      out[out_count].omega_dot[i] = omega_dot_smooth[i];
      out[out_count].omega_dot_unclamped[i] = omega_dot_unclamped[i];
      out[out_count].tangential_correction[i] = tangential_correction[i];
      out[out_count].centripetal_correction[i] = centripetal_correction[i];
      out[out_count].lever_arm_correction[i] = lever_arm_correction[i];
      out[out_count].lever_arm_correction_unclamped[i] =
          lever_arm_correction_unclamped[i];
      out[out_count].cg[i] = cg[i];
      out[out_count].lever_arm[i] = lever_arm[i];
    }
    out[out_count].omega_dot_norm = omega_dot_norm;
    out[out_count].omega_dot_unclamped_norm = omega_dot_unclamped_norm;
    out[out_count].lever_arm_correction_norm = lever_arm_correction_norm;
    out[out_count].lever_arm_correction_unclamped_norm =
        lever_arm_correction_unclamped_norm;
    out[out_count].omega_dot_clipped = omega_dot_clipped;
    out[out_count].lever_arm_correction_clipped =
        lever_arm_correction_clipped;

    out_count++;
  }

  return out_count;
}

} // namespace eskf
