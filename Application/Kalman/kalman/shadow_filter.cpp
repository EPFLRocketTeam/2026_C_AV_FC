// Shadow Filter Implementation
// Part of Phase 1: Standalone Core ESKF Library
//
// Reference: Kalman_filter_planification.md Section 5.1

#include "shadow_filter.hpp"
#include "eskf_logger.hpp"
#include <cmath>

namespace eskf {

// ============================================================
// Rail Shadow Filter Implementation
// ============================================================

void RailShadowFilter::init(const TuningConfig &cfg) {
  cfg_ = cfg;
  g_local_ = static_cast<eskf_scalar>(cfg.local_gravity);
  reset();
}

void RailShadowFilter::reset() {
  // Identity quaternion
  q_[0] = 1;
  q_[1] = 0;
  q_[2] = 0;
  q_[3] = 0;

  // Zero integral
  integral_[0] = 0;
  integral_[1] = 0;
  integral_[2] = 0;

  gate_open_ = true;

  // Reset heading follower state
  heading_estimate_ = 0;
  heading_variance_ = 1000.0; // Large variance = unknown
  heading_initialized_ = false;

  // Reset gyro bias LPF state
  gyro_bias_lpf_[0] = 0;
  gyro_bias_lpf_[1] = 0;
  gyro_bias_lpf_[2] = 0;

  // Reset checkpoint buffer
  checkpoint_head_ = 0;
  checkpoint_count_ = 0;
  last_checkpoint_us_ = 0;

  // Reset ground reference accumulator
  current_window_ = GroundRefWindow{};
  current_window_active_ = false;
  window_head_ = 0;
  window_count_ = 0;
  ground_ref_ = GroundReference{};
  for (size_t i = 0; i < kGroundRefWindowCount; ++i) {
    window_buffer_[i] = GroundRefWindow{};
  }
}

void RailShadowFilter::update(const eskf_scalar accel_body[3],
                              const eskf_scalar gyro_body[3], eskf_scalar dt,
                              uint64_t timestamp_us) {
  // --- 0. Gyro Bias LPF Update ---
  // Update BEFORE any bias correction. Since rocket is stationary pre-flight,
  // raw gyro reading = true_omega(0) + bias ≈ bias
  const eskf_scalar alpha = cfg_.gyro_bias_lpf_alpha;
  gyro_bias_lpf_[0] = alpha * gyro_bias_lpf_[0] + (1 - alpha) * gyro_body[0];
  gyro_bias_lpf_[1] = alpha * gyro_bias_lpf_[1] + (1 - alpha) * gyro_body[1];
  gyro_bias_lpf_[2] = alpha * gyro_bias_lpf_[2] + (1 - alpha) * gyro_body[2];

  // --- 1. Gating Logic ---
  // Only use accelerometer feedback if magnitude is close to gravity
  const eskf_scalar accel_norm =
      std::sqrt(accel_body[0] * accel_body[0] + accel_body[1] * accel_body[1] +
                accel_body[2] * accel_body[2]);

  gate_open_ = std::fabs(accel_norm - g_local_) < cfg_.shadow_rail_gate;

  // --- 2. Compute Error (when gate open) ---
  eskf_scalar error[3] = {0, 0, 0};
  eskf_scalar expected_accel_body[3] = {0, 0, 0};

  if (gate_open_ && accel_norm > 1e-6) {
    // Normalize accelerometer reading
    // NOTE: Accelerometers measure SPECIFIC FORCE, not gravity.
    // Specific force = acceleration - gravity, so for a stationary object f =
    // -g. Therefore accel_body points OPPOSITE to gravity (i.e., "up" in NED
    // terms).
    const eskf_scalar inv_norm = 1.0 / accel_norm;
    const eskf_scalar accel_hat[3] = {accel_body[0] * inv_norm,
                                      accel_body[1] * inv_norm,
                                      accel_body[2] * inv_norm};

    // Expected specific force direction in body frame: opposite to gravity.
    // In NED, gravity is [0, 0, +1] (down), so specific force is [0, 0, -1]
    // (up). Rotate [0, 0, -1] from NED to body frame using q^(-1).
    eskf_scalar up_ned[3] = {0, 0, -1}; // NED Up = -Z = opposite of gravity
    math::quatRotateVectorInverse(expected_accel_body, q_, up_ned);

    // [FIXED] Error = Measured × Expected (not Expected × Measured!)
    //
    // The cross product gives the rotation axis needed to align expected with
    // measured. For convergence, we need error to point in the direction we
    // must rotate.
    //
    // Example: identity q (level) with rocket vertical (measured = [1,0,0]):
    //   expected = [0,0,-1], measured = [1,0,0]
    //   error = measured × expected = [1,0,0] × [0,0,-1] = [0,1,0]
    //   This positive Y rotation tilts nose UP toward vertical ✓
    //
    // With the wrong order (expected × measured = [0,-1,0]), the nose would
    // tilt DOWN, diverging from the correct orientation.
    error[0] = accel_hat[1] * expected_accel_body[2] -
               accel_hat[2] * expected_accel_body[1];
    error[1] = accel_hat[2] * expected_accel_body[0] -
               accel_hat[0] * expected_accel_body[2];
    error[2] = accel_hat[0] * expected_accel_body[1] -
               accel_hat[1] * expected_accel_body[0];
  }

  // --- 3. Mahony PI Controller ---
  // Update integral (only when gate is open to prevent windup)
  if (gate_open_) {
    integral_[0] += error[0] * cfg_.shadow_rail_ki * dt;
    integral_[1] += error[1] * cfg_.shadow_rail_ki * dt;
    integral_[2] += error[2] * cfg_.shadow_rail_ki * dt;

    // Hard clamp integral to prevent windup from tiny constant bias
    // Max value ~0.1 rad/s corresponds to ~5° of steady-state correction
    constexpr eskf_scalar kMaxIntegral = 0.1;
    for (int i = 0; i < 3; ++i) {
      if (integral_[i] > kMaxIntegral)
        integral_[i] = kMaxIntegral;
      if (integral_[i] < -kMaxIntegral)
        integral_[i] = -kMaxIntegral;
    }
  }

  // Correction = Kp * error + integral
  const eskf_scalar correction[3] = {
      error[0] * cfg_.shadow_rail_kp + integral_[0],
      error[1] * cfg_.shadow_rail_kp + integral_[1],
      error[2] * cfg_.shadow_rail_kp + integral_[2]};

  // Capture debug state
  debug_state_.expected_accel_body[0] = expected_accel_body[0];
  debug_state_.expected_accel_body[1] = expected_accel_body[1];
  debug_state_.expected_accel_body[2] = expected_accel_body[2];

  debug_state_.error[0] = error[0];
  debug_state_.error[1] = error[1];
  debug_state_.error[2] = error[2];

  debug_state_.integral[0] = integral_[0];
  debug_state_.integral[1] = integral_[1];
  debug_state_.integral[2] = integral_[2];

  debug_state_.correction[0] = correction[0];
  debug_state_.correction[1] = correction[1];
  debug_state_.correction[2] = correction[2];

  // --- 4. Apply Corrected Gyro to Quaternion ---
  // ω_corrected = ω + correction
  const eskf_scalar gyro_corrected[3] = {gyro_body[0] + correction[0],
                                         gyro_body[1] + correction[1],
                                         gyro_body[2] + correction[2]};

  // Convert angular rate to quaternion delta: dq ≈ [1, ω*dt/2]
  const eskf_scalar half_dt = 0.5 * dt;
  eskf_scalar dq[4] = {1.0, gyro_corrected[0] * half_dt,
                       gyro_corrected[1] * half_dt,
                       gyro_corrected[2] * half_dt};

  // Quaternion multiplication: q = q ⊗ dq
  eskf_scalar q_new[4];
  math::quatMultiply(q_new, q_, dq);

  // Normalize to prevent drift
  math::quatNormalize(q_new);

  // Update state
  q_[0] = q_new[0];
  q_[1] = q_new[1];
  q_[2] = q_new[2];
  q_[3] = q_new[3];
}

void RailShadowFilter::getQuaternion(eskf_scalar q_out[4]) const {
  q_out[0] = q_[0];
  q_out[1] = q_[1];
  q_out[2] = q_[2];
  q_out[3] = q_[3];
}

void RailShadowFilter::getCombinedQuaternion(eskf_scalar q_out[4]) const {
  // Start with gravity-based orientation (has correct roll/pitch, arbitrary
  // yaw) Extract current yaw from gravity quaternion
  eskf_scalar R_dcm[3][3];
  math::quatToDcm(R_dcm, q_);

  // R[0][0] and R[1][0] approach zero as pitch approaches ±90°.
  // Check magnitude of the projection onto the XY plane.
  // Threshold 0.05 allows pitch up to ~87 degrees (cos(87) ≈ 0.052)
  eskf_scalar xy_proj =
      std::sqrt(R_dcm[0][0] * R_dcm[0][0] + R_dcm[1][0] * R_dcm[1][0]);

  eskf_scalar current_yaw;
  if (xy_proj < 0.05) {
    // Singularity: Rocket is effectively vertical (>87 deg).
    // Cannot determine yaw from gravity vector alone.
    // Rely solely on the target heading application below.
    current_yaw = 0;
  } else {
    current_yaw = std::atan2(R_dcm[1][0], R_dcm[0][0]);
  }

  // If heading not initialized, use rail heading fallback
  eskf_scalar target_yaw =
      heading_initialized_ ? heading_estimate_ : cfg_.launch_rail_heading;
  eskf_scalar delta_yaw = target_yaw - current_yaw;

  // Normalize delta yaw to [-π, π]
  while (delta_yaw > constants::kPi)
    delta_yaw -= 2 * constants::kPi;
  while (delta_yaw < -constants::kPi)
    delta_yaw += 2 * constants::kPi;

  // Create rotation for delta_yaw around Z axis (NED Down)
  eskf_scalar half = 0.5 * delta_yaw;
  eskf_scalar dq[4] = {std::cos(half), 0, 0, std::sin(half)};

  // Apply: q_combined = dq * q_gravity (global Z rotation)
  math::quatMultiply(q_out, dq, q_);
  math::quatNormalize(q_out);
}

void RailShadowFilter::updateHeading(eskf_scalar heading_rad, eskf_scalar R) {
  if (!heading_initialized_) {
    // First heading: SNAP (no EKF update on uninitialized state)
    heading_estimate_ = heading_rad;
    heading_variance_ = R;
    heading_initialized_ = true;
    return;
  }

  // Simple scalar Kalman update
  eskf_scalar innovation = heading_rad - heading_estimate_;
  // Wrap to [-π, π]
  while (innovation > constants::kPi)
    innovation -= 2 * constants::kPi;
  while (innovation < -constants::kPi)
    innovation += 2 * constants::kPi;

  eskf_scalar S = heading_variance_ + R;
  eskf_scalar K = heading_variance_ / S;

  heading_estimate_ += K * innovation;
  heading_variance_ = (1 - K) * heading_variance_;

  // Normalize heading to [-π, π]
  while (heading_estimate_ > constants::kPi)
    heading_estimate_ -= 2 * constants::kPi;
  while (heading_estimate_ < -constants::kPi)
    heading_estimate_ += 2 * constants::kPi;
}

void RailShadowFilter::saveCheckpoint(uint64_t timestamp_us) {
  RailShadowCheckpoint &cp = checkpoint_buffer_[checkpoint_head_];

  // Copy current state
  for (int i = 0; i < 4; ++i)
    cp.q[i] = q_[i];
  for (int i = 0; i < 3; ++i)
    cp.gyro_bias[i] = gyro_bias_lpf_[i];
  cp.heading_estimate = heading_estimate_;
  cp.heading_variance = heading_variance_;
  cp.heading_initialized = heading_initialized_;
  cp.timestamp_us = timestamp_us;

  // Advance ring buffer
  checkpoint_head_ = (checkpoint_head_ + 1) % ESKF_RAIL_CHECKPOINT_BUFFER_SIZE;
  if (checkpoint_count_ < ESKF_RAIL_CHECKPOINT_BUFFER_SIZE) {
    checkpoint_count_++;
  }
  last_checkpoint_us_ = timestamp_us;
}

const RailShadowCheckpoint *
RailShadowFilter::findCheckpointBefore(uint64_t timestamp_us) const {
  if (checkpoint_count_ == 0)
    return nullptr;

  const RailShadowCheckpoint *best = nullptr;
  uint64_t best_ts = 0;

  // Search all checkpoints for most recent one at or before target timestamp
  for (size_t i = 0; i < checkpoint_count_; ++i) {
    size_t oldest = (checkpoint_count_ < ESKF_RAIL_CHECKPOINT_BUFFER_SIZE)
                        ? 0
                        : checkpoint_head_;
    size_t idx = (oldest + i) % ESKF_RAIL_CHECKPOINT_BUFFER_SIZE;
    const RailShadowCheckpoint &cp = checkpoint_buffer_[idx];

    if (cp.timestamp_us <= timestamp_us && cp.timestamp_us > best_ts) {
      best_ts = cp.timestamp_us;
      best = &cp;
    }
  }

  return best;
}

RailShadowCheckpoint
RailShadowFilter::currentAsCheckpoint(uint64_t timestamp_us) const {
  RailShadowCheckpoint cp;

  for (int i = 0; i < 4; ++i)
    cp.q[i] = q_[i];
  for (int i = 0; i < 3; ++i)
    cp.gyro_bias[i] = gyro_bias_lpf_[i];
  cp.heading_estimate = heading_estimate_;
  cp.heading_variance = heading_variance_;
  cp.heading_initialized = heading_initialized_;
  cp.timestamp_us = timestamp_us;

  // Include ground reference
  cp.ground_pressure_pa = ground_ref_.pressure_pa;
  cp.ground_temperature_k = ground_ref_.temperature_k;
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    cp.baro_offsets_pa[i] = ground_ref_.per_baro_offset_pa[i];
  }
  cp.ground_reference_valid = ground_ref_.valid;

  return cp;
}

void RailShadowFilter::getCombinedQuaternionFromCheckpoint(
    const RailShadowCheckpoint &cp, eskf_scalar q_out[4]) {
  // Reconstruct combined quaternion from checkpoint's gravity quaternion and
  // heading. This is identical to getCombinedQuaternion() but uses checkpoint
  // state instead of current.

  // Extract current yaw from checkpoint's gravity quaternion
  eskf_scalar R_dcm[3][3];
  math::quatToDcm(R_dcm, cp.q);

  // Handle Gimbal Lock (Vertical Rocket)
  // R[0][0] and R[1][0] approach zero as pitch approaches ±90°.
  eskf_scalar xy_proj =
      std::sqrt(R_dcm[0][0] * R_dcm[0][0] + R_dcm[1][0] * R_dcm[1][0]);

  eskf_scalar current_yaw;
  if (xy_proj < 0.05) {
    // Singularity: Rocket is effectively vertical (>87 deg).
    current_yaw = 0;
  } else {
    current_yaw = std::atan2(R_dcm[1][0], R_dcm[0][0]);
  }

  // Use checkpoint's heading if initialized, otherwise rail heading fallback
  eskf_scalar target_yaw =
      cp.heading_initialized
          ? cp.heading_estimate
          : getDefaultTuningConfig().launch_rail_heading; // Static fallback
  eskf_scalar delta_yaw = target_yaw - current_yaw;

  // Normalize delta yaw to [-π, π]
  while (delta_yaw > constants::kPi)
    delta_yaw -= 2 * constants::kPi;
  while (delta_yaw < -constants::kPi)
    delta_yaw += 2 * constants::kPi;

  // Create rotation for delta_yaw around Z axis (NED Down)
  eskf_scalar half = 0.5 * delta_yaw;
  eskf_scalar dq[4] = {std::cos(half), 0, 0, std::sin(half)};

  // Apply: q_combined = dq * q_gravity (global Z rotation)
  math::quatMultiply(q_out, dq, cp.q);
  math::quatNormalize(q_out);
}

void RailShadowFilter::initializeFromAccel(const eskf_scalar accel_body[3]) {
  // Initialize quaternion from accelerometer to align body frame with NED.
  // Body frame convention: X=forward (nose), Y=right, Z=down (FRD)
  // When rocket is vertical on rail, body +X points UP (toward nose).
  // Accelerometer measures specific force ≈ +1g on X-axis when
  // stationary/vertical.
  //
  // We compute a quaternion that rotates the measured specific force direction
  // to the NED "up" direction [0, 0, -1], giving the body-to-NED rotation.

  // 1. Normalize accelerometer reading
  eskf_scalar norm =
      std::sqrt(accel_body[0] * accel_body[0] + accel_body[1] * accel_body[1] +
                accel_body[2] * accel_body[2]);
  if (norm < 1e-3)
    return; // Invalid reading, keep current quaternion

  eskf_scalar ax = accel_body[0] / norm;
  eskf_scalar ay = accel_body[1] / norm;
  eskf_scalar az = accel_body[2] / norm;

  // 2. Compute shortest-arc rotation from accel to NED Up [0, 0, -1]
  //    Cross product gives rotation axis: axis = accel × up_ned
  eskf_scalar cross_x = ay * (-1.0) - az * 0.0; // -ay
  eskf_scalar cross_y = az * 0.0 - ax * (-1.0); // ax
  eskf_scalar cross_z = ax * 0.0 - ay * 0.0;    // 0

  //    Dot product gives cos(angle): dot = accel · up_ned = -az
  eskf_scalar dot = -az;

  // 3. Build quaternion using half-angle formula:
  //    q = [cos(θ/2), axis·sin(θ/2)]
  //    Using identity: s = 2·cos(θ/2) = sqrt(2·(1+cos(θ)))
  eskf_scalar s = std::sqrt((1.0 + dot) * 2.0);
  if (s < 1e-6) {
    // 180° case: accel points opposite to up (rocket upside down)
    // Use arbitrary perpendicular axis (X-axis)
    q_[0] = 0.0;
    q_[1] = 1.0;
    q_[2] = 0.0;
    q_[3] = 0.0;
  } else {
    eskf_scalar inv_s = 1.0 / s;
    q_[0] = 0.5 * s;
    q_[1] = cross_x * inv_s;
    q_[2] = cross_y * inv_s;
    q_[3] = cross_z * inv_s;
  }

  math::quatNormalize(q_);
}

// ============================================================
// Ground Reference Accumulator Implementation
// ============================================================

void RailShadowFilter::updateBaro(const eskf_scalar *pressures,
                                  const eskf_scalar *temps, const bool *valid,
                                  size_t baro_count, uint64_t timestamp_us) {
  // Initialize window start if needed
  if (!current_window_active_) {
    current_window_.window_start_us = timestamp_us;
    current_window_active_ = true;
  }

  // Check if current window is complete (1 second elapsed)
  if (timestamp_us - current_window_.window_start_us >= kGroundRefWindowUs) {
    bool was_valid = ground_ref_.valid;
    finalizeCurrentWindow();
    // Log event when ground reference first becomes valid
    if (!was_valid && ground_ref_.valid) {
      getEskfLogger().logEvent(EskfEventType::GroundRefCalibrated, timestamp_us,
                               static_cast<float>(ground_ref_.pressure_pa));
    }
    // Start new window
    current_window_ = GroundRefWindow{};
    current_window_.window_start_us = timestamp_us;
    current_window_active_ = true;
  }

  // Accumulate samples for each valid baro
  for (size_t i = 0; i < baro_count && i < ESKF_MAX_BAROS; ++i) {
    if (valid[i] && pressures[i] > 0) {
      current_window_.pressure_sum[i] += pressures[i];
      current_window_.sample_count[i]++;
    }
  }

  // Accumulate temperature (use first valid sensor's temp)
  for (size_t i = 0; i < baro_count && i < ESKF_MAX_BAROS; ++i) {
    if (valid[i] && temps[i] > 0) {
      current_window_.temperature_sum += temps[i];
      current_window_.temp_sample_count++;
      break; // Only use first valid temp
    }
  }
}

void RailShadowFilter::finalizeCurrentWindow() {
  // Check if window has enough samples
  bool has_data = false;
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    if (current_window_.sample_count[i] > 0) {
      has_data = true;
      break;
    }
  }

  if (!has_data || current_window_.temp_sample_count == 0) {
    return; // No valid data in this window
  }

  // Mark window as complete and store in ring buffer
  current_window_.complete = true;
  window_buffer_[window_head_] = current_window_;
  window_head_ = (window_head_ + 1) % kGroundRefWindowCount;
  if (window_count_ < kGroundRefWindowCount) {
    window_count_++;
  }

  // Recompute ground reference from oldest complete window
  computeGroundReference();
}

void RailShadowFilter::computeGroundReference() {
  if (window_count_ == 0)
    return;

  // Find oldest complete window
  size_t oldest_idx =
      (window_count_ < kGroundRefWindowCount) ? 0 : window_head_;
  const GroundRefWindow &oldest = window_buffer_[oldest_idx];

  if (!oldest.complete)
    return;

  // Compute per-sensor averages
  eskf_scalar per_sensor_avg[ESKF_MAX_BAROS] = {0};
  size_t valid_sensor_count = 0;
  eskf_scalar sum_all_sensors = 0;

  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    if (oldest.sample_count[i] > 0) {
      per_sensor_avg[i] = oldest.pressure_sum[i] / oldest.sample_count[i];
      sum_all_sensors += per_sensor_avg[i];
      valid_sensor_count++;
    }
  }

  if (valid_sensor_count == 0)
    return;

  // Virtual ground = average of all sensors
  eskf_scalar virtual_ground_pa = sum_all_sensors / valid_sensor_count;

  // Per-sensor offset = virtual_ground - sensor_avg
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    if (oldest.sample_count[i] > 0) {
      ground_ref_.per_baro_offset_pa[i] = virtual_ground_pa - per_sensor_avg[i];
    } else {
      ground_ref_.per_baro_offset_pa[i] = 0;
    }
  }

  // Store virtual ground P/T
  ground_ref_.pressure_pa = virtual_ground_pa;
  ground_ref_.temperature_k = oldest.temperature_sum / oldest.temp_sample_count;
  ground_ref_.valid = true;
}

const GroundReference &RailShadowFilter::getOldestGroundReference() const {
  return ground_ref_;
}

bool RailShadowFilter::estimateGroundReferenceFromActiveWindow(
    GroundReference &out) const {
  out = GroundReference{};

  if (!current_window_active_) {
    return false;
  }
  if (current_window_.temp_sample_count == 0) {
    return false;
  }

  eskf_scalar per_sensor_avg[ESKF_MAX_BAROS] = {0};
  size_t valid_sensor_count = 0;
  eskf_scalar sum_all_sensors = 0;

  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    if (current_window_.sample_count[i] > 0) {
      per_sensor_avg[i] =
          current_window_.pressure_sum[i] / current_window_.sample_count[i];
      sum_all_sensors += per_sensor_avg[i];
      valid_sensor_count++;
    }
  }

  if (valid_sensor_count == 0) {
    return false;
  }

  const eskf_scalar virtual_ground_pa = sum_all_sensors / valid_sensor_count;
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    if (current_window_.sample_count[i] > 0) {
      out.per_baro_offset_pa[i] = virtual_ground_pa - per_sensor_avg[i];
    }
  }

  out.pressure_pa = virtual_ground_pa;
  out.temperature_k =
      current_window_.temperature_sum / current_window_.temp_sample_count;
  out.valid = true;
  return true;
}

// ============================================================
// Flight Shadow Filter Implementation
// ============================================================

void FlightShadowFilter::init(const TuningConfig &cfg) {
  cfg_ = cfg;
  if (cfg_.shadow_aero_blind_enter_speed <= 0.0f) {
    cfg_.shadow_aero_blind_enter_speed = cfg_.shadow_aero_blind_speed;
  }
  if (cfg_.shadow_aero_blind_exit_speed <= 0.0f ||
      cfg_.shadow_aero_blind_exit_speed > cfg_.shadow_aero_blind_enter_speed) {
    cfg_.shadow_aero_blind_exit_speed =
        0.9f * cfg_.shadow_aero_blind_enter_speed;
  }
  g_local_ = static_cast<eskf_scalar>(cfg.local_gravity);
}

void FlightShadowFilter::reset(const eskf_scalar initial_q[4]) {
  if (initial_q) {
    // Copy initial quaternion from Rail Shadow
    q_[0] = initial_q[0];
    q_[1] = initial_q[1];
    q_[2] = initial_q[2];
    q_[3] = initial_q[3];
  } else {
    // Default to identity if no initial quaternion provided
    q_[0] = 1.0;
    q_[1] = 0.0;
    q_[2] = 0.0;
    q_[3] = 0.0;
  }

  // Zero vertical state
  z_ = 0;
  v_ = 0;

  // Reset aero-blind state
  is_aero_blind_ = false;
  was_aero_blind_ = false;
  aero_blind_enter_accum_s_ = 0;
  aero_blind_exit_accum_s_ = 0;
  last_reengage_snap_delta_m_ = 0;
  last_predict_timestamp_us_ = 0;
  saved_alt_snapshot_ = 0;
  snapshot_timestamp_us_ = 0;
  last_baro_timestamp_us_ = 0;
  has_alt_snapshot_ = false;
  last_known_dt_baro_ = 0.01; // Reset cached dt to default
}

void FlightShadowFilter::predict(const eskf_scalar accel_body[3],
                                 const eskf_scalar gyro_body[3], eskf_scalar dt,
                                 uint64_t timestamp_us) {
  // Guard against invalid dt
  if (dt <= 0)
    return;

  if (timestamp_us > 0) {
    last_predict_timestamp_us_ = timestamp_us;
  }

  // --- 1. Update Dead-Reckoning Attitude (Pure Gyro, No Corrections) ---
  // This maintains independence from the main ESKF
  // Using exact Rodrigues rotation formula for stability at high spin rates.
  // More stable than linear approximation when rocket spins > 360 deg/s,
  // without the complexity of coning/sculling history buffers.
  const eskf_scalar w_norm =
      std::sqrt(gyro_body[0] * gyro_body[0] + gyro_body[1] * gyro_body[1] +
                gyro_body[2] * gyro_body[2]);

  const eskf_scalar theta = w_norm * dt;

  eskf_scalar dq[4];
  if (w_norm > 1e-6) {
    // Exact integration (Rodrigues formula)
    const eskf_scalar half_theta = 0.5 * theta;
    const eskf_scalar k = std::sin(half_theta) / w_norm;
    dq[0] = std::cos(half_theta);
    dq[1] = gyro_body[0] * k;
    dq[2] = gyro_body[1] * k;
    dq[3] = gyro_body[2] * k;
  } else {
    // Linear fallback for small angles (avoids division by near-zero)
    const eskf_scalar half_dt = 0.5 * dt;
    dq[0] = 1.0;
    dq[1] = gyro_body[0] * half_dt;
    dq[2] = gyro_body[1] * half_dt;
    dq[3] = gyro_body[2] * half_dt;
  }

  eskf_scalar q_new[4];
  math::quatMultiply(q_new, q_, dq);
  math::quatNormalize(q_new);

  q_[0] = q_new[0];
  q_[1] = q_new[1];
  q_[2] = q_new[2];
  q_[3] = q_new[3];

  // --- 2. Rotate Acceleration to NED Frame ---
  eskf_scalar accel_ned[3];
  math::quatRotateVector(accel_ned, q_, accel_body);

  // --- 3. Extract Vertical Acceleration and Add Gravity ---
  // In NED: +Z is Down, gravity points +Z
  // az_nav = az_body_rotated + g (gravity adds to downward acceleration)
  const eskf_scalar az = accel_ned[2] + g_local_;

  // --- 4. Integrate Velocity and Position (Trapezoidal / Average Velocity) ---
  // Using trapezoidal integration for better accuracy during motor burn
  // when acceleration changes rapidly. This is the kinematic equation:
  // z_new = z_old + 0.5 * (v_old + v_new) * dt
  const eskf_scalar v_prev = v_;
  v_ += az * dt;
  z_ += 0.5 * (v_prev + v_) * dt;

  // --- 5. Update Aero-Blind State with hysteresis + debounce ---
  const eskf_scalar abs_v = std::fabs(v_);
  const eskf_scalar enter_debounce_s =
      static_cast<eskf_scalar>(cfg_.shadow_aero_blind_enter_debounce_ms) *
      1e-3f;
  const eskf_scalar exit_debounce_s =
      static_cast<eskf_scalar>(cfg_.shadow_aero_blind_exit_debounce_ms) *
      1e-3f;

  if (!is_aero_blind_) {
    aero_blind_exit_accum_s_ = 0;
    if (abs_v > cfg_.shadow_aero_blind_enter_speed) {
      aero_blind_enter_accum_s_ += dt;
      if (aero_blind_enter_accum_s_ >= enter_debounce_s) {
        is_aero_blind_ = true;
        was_aero_blind_ = true;
        aero_blind_enter_accum_s_ = 0;
        if (timestamp_us > 0) {
          getEskfLogger().logEvent(EskfEventType::AeroBlindEntered,
                                   timestamp_us,
                                   static_cast<float>(v_));
        }
      }
    } else {
      aero_blind_enter_accum_s_ = 0;
    }
  } else {
    aero_blind_enter_accum_s_ = 0;
    if (abs_v < cfg_.shadow_aero_blind_exit_speed) {
      aero_blind_exit_accum_s_ += dt;
      if (aero_blind_exit_accum_s_ >= exit_debounce_s) {
        is_aero_blind_ = false;
        aero_blind_exit_accum_s_ = 0;
        if (timestamp_us > 0) {
          getEskfLogger().logEvent(EskfEventType::AeroBlindExited,
                                   timestamp_us,
                                   static_cast<float>(v_));
        }
      }
    } else {
      aero_blind_exit_accum_s_ = 0;
    }
  }
}

void FlightShadowFilter::forceExitAeroBlindForImuOutage() {
  if (!is_aero_blind_) {
    return;
  }
  is_aero_blind_ = false;
  was_aero_blind_ = true;
  aero_blind_enter_accum_s_ = 0;
  aero_blind_exit_accum_s_ = 0;
}

/// Barometer correction step (observer update).
/// @param z_baro Measured barometric altitude (m, POSITIVE UP - will be
/// converted to NED internally)
/// @param dt_baro Time since last baro measurement (seconds)
void FlightShadowFilter::correctBaro(eskf_scalar z_baro, eskf_scalar dt_baro) {
  // Guard against invalid dt
  if (dt_baro <= 0)
    return;

  // Cache dt for fallback path
  last_known_dt_baro_ = dt_baro;

  // Barometer reports altitude (positive up), convert to NED (positive down)
  const eskf_scalar z_baro_ned = -z_baro;

  // --- Aero-Blind Mode Check ---
  if (is_aero_blind_) {
    // Pure inertial integration, ignore corrupted barometer
    // was_aero_blind_ already set in predict() on entry
    return;
  }

  // --- Re-Engagement Snap ---
  // On transition from aero-blind to subsonic, snap position to baro
  // This prevents velocity spike from accumulated drift
  if (was_aero_blind_) {
    last_reengage_snap_delta_m_ = std::fabs(z_baro_ned - z_);
    z_ = z_baro_ned;
    was_aero_blind_ = false;
    // Skip normal correction this cycle to let velocity stabilize
    return;
  }

  // --- Normal Observer Correction ---
  // Observer gains designed for specific convergence rate (pole placement)
  // K_z = 2 * zeta * omega_n (position gain)
  // K_v = omega_n^2 (velocity gain)
  const eskf_scalar K_z =
      2.0 * cfg_.shadow_flight_zeta * cfg_.shadow_flight_omega_n;
  const eskf_scalar K_v =
      cfg_.shadow_flight_omega_n * cfg_.shadow_flight_omega_n;

  // Innovation (measurement residual)
  const eskf_scalar innovation = z_baro_ned - z_;

  // Apply observer correction
  // Note: correction scaled by dt_baro as per continuous-time observer design
  z_ += K_z * innovation * dt_baro;
  v_ += K_v * innovation * dt_baro;
}

void FlightShadowFilter::correctBaroWithSnapshot(
    eskf_scalar measured_alt, eskf_scalar predicted_alt_at_trigger,
    eskf_scalar dt_baro) {
  // Innovation Transport (Section 3.4.A):
  // Calculate innovation using snapshot from trigger time,
  // but apply correction to current state.

  // Convert both to NED (positive down)
  const eskf_scalar z_meas_ned = -measured_alt;
  const eskf_scalar z_snapshot_ned = -predicted_alt_at_trigger;

  // --- Aero-Blind Mode Check ---
  if (is_aero_blind_) {
    return; // Ignore corrupted barometer
  }

  // --- Re-Engagement Snap ---
  if (was_aero_blind_) {
    last_reengage_snap_delta_m_ = std::fabs(z_meas_ned - z_);
    z_ = z_meas_ned;
    was_aero_blind_ = false;
    return;
  }

  // --- Normal Observer Correction with Innovation Transport ---
  const eskf_scalar K_z =
      2.0 * cfg_.shadow_flight_zeta * cfg_.shadow_flight_omega_n;
  const eskf_scalar K_v =
      cfg_.shadow_flight_omega_n * cfg_.shadow_flight_omega_n;

  // Innovation is against SNAPSHOT, not current state
  const eskf_scalar innovation = z_meas_ned - z_snapshot_ned;

  // Apply observer correction to CURRENT state
  z_ += K_z * innovation * dt_baro;
  v_ += K_v * innovation * dt_baro;
}

void FlightShadowFilter::saveAltitudeSnapshot(uint64_t trigger_timestamp_us) {
  saved_alt_snapshot_ = -z_; // Convert from NED to positive-up
  snapshot_timestamp_us_ = trigger_timestamp_us;
  has_alt_snapshot_ = true;
}

void FlightShadowFilter::correctBaroFromSnapshot(
    eskf_scalar measured_alt, uint64_t measurement_ready_us) {
  if (!has_alt_snapshot_) {
    // No snapshot saved - can't use innovation transport, fall back to regular.
    // Use cached dt from last successful correction.
    correctBaro(measured_alt, last_known_dt_baro_);
    last_baro_timestamp_us_ = measurement_ready_us;
    return;
  }

  // Compute dt from timestamps
  eskf_scalar dt_baro;
  if (last_baro_timestamp_us_ > 0) {
    dt_baro = (measurement_ready_us - last_baro_timestamp_us_) * 1e-6;
  } else {
    dt_baro = 0.01; // 10ms default for first sample
  }
  last_baro_timestamp_us_ = measurement_ready_us;

  // Use saved snapshot for innovation transport
  correctBaroWithSnapshot(measured_alt, saved_alt_snapshot_, dt_baro);

  // Clear snapshot after use
  has_alt_snapshot_ = false;
}

} // namespace eskf
