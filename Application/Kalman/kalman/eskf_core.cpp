// ESKF Core Filter Implementation
// Part of Phase 1: Standalone Core ESKF Library

#include "eskf_core.hpp"
#include "eskf_logger.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace eskf {

// ============================================================

void EskfCore::init(const TuningConfig &cfg) {
  cfg_ = cfg;
  g_local_ = static_cast<eskf_scalar>(cfg.local_gravity);
  reset();
}

void EskfCore::reset() {
  state_.setIdentity();
  cov_.setIdentity(1.0);
  // Note: Q_ is NOT reset here - it's configuration, not state
  last_nis_ = 0;
  last_innovation_ = 0;
  consecutive_high_nis_count_ = 0;
  consecutive_low_nis_count_ = 0;
  predict_dt_clamped_count_ = 0;
  diverged_ = false;
  nis_soft_diverged_ = false;
  mode_ = FilterMode::Settling;

  // Reset integration state
  prev_gyro_.setZero();
  prev_accel_body_.setZero();
  prev_vel_.setZero();
  first_run_ = true;
  cov_decimation_counter_ = 0;
  imu_dynamics_log_counter_ = 0;
#if ESKF_COVARIANCE_DECIMATION > 1
  cov_decimation_dt_acc_s_ = 0;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_decimation_F_acc_[i][j] = (i == j) ? 1 : 0;
      cov_decimation_Q_acc_[i][j] = 0;
    }
  }
#endif
  heading_aligned_ = false;

  // Do NOT reset g_local_ here (it's configuration)

  // Log reset event
  getEskfLogger().logEvent(EskfEventType::FilterReset, 0);
}

void EskfCore::initialize(const State &initial_state,
                          const InitialCovariance &P0, const ProcessNoise &Q) {
  state_ = initial_state;

  // Build diagonal P0 from configuration
  eskf_scalar diag[kDimError];
  P0.toDiagonal(diag);
  cov_.setDiagonal(diag);

  Q_ = Q;
  last_nis_ = 0;
  last_innovation_ = 0;
  consecutive_high_nis_count_ = 0;
  consecutive_low_nis_count_ = 0;
  predict_dt_clamped_count_ = 0;
  diverged_ = false;
  nis_soft_diverged_ = false;
  mode_ = FilterMode::Settling;

  // Reset integration state
  resetIntegrationState();
  cov_decimation_counter_ = 0;
  imu_dynamics_log_counter_ = 0;
#if ESKF_COVARIANCE_DECIMATION > 1
  cov_decimation_dt_acc_s_ = 0;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_decimation_F_acc_[i][j] = (i == j) ? 1 : 0;
      cov_decimation_Q_acc_[i][j] = 0;
    }
  }
#endif
  heading_aligned_ = false;

  // Normalize quaternion just in case
  math::quatNormalize(state_.q);

  // Log initialization event
  getEskfLogger().logEvent(EskfEventType::FilterInitialized,
                           initial_state.timestamp_us);
}

bool EskfCore::freezeAccelBiasInFlight() const {
  return mode_ == FilterMode::Flight && cfg_.freeze_accel_bias_in_flight;
}

bool EskfCore::freezeGyroBiasInFlight() const {
  return mode_ == FilterMode::Flight && cfg_.freeze_gyro_bias_in_flight;
}

void EskfCore::freezeFlightBiasCovariance() {
  const bool freeze_accel_bias = freezeAccelBiasInFlight();
  const bool freeze_gyro_bias = freezeGyroBiasInFlight();
  if (!freeze_accel_bias && !freeze_gyro_bias) {
    return;
  }

  flushDeferredCovariancePropagation();

  if (freeze_accel_bias) {
    for (int axis = 0; axis < 3; ++axis) {
      const int b_idx = idx::kAccBias + axis;
      for (int i = 0; i < kDimError; ++i) {
        cov_.P[b_idx][i] = 0;
        cov_.P[i][b_idx] = 0;
      }
    }
  }

  if (freeze_gyro_bias) {
    for (int axis = 0; axis < 3; ++axis) {
      const int b_idx = idx::kGyrBias + axis;
      for (int i = 0; i < kDimError; ++i) {
        cov_.P[b_idx][i] = 0;
        cov_.P[i][b_idx] = 0;
      }
    }
  }
}

// ============================================================
// Prediction Step
// ============================================================

void EskfCore::predict(const ImuFrame &imu, eskf_scalar dt) {
  if (!std::isfinite(dt) || dt <= 0 || diverged_)
    return;

  eskf_scalar dt_used = dt;
  if (cfg_.predict_max_dt_s > 0 && dt_used > cfg_.predict_max_dt_s) {
    predict_dt_clamped_count_++;
    getEskfLogger().logEvent(EskfEventType::PredictDtClamped,
                             imu.timestamp_us,
                             static_cast<float>(dt_used));
    dt_used = cfg_.predict_max_dt_s;
  }

  // Get bias-corrected measurements
  eskf_scalar accel_body[3], gyro_body[3];
  for (int i = 0; i < 3; ++i) {
    accel_body[i] = imu.accel[i] - state_.b_acc[i];
    gyro_body[i] = imu.gyro[i] - state_.b_gyro[i];
  }

  // Handle first run for trapezoidal/coning
  if (first_run_) {
    for (int i = 0; i < 3; i++) {
      prev_gyro_(i) = gyro_body[i];
      prev_accel_body_(i) = accel_body[i];
      prev_vel_(i) = state_.v[i];
    }
    first_run_ = false; // Mark initialization complete
  }

  // ----------------------------------------------------------------
  // 1. Attitude Update (Coning Compensation)
  // ----------------------------------------------------------------
  eskf_scalar dtheta[3];

#if ESKF_USE_CONING_COMPENSATION
  // Coning & Sculling Compensation
  // alpha = gyro * dt (approx)
  // beta = accel * dt (approx)

  // Use user-provided logic:
  // alpha_new = gyro_current * dt
  // alpha_old = gyro_previous * dt
  // beta_new = accel_current * dt
  // beta_old = accel_previous * dt

  // Current values
  math::Vector3 gyro_curr;
  gyro_curr(0) = gyro_body[0];
  gyro_curr(1) = gyro_body[1];
  gyro_curr(2) = gyro_body[2];

  math::Vector3 acc_body_curr;
  acc_body_curr(0) = accel_body[0];
  acc_body_curr(1) = accel_body[1];
  acc_body_curr(2) = accel_body[2];

  // Deltas
  math::Vector3 alpha_new = gyro_curr * dt_used;
  math::Vector3 alpha_old = prev_gyro_ * dt_used;

  math::Vector3 beta_new = acc_body_curr * dt_used;
  math::Vector3 beta_old = prev_accel_body_ * dt_used;

  // --- Coning Correction ---
  // (1/12) * (alpha_old x alpha_new)
  math::Vector3 coning_corr =
      (1.0 / 12.0) * prev_gyro_.cross(gyro_curr) * (dt_used * dt_used);

  // Total Rotation Vector
  math::Vector3 rotation_vector = alpha_new + coning_corr;

  for (int i = 0; i < 3; ++i)
    dtheta[i] = rotation_vector(i);

  // --- Sculling Correction ---
  // (1/12) * (alpha_old x beta_new + beta_old x alpha_new)
  // Note: Cross product of vectors is consistent regardless of dt factor
  // scaling, but let's be precise with the formula: correction = (1/12) * (
  // (w_old*dt) x (a_new*dt) + (a_old*dt) x (w_new*dt) )
  //            = (1/12) * dt*dt * ( w_old x a_new + a_old x w_new )
  math::Vector3 sculling_corr =
      (1.0 / 12.0) * (alpha_old.cross(beta_new) + beta_old.cross(alpha_new));

  // Apply to Body Velocity Change (Beta)
  math::Vector3 beta_corrected = beta_new + sculling_corr;

#else
  // Standard rectangular integration
  for (int i = 0; i < 3; ++i) {
    dtheta[i] = gyro_body[i] * dt_used;
  }
  // No sculling, just use raw beta
  math::Vector3 beta_corrected;
  beta_corrected(0) = accel_body[0] * dt_used;
  beta_corrected(1) = accel_body[1] * dt_used;
  beta_corrected(2) = accel_body[2] * dt_used;
#endif

  // Update History
  prev_gyro_ = gyro_curr;
  prev_accel_body_ = acc_body_curr;

  // ----------------------------------------------------------------
  // 2. Velocity Update (Midpoint Rotation)
  // ----------------------------------------------------------------

  // We want to rotate the Delta-V (beta) by the "Average" attitude over the
  // interval. Using the OLD quaternion + a rotational correction is the
  // standard Strapdown approach. Formula: v_nav = q_old * (beta_corrected + 0.5
  // * (theta x beta_corrected)) This approximates multiplying by q_midpoint.

  math::Vector3 beta_rot_corr =
      beta_corrected + (0.5 * rotation_vector.cross(beta_corrected));

  // Rotate to NED using OLD quaternion (before update)
  eskf_scalar beta_final_arr[3] = {beta_rot_corr(0), beta_rot_corr(1),
                                   beta_rot_corr(2)};
  eskf_scalar vel_inc_ned[3];
  math::quatRotateVector(vel_inc_ned, state_.q,
                         beta_final_arr); // state_.q is still OLD here
  eskf_scalar accel_ned[3];
  math::quatRotateVector(accel_ned, state_.q, accel_body);

  // ----------------------------------------------------------------
  // 3. Attitude Update
  // ----------------------------------------------------------------

  eskf_scalar dq[4];
  math::quatFromRotationVector(dq, dtheta);

  // Update quaternion: q_new = q_old ⊗ dq
  eskf_scalar q_new[4];
  math::quatMultiply(q_new, state_.q, dq);
  for (int i = 0; i < 4; ++i)
    state_.q[i] = q_new[i];
  math::quatNormalize(state_.q);

  // ----------------------------------------------------------------
  // 4. State Integration (Mixed Order)
  // ----------------------------------------------------------------

  math::Vector3 vel_inc_ned_vec;
  vel_inc_ned_vec(0) = vel_inc_ned[0];
  vel_inc_ned_vec(1) = vel_inc_ned[1];
  vel_inc_ned_vec(2) = vel_inc_ned[2];

  // Gravity in NED
  eskf_scalar g_ned[3];
  gravityNed(g_ned);
  math::Vector3 g_vec;
  g_vec(0) = g_ned[0];
  g_vec(1) = g_ned[1];
  g_vec(2) = g_ned[2];

  // --- VELOCITY UPDATE ---
  // Use the Sculling-Corrected Delta-V directly.
  // Add Gravity (Trapezoidal on gravity is just gravity since it's constant).
  math::Vector3 vel_prev;
  vel_prev(0) = state_.v[0];
  vel_prev(1) = state_.v[1];
  vel_prev(2) = state_.v[2];

  // v_new = v_old + delta_v_body_rotated + (g * dt)
  math::Vector3 vel_new = vel_prev + vel_inc_ned_vec + (g_vec * dt_used);

  // --- POSITION UPDATE (Trapezoidal) ---
  // p_new = p_old + 0.5 * (v_old + v_new) * dt
  math::Vector3 pos_prev;
  pos_prev(0) = state_.p[0];
  pos_prev(1) = state_.p[1];
  pos_prev(2) = state_.p[2];

  math::Vector3 pos_new = pos_prev + 0.5 * (vel_prev + vel_new) * dt_used;

  // Update State
  for (int i = 0; i < 3; ++i) {
    state_.v[i] = vel_new(i);
    state_.p[i] = pos_new(i);
  }

  // ----------------------------------------------------------------
  // 5. Covariance Propagation (Sparse)
  // ----------------------------------------------------------------

#if ESKF_COVARIANCE_DECIMATION > 1
  eskf_scalar F_step[kDimError][kDimError];
  computeF(F_step, accel_body, dt_used);

  // Build per-step discrete process noise.
  const bool freeze_accel_bias = freezeAccelBiasInFlight();
  const bool freeze_gyro_bias = freezeGyroBiasInFlight();
  eskf_scalar Q_step[kDimError][kDimError] = {};
  const eskf_scalar q_vel = Q_.accel_noise * Q_.accel_noise * dt_used;
  const eskf_scalar q_att = Q_.gyro_noise * Q_.gyro_noise * dt_used;
  const eskf_scalar q_ba = freeze_accel_bias ? 0 : Q_.accel_bias_walk * dt_used;
  const eskf_scalar q_bg = freeze_gyro_bias ? 0 : Q_.gyro_bias_walk * dt_used;
  const eskf_scalar q_baro = Q_.baro_bias_walk * dt_used;

  Q_step[idx::kPos + 0][idx::kPos + 0] = q_vel * dt_used * dt_used / 3;
  Q_step[idx::kPos + 1][idx::kPos + 1] = q_vel * dt_used * dt_used / 3;
  Q_step[idx::kPos + 2][idx::kPos + 2] = q_vel * dt_used * dt_used / 3;
  Q_step[idx::kVel + 0][idx::kVel + 0] = q_vel;
  Q_step[idx::kVel + 1][idx::kVel + 1] = q_vel;
  Q_step[idx::kVel + 2][idx::kVel + 2] = q_vel;
  Q_step[idx::kAtt + 0][idx::kAtt + 0] = q_att;
  Q_step[idx::kAtt + 1][idx::kAtt + 1] = q_att;
  Q_step[idx::kAtt + 2][idx::kAtt + 2] = q_att;
  Q_step[idx::kAccBias + 0][idx::kAccBias + 0] = q_ba;
  Q_step[idx::kAccBias + 1][idx::kAccBias + 1] = q_ba;
  Q_step[idx::kAccBias + 2][idx::kAccBias + 2] = q_ba;
  Q_step[idx::kGyrBias + 0][idx::kGyrBias + 0] = q_bg;
  Q_step[idx::kGyrBias + 1][idx::kGyrBias + 1] = q_bg;
  Q_step[idx::kGyrBias + 2][idx::kGyrBias + 2] = q_bg;
  Q_step[idx::kBarBias][idx::kBarBias] = q_baro;

  // Accumulate Φ_total = Φ_k * ... * Φ_2 * Φ_1 over decimated samples.
  if (cov_decimation_counter_ == 0) {
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        cov_decimation_F_acc_[i][j] = F_step[i][j];
        cov_decimation_Q_acc_[i][j] = Q_step[i][j];
      }
    }
  } else {
    eskf_scalar F_next[kDimError][kDimError];
    eskf_scalar FQ[kDimError][kDimError];
    eskf_scalar Q_next[kDimError][kDimError];

    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        eskf_scalar sumF = 0;
        for (int k = 0; k < kDimError; ++k) {
          sumF += F_step[i][k] * cov_decimation_F_acc_[k][j];
        }
        F_next[i][j] = sumF;
      }
    }

    // Propagate accumulated process noise to the current step:
    // Q_acc_next = F_step * Q_acc_prev * F_step^T + Q_step
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        eskf_scalar sum = 0;
        for (int k = 0; k < kDimError; ++k) {
          sum += F_step[i][k] * cov_decimation_Q_acc_[k][j];
        }
        FQ[i][j] = sum;
      }
    }
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        eskf_scalar sum = 0;
        for (int k = 0; k < kDimError; ++k) {
          sum += FQ[i][k] * F_step[j][k];
        }
        Q_next[i][j] = sum + Q_step[i][j];
      }
    }

    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        cov_decimation_F_acc_[i][j] = F_next[i][j];
        cov_decimation_Q_acc_[i][j] = Q_next[i][j];
      }
    }
  }

  cov_decimation_dt_acc_s_ += dt_used;
  cov_decimation_counter_++;
  if (cov_decimation_counter_ >= ESKF_COVARIANCE_DECIMATION) {
    // Apply deferred covariance before any asynchronous correction can use P.
    flushDeferredCovariancePropagation();
  }
#else
  // Standard propagation every step
  eskf_scalar F[kDimError][kDimError];
  computeF(F, accel_body, dt_used);
  propagateCovariance(F, dt_used);
#endif

  // Update timestamp
  state_.timestamp_us = imu.timestamp_us;

  // Log IMU integration debug snapshot (decimated)
  if (++imu_dynamics_log_counter_ >= ESKF_IMU_DYNAMICS_LOG_DECIMATION) {
    imu_dynamics_log_counter_ = 0;
    ImuDynamicsSnapshot imu_snap{};
    for (int i = 0; i < 3; ++i) {
      imu_snap.accel_body[i] = static_cast<float>(accel_body[i]);
      imu_snap.gyro_body[i] = static_cast<float>(gyro_body[i]);
      imu_snap.accel_ned[i] = static_cast<float>(accel_ned[i]);
      imu_snap.gravity_ned[i] = static_cast<float>(g_ned[i]);
      imu_snap.vel_inc_ned[i] = static_cast<float>(vel_inc_ned[i]);
    }
    for (int i = 0; i < 4; ++i) {
      imu_snap.q[i] = static_cast<float>(state_.q[i]);
    }
    imu_snap.dt_s = static_cast<float>(dt_used);
    imu_snap.timestamp_us = state_.timestamp_us;
    getEskfLogger().logImuDynamics(imu_snap);
  }

  // Check for numerical issues
  checkNumericalHealth();
}

// ============================================================
// Correction Steps
// ============================================================

void EskfCore::correctGpsPosition(const eskf_scalar pos_ned[3],
                                  const eskf_scalar R[3]) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return;

  eskf_scalar total_nis = 0;

  // Sequential scalar updates for each axis
  for (int axis = 0; axis < 3; ++axis) {
    // Measurement model: z = p[axis] + noise
    // Jacobian H: all zeros except H[axis] = 1
    eskf_scalar z = pos_ned[axis];
    eskf_scalar h = state_.p[axis];

#if !ESKF_USE_CUSTOM_LINALG
    // Eigen backend
    math::RowVector15 H = math::RowVector15::Zero();
    H(0, idx::kPos + axis) = 1;

    math::Vector15 dx = math::Vector15::Zero();
    math::Matrix15 P;
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        P(i, j) = cov_.P[i][j];
      }
    }

    eskf_scalar nis = math::scalarUpdate(dx, P, z, h, H, R[axis]);
    total_nis += nis;

    // Copy back
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        cov_.P[i][j] = P(i, j);
      }
    }

    // Inject error state
    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx(i, 0);
    }
    injectErrorState(dx_arr);
#else
    // Custom backend
    eskf_scalar H[kDimError] = {};
    H[idx::kPos + axis] = 1;

    math::Vector15 dx = math::Vector15::Zero();
    eskf_scalar nis = math::scalarUpdate(
        dx, reinterpret_cast<math::Matrix15 &>(cov_.P), z, h, H, R[axis]);
    total_nis += nis;

    // Inject error state
    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx.data[i][0];
    }
    injectErrorState(dx_arr);
#endif
  }

  last_nis_ = total_nis;
  checkNumericalHealth();

  // Log correction event
  getEskfLogger().logCorrection(
      EskfEventType::GpsPosCorrection, state_.timestamp_us,
      static_cast<float>(last_innovation_), static_cast<float>(total_nis));
}

// Overload for backward compatibility (assumes zero lever arm)
void EskfCore::correctGpsVelocity(const eskf_scalar vel_ned[3],
                                  const eskf_scalar R[3]) {
  constexpr eskf_scalar zero_arm[3] = {0, 0, 0};
  correctGpsVelocity(vel_ned, R, zero_arm);
}

// GPS Velocity Correction with Lever-Arm Jacobians (Fix #2 from code review)
// The observation model: h_vel = v_cg + R_bn * (ω_body × r_arm)
// We correct measurement to CG: z_cg = z_ant - R_bn * (ω × r_arm)
//
// Full Jacobian terms:
// ∂h/∂(δv) = I (velocity directly observed)
// ∂h/∂(δθ) = -[R_bn * (ω × r)]× (attitude affects rotation of lever-arm
// velocity) ∂h/∂(δb_g) = R_bn * [r]× (gyro bias affects ω used in correction)
void EskfCore::correctGpsVelocity(const eskf_scalar vel_ned[3],
                                  const eskf_scalar R[3],
                                  const eskf_scalar lever_arm_body[3]) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return;

  eskf_scalar total_nis = 0;

  // Get current body rate estimate for lever arm correction.
  //
  // NOTE on timing: prev_gyro_ contains the bias-corrected gyro rate from the
  // PREVIOUS IMU prediction step. During GPS rewind (time-machine replay),
  // the filter is rewound to a checkpoint and replayed forward through IMU
  // samples, so prev_gyro_ is correctly updated to reflect the gyro rate
  // just before this GPS velocity measurement timestamp. For 6.4kHz IMU and
  // 16Hz GPS, this represents approximately 156µs staleness - acceptable for
  // the lever arm velocity calculation.
  eskf_scalar gyro_unbiased[3];
  if (first_run_) {
    gyro_unbiased[0] = 0;
    gyro_unbiased[1] = 0;
    gyro_unbiased[2] = 0;
  } else {
    gyro_unbiased[0] = prev_gyro_(0);
    gyro_unbiased[1] = prev_gyro_(1);
    gyro_unbiased[2] = prev_gyro_(2);
  }

  math::Vector3 w, r;
  w(0) = gyro_unbiased[0];
  w(1) = gyro_unbiased[1];
  w(2) = gyro_unbiased[2];
  r(0) = lever_arm_body[0];
  r(1) = lever_arm_body[1];
  r(2) = lever_arm_body[2];

  // ω × r (lever-arm velocity in body frame)
  math::Vector3 v_arm_body = w.cross(r);

  // Rotate lever-arm velocity to NED
  eskf_scalar v_arm_body_arr[3] = {v_arm_body(0), v_arm_body(1), v_arm_body(2)};
  eskf_scalar v_arm_ned[3];
  math::quatRotateVector(v_arm_ned, state_.q, v_arm_body_arr);

  // Corrected measurement (virtual measurement at CG)
  eskf_scalar vel_cg_meas[3];
  for (int i = 0; i < 3; ++i) {
    vel_cg_meas[i] = vel_ned[i] - v_arm_ned[i];
  }

  // Compute skew-symmetric matrices for Jacobian terms
  // [v_arm_ned]× for attitude Jacobian
  eskf_scalar v_arm_skew[3][3];
  math::skewSymmetric(v_arm_skew, v_arm_ned);

  // [r]× for gyro bias Jacobian
  eskf_scalar r_arr[3] = {r(0), r(1), r(2)};
  eskf_scalar r_skew[3][3];
  math::skewSymmetric(r_skew, r_arr);

  // Get rotation matrix for gyro bias Jacobian
  eskf_scalar R_bn[3][3];
  math::quatToDcm(R_bn, state_.q);

  // R_bn * [r]× (3×3 result for gyro bias contribution)
  eskf_scalar R_r_skew[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R_r_skew[i][j] = 0;
      for (int k = 0; k < 3; ++k) {
        R_r_skew[i][j] += R_bn[i][k] * r_skew[k][j];
      }
    }
  }

  const bool decouple_lever_arm_attitude =
      cfg_.disable_gps_vel_lever_arm_attitude_jacobian;
  const bool freeze_gyro_bias = freezeGyroBiasInFlight();

  for (int axis = 0; axis < 3; ++axis) {
    eskf_scalar z = vel_cg_meas[axis];
    eskf_scalar h = state_.v[axis];

#if !ESKF_USE_CUSTOM_LINALG
    math::RowVector15 H = math::RowVector15::Zero();

    // Velocity: direct observation
    H(0, idx::kVel + axis) = 1;

    // Attitude: ∂h/∂(δθ) = -[v_arm_ned]× (row 'axis')
    // This accounts for how rotation error affects the lever-arm velocity
    // projection
    H(0, idx::kAtt + 0) =
        decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][0];
    H(0, idx::kAtt + 1) =
        decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][1];
    H(0, idx::kAtt + 2) =
        decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][2];

    // Gyro Bias: ∂h/∂(δb_g) = R_bn * [r]× (row 'axis')
    // This accounts for how gyro bias error affects the ω used in lever-arm
    // correction
    H(0, idx::kGyrBias + 0) =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][0];
    H(0, idx::kGyrBias + 1) =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][1];
    H(0, idx::kGyrBias + 2) =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][2];

    math::Vector15 dx = math::Vector15::Zero();
    math::Matrix15 P;
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        P(i, j) = cov_.P[i][j];
      }
    }

    eskf_scalar nis = math::scalarUpdate(dx, P, z, h, H, R[axis]);
    total_nis += nis;

    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        cov_.P[i][j] = P(i, j);
      }
    }

    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx(i, 0);
    }
    injectErrorState(dx_arr);
#else
    eskf_scalar H[kDimError] = {};
    H[idx::kVel + axis] = 1;
    H[idx::kAtt + 0] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][0];
    H[idx::kAtt + 1] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][1];
    H[idx::kAtt + 2] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][2];
    H[idx::kGyrBias + 0] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][0];
    H[idx::kGyrBias + 1] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][1];
    H[idx::kGyrBias + 2] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][2];

    math::Vector15 dx = math::Vector15::Zero();
    eskf_scalar nis = math::scalarUpdate(
        dx, reinterpret_cast<math::Matrix15 &>(cov_.P), z, h, H, R[axis]);
    total_nis += nis;

    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx.data[i][0];
    }
    injectErrorState(dx_arr);
#endif
  }

  last_nis_ = total_nis;
  checkNumericalHealth();

  // Log correction event
  getEskfLogger().logCorrection(
      EskfEventType::GpsVelCorrection, state_.timestamp_us,
      static_cast<float>(last_innovation_), static_cast<float>(total_nis));
}

// GPS Velocity Correction with Pre-Computed Averaged Lever Arm
// Used when ESKF_USE_LEVER_ARM_AVERAGING is enabled.
// The averaged_lever_vel_ned is computed externally by averaging (ω × r) in NED
// over the GPS internal filter window.
void EskfCore::correctGpsVelocityWithAveragedLeverArm(
    const eskf_scalar vel_ned[3], const eskf_scalar R[3],
    const eskf_scalar lever_arm_body[3],
    const eskf_scalar averaged_lever_vel_ned[3]) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return;

  eskf_scalar total_nis = 0;

  // Use the pre-computed averaged lever arm velocity
  eskf_scalar v_arm_ned[3];
  for (int i = 0; i < 3; ++i) {
    v_arm_ned[i] = averaged_lever_vel_ned[i];
  }

  // Corrected measurement (virtual measurement at CG)
  eskf_scalar vel_cg_meas[3];
  for (int i = 0; i < 3; ++i) {
    vel_cg_meas[i] = vel_ned[i] - v_arm_ned[i];
  }

  // Compute skew-symmetric matrices for Jacobian terms
  // [v_arm_ned]× for attitude Jacobian
  eskf_scalar v_arm_skew[3][3];
  math::skewSymmetric(v_arm_skew, v_arm_ned);

  // [r]× for gyro bias Jacobian
  eskf_scalar r_skew[3][3];
  math::skewSymmetric(r_skew, lever_arm_body);

  // Get rotation matrix for gyro bias Jacobian
  eskf_scalar R_bn[3][3];
  math::quatToDcm(R_bn, state_.q);

  // R_bn * [r]× (3×3 result for gyro bias contribution)
  eskf_scalar R_r_skew[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R_r_skew[i][j] = 0;
      for (int k = 0; k < 3; ++k) {
        R_r_skew[i][j] += R_bn[i][k] * r_skew[k][j];
      }
    }
  }

  const bool decouple_lever_arm_attitude =
      cfg_.disable_gps_vel_lever_arm_attitude_jacobian;
  const bool freeze_gyro_bias = freezeGyroBiasInFlight();

  for (int axis = 0; axis < 3; ++axis) {
    eskf_scalar z = vel_cg_meas[axis];
    eskf_scalar h = state_.v[axis];

#if !ESKF_USE_CUSTOM_LINALG
    math::RowVector15 H = math::RowVector15::Zero();

    // Velocity: direct observation
    H(0, idx::kVel + axis) = 1;

    // Attitude: ∂h/∂(δθ) = -[v_arm_ned]×
    H(0, idx::kAtt + 0) =
        decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][0];
    H(0, idx::kAtt + 1) =
        decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][1];
    H(0, idx::kAtt + 2) =
        decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][2];

    // Gyro Bias: ∂h/∂(δb_g) = R_bn * [r]×
    H(0, idx::kGyrBias + 0) =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][0];
    H(0, idx::kGyrBias + 1) =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][1];
    H(0, idx::kGyrBias + 2) =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][2];

    math::Vector15 dx = math::Vector15::Zero();
    math::Matrix15 P;
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        P(i, j) = cov_.P[i][j];
      }
    }

    eskf_scalar nis = math::scalarUpdate(dx, P, z, h, H, R[axis]);
    total_nis += nis;

    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        cov_.P[i][j] = P(i, j);
      }
    }

    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx(i, 0);
    }
    injectErrorState(dx_arr);
#else
    eskf_scalar H[kDimError] = {};
    H[idx::kVel + axis] = 1;
    H[idx::kAtt + 0] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][0];
    H[idx::kAtt + 1] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][1];
    H[idx::kAtt + 2] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][2];
    H[idx::kGyrBias + 0] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][0];
    H[idx::kGyrBias + 1] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][1];
    H[idx::kGyrBias + 2] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][2];

    math::Vector15 dx = math::Vector15::Zero();
    eskf_scalar nis = math::scalarUpdate(
        dx, reinterpret_cast<math::Matrix15 &>(cov_.P), z, h, H, R[axis]);
    total_nis += nis;

    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx.data[i][0];
    }
    injectErrorState(dx_arr);
#endif
  }

  last_nis_ = total_nis;
  checkNumericalHealth();

  // Log correction event (same as regular GPS velocity)
  getEskfLogger().logCorrection(
      EskfEventType::GpsVelCorrection, state_.timestamp_us,
      static_cast<float>(last_innovation_), static_cast<float>(total_nis));
}

void EskfCore::correctBaroAltitude(eskf_scalar alt_m, eskf_scalar R) {
  flushDeferredCovariancePropagation();
  if (diverged_ && !nis_soft_diverged_)
    return;  // Hard divergence (NaN, negative P) — truly broken, stop.

  // Soft NIS divergence: keep fusing but with heavily inflated noise so the
  // corrections are very gentle.  This lets b_baro slowly adapt through the
  // process noise Q, eventually reducing NIS and enabling recovery.
  if (nis_soft_diverged_) {
    R *= 100.0;
  }

  // Barometer measures altitude (positive up).
  // In NED, Down is positive, so altitude = -p[2] + b_baro
  // z = alt_m = -p[2] + b_baro
  // h = -state_.p[2] + state_.b_baro
  // H = [0,0,-1, 0,0,0, 0,0,0, 0,0,0, 0,0,0] for p_D (index 2)
  // Note: b_baro is not in error state, handled separately

  eskf_scalar z = alt_m;
  eskf_scalar h = -state_.p[2] + state_.b_baro;

  // Innovation Clamping (Section 6.4.B)
  const eskf_scalar kMaxBaroInn = cfg_.baro_innovation_clamp;
  eskf_scalar y = z - h;
  if (y > kMaxBaroInn) {
    z = h + kMaxBaroInn; // Clamp z effectively
  } else if (y < -kMaxBaroInn) {
    z = h - kMaxBaroInn;
  }

#if !ESKF_USE_CUSTOM_LINALG
  math::RowVector15 H = math::RowVector15::Zero();
  H(0, idx::kPos + 2) = -1; // d(h)/d(p_D) = -1
  H(0, idx::kBarBias) = 1;  // d(h)/d(b_baro) = 1

  math::Vector15 dx = math::Vector15::Zero();
  math::Matrix15 P;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      P(i, j) = cov_.P[i][j];
    }
  }

  eskf_scalar nis = math::scalarUpdate(dx, P, z, h, H, R);
  last_nis_ = nis;

  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_.P[i][j] = P(i, j);
    }
  }

  eskf_scalar dx_arr[kDimError];
  for (int i = 0; i < kDimError; ++i) {
    dx_arr[i] = dx(i, 0);
  }
  injectErrorState(dx_arr);
#else
  eskf_scalar H[kDimError] = {};
  H[idx::kPos + 2] = -1;
  H[idx::kBarBias] = 1;

  math::Vector15 dx = math::Vector15::Zero();
  eskf_scalar nis = math::scalarUpdate(
      dx, reinterpret_cast<math::Matrix15 &>(cov_.P), z, h, H, R);
  last_nis_ = nis;

  eskf_scalar dx_arr[kDimError];
  for (int i = 0; i < kDimError; ++i) {
    dx_arr[i] = dx.data[i][0];
  }
  injectErrorState(dx_arr);
#endif

  checkNumericalHealth();

  // Log correction event
  getEskfLogger().logCorrection(
      EskfEventType::BaroCorrection, state_.timestamp_us,
      static_cast<float>(last_innovation_), static_cast<float>(last_nis_));
}

void EskfCore::correctBaroAltitudeAuto(eskf_scalar alt_m) {
  // Compute velocity-dependent R based on current state
  // R_baro = (σ_base + K_aero·v²)² with transonic penalty

  eskf_scalar v_sq = state_.v[0] * state_.v[0] + state_.v[1] * state_.v[1] +
                     state_.v[2] * state_.v[2];
  eskf_scalar speed = std::sqrt(v_sq);

  // Base variance plus velocity-squared inflation
  eskf_scalar sigma = cfg_.baro_sigma_base + cfg_.baro_k_aero * v_sq;

  // Transonic penalty (massive inflation in shock wave regime)
  if (speed > cfg_.baro_transonic_low && speed < cfg_.baro_transonic_high) {
    sigma += cfg_.baro_transonic_penalty;
  }

  eskf_scalar R = sigma * sigma;

  // Call the normal baro correction with computed R
  correctBaroAltitude(alt_m, R);
}

void EskfCore::correctBaroWithSnapshot(eskf_scalar measured_alt,
                                       eskf_scalar predicted_alt_at_trigger,
                                       eskf_scalar R) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return;

  // Innovation Transport (Section 3.4.A):
  // The key insight is that we calculate innovation using the SNAPSHOT from
  // trigger time, but apply the correction to the CURRENT state. This works
  // because:
  // 1. Position error covariance dynamics are slow (~10ms delay)
  // 2. We isolate true sensor error (not rocket motion during delay)
  //
  // Innovation: y = Measured - Predicted_at_trigger
  // This is applied to current state using current covariance.

  eskf_scalar z = measured_alt;
  eskf_scalar h = predicted_alt_at_trigger; // NOT current state - use snapshot!

  // Innovation Clamping (same as normal baro correction)
  const eskf_scalar kMaxBaroInn = cfg_.baro_innovation_clamp;
  eskf_scalar y = z - h;
  if (y > kMaxBaroInn) {
    z = h + kMaxBaroInn;
  } else if (y < -kMaxBaroInn) {
    z = h - kMaxBaroInn;
  }

  // Jacobian is same as normal baro (affects current state)
#if !ESKF_USE_CUSTOM_LINALG
  math::RowVector15 H = math::RowVector15::Zero();
  H(0, idx::kPos + 2) = -1; // d(h)/d(p_D) = -1
  H(0, idx::kBarBias) = 1;  // d(h)/d(b_baro) = 1

  math::Vector15 dx = math::Vector15::Zero();
  math::Matrix15 P;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      P(i, j) = cov_.P[i][j];
    }
  }

  eskf_scalar nis = math::scalarUpdate(dx, P, z, h, H, R);
  last_nis_ = nis;

  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_.P[i][j] = P(i, j);
    }
  }

  eskf_scalar dx_arr[kDimError];
  for (int i = 0; i < kDimError; ++i) {
    dx_arr[i] = dx(i, 0);
  }
  injectErrorState(dx_arr);
#else
  eskf_scalar H[kDimError] = {};
  H[idx::kPos + 2] = -1;
  H[idx::kBarBias] = 1;

  math::Vector15 dx = math::Vector15::Zero();
  eskf_scalar nis = math::scalarUpdate(
      dx, reinterpret_cast<math::Matrix15 &>(cov_.P), z, h, H, R);
  last_nis_ = nis;

  eskf_scalar dx_arr[kDimError];
  for (int i = 0; i < kDimError; ++i) {
    dx_arr[i] = dx.data[i][0];
  }
  injectErrorState(dx_arr);
#endif

  checkNumericalHealth();

  // Log correction event (same as regular baro)
  getEskfLogger().logCorrection(
      EskfEventType::BaroCorrection, state_.timestamp_us,
      static_cast<float>(last_innovation_), static_cast<float>(last_nis_));
}

bool EskfCore::attemptHeadingAlignment(const eskf_scalar gps_vel_ned[3],
                                       eskf_scalar gps_sAcc) {
  // One-shot: don't align if already aligned
  if (heading_aligned_) {
    return false;
  }

  if (diverged_) {
    return false;
  }

  // === Gate Conditions (Section 5.4.B) ===

  // 1. Minimum horizontal speed
  // This is the primary observability gate. A rocket at any pitch with
  // sufficient horizontal velocity has an observable heading from GPS velocity
  // vector.
  eskf_scalar v_horiz_sq =
      gps_vel_ned[0] * gps_vel_ned[0] + gps_vel_ned[1] * gps_vel_ned[1];
  eskf_scalar v_horiz = std::sqrt(v_horiz_sq);

  if (v_horiz < cfg_.heading_align_min_speed) {
    return false; // Not enough speed for reliable heading
  }

  if (gps_sAcc > cfg_.heading_align_max_sacc) {
    return false; // GPS too noisy
  }

  // NOTE: Pitch gate removed (Section 6 of architecture doc).
  // A rocket flying at 89° pitch at 100 m/s still has perfectly observable
  // heading from the horizontal velocity components. The minimum horizontal
  // speed gate above already ensures heading observability regardless of flight
  // path angle.

  // 3. Low rotation rate on transverse axes (pitch/yaw, not roll)
  //    Roll (X-axis) doesn't affect heading alignment - rocket can spin on its
  //    axis
  if (!first_run_) {
    eskf_scalar transverse_gyro_sq = prev_gyro_(1) * prev_gyro_(1) + // Y: pitch
                                     prev_gyro_(2) * prev_gyro_(2);  // Z: yaw
    eskf_scalar transverse_gyro = std::sqrt(transverse_gyro_sq);

    if (transverse_gyro > cfg_.heading_align_max_gyro) {
      return false; // Rotating too fast on transverse axes
    }
  }

  // === All gates passed - perform alignment ===

  // Compute heading from GPS velocity (course over ground).
  eskf_scalar gps_heading = std::atan2(gps_vel_ned[1], gps_vel_ned[0]);
  eskf_scalar snapped_heading = gps_heading;

  if (cfg_.gps_heading_bootstrap_mode ==
      GpsHeadingBootstrapMode::VelocityAngleDelta) {
    // Wind can bias COG away from body yaw. Compare GNSS and ESKF velocity
    // directions and only apply their angular delta to yaw.
    eskf_scalar eskf_v_horiz_sq =
        state_.v[0] * state_.v[0] + state_.v[1] * state_.v[1];
    if (eskf_v_horiz_sq > static_cast<eskf_scalar>(1e-6f)) {
      eskf_scalar eskf_heading = std::atan2(state_.v[1], state_.v[0]);
      eskf_scalar delta_yaw = math::wrapPi(gps_heading - eskf_heading);

      eskf_scalar R_dcm[3][3];
      math::quatToDcm(R_dcm, state_.q);
      eskf_scalar current_yaw = std::atan2(R_dcm[1][0], R_dcm[0][0]);
      snapped_heading = math::wrapPi(current_yaw + delta_yaw);
    }
  }

  last_heading_meas_rad_ = gps_heading;

  // Apply hard snap with GPS velocity override
  // GPS provides "gold standard" velocity - overwrite state.v
  getEskfLogger().logEvent(EskfEventType::HeadingStateSnapshotPre,
                           state_.timestamp_us, 1.0f);
  getEskfLogger().logStateCritical(createStateSnapshot());
  forceYaw(snapped_heading, true, gps_vel_ned, true);
  getEskfLogger().logStateCritical(createStateSnapshot());
  getEskfLogger().logEvent(EskfEventType::HeadingStateSnapshotPost,
                           state_.timestamp_us, 1.0f);

  // Mark as aligned (one-shot)
  heading_aligned_ = true;
  heading_initialized_ = true; // GPS alignment also initializes heading

  // Log heading alignment event
  getEskfLogger().logEvent(EskfEventType::HeadingAligned, state_.timestamp_us,
                           static_cast<float>(snapped_heading));
  last_heading_update_result_ = HeadingUpdateResult::Snapped;
  last_heading_innovation_rad_ = 0;
  last_heading_gate_threshold_ = 0;

  return true;
}

void EskfCore::correctHeadingWithEvent(eskf_scalar heading_rad, eskf_scalar R,
                                       EskfEventType correction_event) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return;

  // Extract current heading from quaternion
  // Heading = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
  const eskf_scalar qw = state_.q[0], qx = state_.q[1];
  const eskf_scalar qy = state_.q[2], qz = state_.q[3];

  eskf_scalar current_heading =
      std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

  // Wrap angle difference to [-π, π]
  eskf_scalar innovation = heading_rad - current_heading;
  while (innovation > constants::kPi)
    innovation -= 2 * constants::kPi;
  while (innovation < -constants::kPi)
    innovation += 2 * constants::kPi;

  // Jacobian: H affects yaw (δθ_z at index 8)
  // For small angles, Δheading ≈ δθ_z
#if !ESKF_USE_CUSTOM_LINALG
  math::RowVector15 H = math::RowVector15::Zero();
  H(0, idx::kAtt + 2) = 1; // d(heading)/d(δθ_z) ≈ 1

  math::Vector15 dx = math::Vector15::Zero();
  math::Matrix15 P;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      P(i, j) = cov_.P[i][j];
    }
  }

  // Use innovation directly as measurement - prediction difference
  eskf_scalar nis = math::scalarUpdate(dx, P, innovation, 0, H, R);
  last_nis_ = nis;

  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_.P[i][j] = P(i, j);
    }
  }

  eskf_scalar dx_arr[kDimError];
  for (int i = 0; i < kDimError; ++i) {
    dx_arr[i] = dx(i, 0);
  }
  injectErrorState(dx_arr);
#else
  eskf_scalar H[kDimError] = {};
  H[idx::kAtt + 2] = 1;

  math::Vector15 dx = math::Vector15::Zero();
  eskf_scalar nis = math::scalarUpdate(
      dx, reinterpret_cast<math::Matrix15 &>(cov_.P), innovation, 0, H, R);
  last_nis_ = nis;

  eskf_scalar dx_arr[kDimError];
  for (int i = 0; i < kDimError; ++i) {
    dx_arr[i] = dx.data[i][0];
  }
  injectErrorState(dx_arr);
#endif

  checkNumericalHealth();

  // Log correction event
  getEskfLogger().logCorrection(correction_event, state_.timestamp_us,
                                static_cast<float>(last_innovation_),
                                static_cast<float>(last_nis_));
}

void EskfCore::correctMagHeading(eskf_scalar heading_rad, eskf_scalar R) {
  correctHeadingWithEvent(heading_rad, R, EskfEventType::MagCorrection);
}

void EskfCore::forceYaw(eskf_scalar heading_rad, bool reset_covariance,
                        const eskf_scalar *explicit_vel_ned,
                        bool rotate_position,
                        bool rotate_velocity) {
  flushDeferredCovariancePropagation();
  // 1. Get current yaw and compute delta
  eskf_scalar R_dcm[3][3];
  math::quatToDcm(R_dcm, state_.q);
  eskf_scalar current_psi = atan2(R_dcm[1][0], R_dcm[0][0]);

  eskf_scalar delta_psi = heading_rad - current_psi;

  // 2. Rotate quaternion around Z axis (Down in NED)
  eskf_scalar dq[4];
  eskf_scalar half = 0.5 * delta_psi;
  dq[0] = cos(half);
  dq[1] = 0;
  dq[2] = 0;
  dq[3] = sin(half);

  // Apply rotation: q_new = dq * q_old (Global Z rotation in NED frame)
  eskf_scalar q_new[4];
  math::quatMultiply(q_new, dq, state_.q);
  for (int i = 0; i < 4; i++)
    state_.q[i] = q_new[i];
  math::quatNormalize(state_.q);

  eskf_scalar cos_d = std::cos(delta_psi);
  eskf_scalar sin_d = std::sin(delta_psi);

  // 3. Position handling
  // Bootstrap heading alignment rotates N/E position to re-anchor the full
  // inertial trajectory. Non-bootstrap heading snaps can skip position
  // rotation to avoid mid-flight teleports during temporary heading outliers.
  if (rotate_position) {
    eskf_scalar old_p_n = state_.p[0];
    eskf_scalar old_p_e = state_.p[1];
    eskf_scalar p_n = old_p_n * cos_d - old_p_e * sin_d;
    eskf_scalar p_e = old_p_n * sin_d + old_p_e * cos_d;
    state_.p[0] = p_n;
    state_.p[1] = p_e;
    // state_.p[2] (Down) unchanged

    if (late_gps_origin_) {
      gps_ned_offset_n_ += p_n - old_p_n;
      gps_ned_offset_e_ += p_e - old_p_e;
    }
  }

  // 4. Handle velocity based on source
  if (explicit_vel_ned != nullptr) {
    // GPS provides "gold standard" velocity - OVERWRITE state
    // This removes magnitude drift from IMU integration
    state_.v[0] = explicit_vel_ned[0];
    state_.v[1] = explicit_vel_ned[1];
    state_.v[2] = explicit_vel_ned[2];
  } else if (rotate_velocity) {
    // Magnetometer case - ROTATE existing velocity by delta_psi
    // Mag gives direction but we trust IMU for speed magnitude
    eskf_scalar v_n = state_.v[0] * cos_d - state_.v[1] * sin_d;
    eskf_scalar v_e = state_.v[0] * sin_d + state_.v[1] * cos_d;
    state_.v[0] = v_n;
    state_.v[1] = v_e;
    // state_.v[2] (Down) unchanged
  }

  // 5. Reset covariance if requested
  if (reset_covariance) {
    // Reset yaw variance (idx::kAtt + 2 = index 8)
    cov_.P[idx::kAtt + 2][idx::kAtt + 2] = cfg_.post_align_yaw_var;

    // Zero out yaw cross-correlations for numerical stability
    for (int i = 0; i < kDimError; ++i) {
      if (i != idx::kAtt + 2) {
        cov_.P[idx::kAtt + 2][i] = 0;
        cov_.P[i][idx::kAtt + 2] = 0;
      }
    }

    // If GPS velocity was used, also reset velocity covariance
    if (explicit_vel_ned != nullptr) {
      constexpr eskf_scalar kGpsVelVar = 0.25; // 0.5 m/s std dev
      // Overwriting velocity state requires a fully consistent velocity
      // covariance block. Keep it diagonal and clear all cross-terms.
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < kDimError; ++j) {
          cov_.P[idx::kVel + i][j] = 0;
          cov_.P[j][idx::kVel + i] = 0;
        }
      }
      cov_.P[idx::kVel + 0][idx::kVel + 0] = kGpsVelVar;
      cov_.P[idx::kVel + 1][idx::kVel + 1] = kGpsVelVar;
      cov_.P[idx::kVel + 2][idx::kVel + 2] = kGpsVelVar;
    }
  }
}

void EskfCore::resetYawCovariance() {
  flushDeferredCovariancePropagation();
  cov_.P[idx::kAtt + 2][idx::kAtt + 2] = cfg_.post_align_yaw_var;

  // Zero yaw cross-correlations
  for (int i = 0; i < kDimError; ++i) {
    if (i != idx::kAtt + 2) {
      cov_.P[idx::kAtt + 2][i] = 0;
      cov_.P[i][idx::kAtt + 2] = 0;
    }
  }
}

void EskfCore::resetVerticalChannelFromBaro(
    eskf_scalar altitude_isa_m,
    eskf_scalar baro_measurement_variance) {
  flushDeferredCovariancePropagation();
  if (!std::isfinite(altitude_isa_m)) {
    return;
  }

  // Baro model: altitude = -p_D + b_baro  =>  p_D = b_baro - altitude
  state_.p[2] = state_.b_baro - altitude_isa_m;

  const int down_idx = idx::kPos + 2;
  const eskf_scalar baro_bias_var =
      std::max<eskf_scalar>(0, cov_.P[idx::kBarBias][idx::kBarBias]);
  const eskf_scalar meas_var =
      std::isfinite(baro_measurement_variance) && baro_measurement_variance > 0
          ? baro_measurement_variance
          : 0;

  for (int i = 0; i < kDimError; ++i) {
    if (i == down_idx) {
      continue;
    }
    cov_.P[down_idx][i] = 0;
    cov_.P[i][down_idx] = 0;
  }
  cov_.P[down_idx][down_idx] = meas_var + baro_bias_var;
}

void EskfCore::resetIntegrationState() {
  prev_gyro_.setZero();
  prev_accel_body_.setZero();
  prev_vel_.setZero();
  first_run_ = true;
  cov_decimation_counter_ = 0;
#if ESKF_COVARIANCE_DECIMATION > 1
  cov_decimation_dt_acc_s_ = 0;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_decimation_F_acc_[i][j] = (i == j) ? 1 : 0;
      cov_decimation_Q_acc_[i][j] = 0;
    }
  }
#endif
}

HeadingUpdateResult EskfCore::processHeadingUpdate(
  eskf_scalar heading_rad, eskf_scalar R,
  EskfEventType fused_correction_event,
  bool allow_resurrection) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return HeadingUpdateResult::Ignored;
  last_heading_meas_rad_ = heading_rad;

  // First heading = SNAP (never fuse update into uninitialized state)
  if (!heading_initialized_) {
    getEskfLogger().logEvent(EskfEventType::HeadingStateSnapshotPre,
                             state_.timestamp_us, 2.0f);
    getEskfLogger().logStateCritical(createStateSnapshot());
    forceYaw(heading_rad, true, nullptr, false, false);
    getEskfLogger().logStateCritical(createStateSnapshot());
    getEskfLogger().logEvent(EskfEventType::HeadingStateSnapshotPost,
                             state_.timestamp_us, 2.0f);
    heading_initialized_ = true;
    consecutive_heading_rejects_ = 0;

    // Log heading initialization (first snap)
    getEskfLogger().logEvent(EskfEventType::HeadingInitialized,
                             state_.timestamp_us,
                             static_cast<float>(heading_rad));
    last_heading_update_result_ = HeadingUpdateResult::Snapped;
    last_heading_innovation_rad_ = 0;
    last_heading_gate_threshold_ = 0;
    return HeadingUpdateResult::Snapped;
  }

  // Calculate innovation
  const eskf_scalar qw = state_.q[0], qx = state_.q[1];
  const eskf_scalar qy = state_.q[2], qz = state_.q[3];
  eskf_scalar current_yaw =
      std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

  eskf_scalar innovation = heading_rad - current_yaw;
  while (innovation > constants::kPi)
    innovation -= 2 * constants::kPi;
  while (innovation < -constants::kPi)
    innovation += 2 * constants::kPi;
  last_heading_innovation_rad_ = innovation;

  // Gating: 3-sigma check
  eskf_scalar P_yaw = cov_.P[idx::kAtt + 2][idx::kAtt + 2];
  eskf_scalar S = P_yaw + R;
  constexpr eskf_scalar kGateSigma = 3.0;
  eskf_scalar gate_threshold = kGateSigma * kGateSigma * S;
  eskf_scalar innovation_sq = innovation * innovation;
  last_heading_gate_threshold_ = gate_threshold;

  if (innovation_sq < gate_threshold) {
    // Inside gate: fuse normally
    correctHeadingWithEvent(heading_rad, R, fused_correction_event);
    consecutive_heading_rejects_ = 0;
    last_heading_update_result_ = HeadingUpdateResult::Fused;
    return HeadingUpdateResult::Fused;
  }

  // Outside gate: outlier
  consecutive_heading_rejects_++;

  // Resurrection: if consistently rejected, filter may be lost
  if (allow_resurrection &&
      consecutive_heading_rejects_ >= cfg_.heading_resurrect_count) {
    getEskfLogger().logEvent(EskfEventType::HeadingStateSnapshotPre,
                             state_.timestamp_us, 3.0f);
    getEskfLogger().logStateCritical(createStateSnapshot());
    forceYaw(heading_rad, false, nullptr, false, false); // Snap without position/velocity teleport
    getEskfLogger().logStateCritical(createStateSnapshot());
    getEskfLogger().logEvent(EskfEventType::HeadingStateSnapshotPost,
                             state_.timestamp_us, 3.0f);
    resetYawCovariance();
    consecutive_heading_rejects_ = 0;

    // Log resurrection snap
    getEskfLogger().logEvent(EskfEventType::HeadingSnapped, state_.timestamp_us,
                             static_cast<float>(heading_rad));
    last_heading_update_result_ = HeadingUpdateResult::Resurrected;
    return HeadingUpdateResult::Resurrected;
  }

  last_heading_update_result_ = HeadingUpdateResult::Rejected;
  return HeadingUpdateResult::Rejected;
}

void EskfCore::correctVelocityHeading(eskf_scalar heading_rad, eskf_scalar R) {
  correctHeadingWithEvent(heading_rad, R,
                          EskfEventType::GpsHeadingCorrection);
}

#if ESKF_ENABLE_SIDESLIP
void EskfCore::correctSideslip(eskf_scalar R_lateral) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return;

  // Sideslip constraint: lateral body velocity ≈ 0
  // v_body = R' * v_ned
  // We constrain v_body[1] (right) ≈ 0

  eskf_scalar v_body[3];
  math::quatRotateVectorInverse(v_body, state_.q, state_.v);

  // z = 0 (target lateral velocity)
  // h = v_body[1] (current lateral velocity)
  eskf_scalar z = 0;
  eskf_scalar h = v_body[1];

  // Yaw-only sideslip Jacobian (decoupled update).
  // h = e_y^T * R_nb^T * v_ned
  // ∂h/∂(δθ) = e_y^T * R_nb^T * [v_ned]×  (attitude → heading correction)
  // ∂h/∂(δv) = 0  (intentionally zeroed)
  //
  // Velocity is already tightly constrained by GPS. Zeroing H_vel makes
  // sideslip a pure attitude correction: 100% of the v_body_y innovation
  // is attributed to yaw error. Velocity is still indirectly pulled via
  // cross-covariance P_va (K_v = P_va * H_att^T / S), which maintains
  // matrix positive-definiteness without aggressive direct velocity pulls.

  eskf_scalar R_nb[3][3];
  math::quatToDcm(R_nb, state_.q);

  // Compute [v_ned]× (skew-symmetric matrix of velocity)
  eskf_scalar v_skew[3][3];
  math::skewSymmetric(v_skew, state_.v);

  // H_att = e_y^T * R_nb^T * [v]× = R_nb[:,1]^T * [v]×
  // Result is a 1×3 row vector
  eskf_scalar H_att[3];
  for (int j = 0; j < 3; ++j) {
    H_att[j] = 0;
    for (int k = 0; k < 3; ++k) {
      H_att[j] += R_nb[k][1] * v_skew[k][j];
    }
  }

#if !ESKF_USE_CUSTOM_LINALG
  math::RowVector15 H = math::RowVector15::Zero();
  // H_vel intentionally zeroed — sideslip only corrects attitude (yaw).
  // H(0, idx::kVel + 0) = R_nb[0][1];  // zeroed
  // H(0, idx::kVel + 1) = R_nb[1][1];  // zeroed
  // H(0, idx::kVel + 2) = R_nb[2][1];  // zeroed

  // Strict heading-only path: only keep yaw-axis attitude sensitivity.
  // Roll/pitch terms are intentionally zeroed so sideslip cannot pull tilt.
  H(0, idx::kAtt + 0) = 0;
  H(0, idx::kAtt + 1) = 0;
  H(0, idx::kAtt + 2) = H_att[2];

  math::Vector15 dx = math::Vector15::Zero();
  math::Matrix15 P;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      P(i, j) = cov_.P[i][j];
    }
  }

  eskf_scalar nis = math::scalarUpdate(dx, P, z, h, H, R_lateral);
  last_nis_ = nis;

  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_.P[i][j] = P(i, j);
    }
  }

  eskf_scalar dx_arr[kDimError];
  for (int i = 0; i < kDimError; ++i) {
    dx_arr[i] = dx(i, 0);
  }
  injectErrorState(dx_arr);
#else
  eskf_scalar H[kDimError] = {};
  // H_vel intentionally zeroed — sideslip only corrects attitude (yaw).
  // H[idx::kVel + 0] = R_nb[0][1];  // zeroed
  // H[idx::kVel + 1] = R_nb[1][1];  // zeroed
  // H[idx::kVel + 2] = R_nb[2][1];  // zeroed
  H[idx::kAtt + 0] = 0;
  H[idx::kAtt + 1] = 0;
  H[idx::kAtt + 2] = H_att[2];

  math::Vector15 dx = math::Vector15::Zero();
  eskf_scalar nis = math::scalarUpdate(
      dx, reinterpret_cast<math::Matrix15 &>(cov_.P), z, h, H, R_lateral);
  last_nis_ = nis;

  eskf_scalar dx_arr[kDimError];
  for (int i = 0; i < kDimError; ++i) {
    dx_arr[i] = dx.data[i][0];
  }
  injectErrorState(dx_arr);
#endif

  checkNumericalHealth();

  // Log correction event
  getEskfLogger().logCorrection(
      EskfEventType::SideslipCorrection, state_.timestamp_us,
      static_cast<float>(last_innovation_), static_cast<float>(last_nis_));
}
#endif // ESKF_ENABLE_SIDESLIP

// ============================================================
// Static Constraint Updates (ZUPT/ZARU)
// ============================================================

void EskfCore::applyZupt(const eskf_scalar R[3]) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return;

  eskf_scalar total_nis = 0;

  // ZUPT: Constrain velocity to zero
  // Measurement model: z = 0 (expected velocity when stationary)
  // h = v[axis] (predicted velocity from state)
  for (int axis = 0; axis < 3; ++axis) {
    eskf_scalar z = 0; // Expected: zero velocity
    eskf_scalar h = state_.v[axis];

#if !ESKF_USE_CUSTOM_LINALG
    math::RowVector15 H = math::RowVector15::Zero();
    H(0, idx::kVel + axis) = 1;

    math::Vector15 dx = math::Vector15::Zero();
    math::Matrix15 P;
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        P(i, j) = cov_.P[i][j];
      }
    }

    eskf_scalar nis = math::scalarUpdate(dx, P, z, h, H, R[axis]);
    total_nis += nis;

    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        cov_.P[i][j] = P(i, j);
      }
    }

    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx(i, 0);
    }
    injectErrorState(dx_arr);
#else
    eskf_scalar H[kDimError] = {};
    H[idx::kVel + axis] = 1;

    math::Vector15 dx = math::Vector15::Zero();
    eskf_scalar nis = math::scalarUpdate(
        dx, reinterpret_cast<math::Matrix15 &>(cov_.P), z, h, H, R[axis]);
    total_nis += nis;

    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx.data[i][0];
    }
    injectErrorState(dx_arr);
#endif
  }

  last_nis_ = total_nis;
  checkNumericalHealth();

  // Log ZUPT event
  getEskfLogger().logEvent(EskfEventType::ZuptApplied, state_.timestamp_us,
                           static_cast<float>(total_nis));
}

void EskfCore::applyZaru(const eskf_scalar R[3]) {
  flushDeferredCovariancePropagation();
  if (diverged_)
    return;

  eskf_scalar total_nis = 0;

  // ZARU: Zero Angular Rate Update (Direct Bias Observation)
  //
  // Implementation: Direct Bias Observation
  // When stationary on pad, we assume true angular rate ω_true = 0.
  // Therefore: ω_measured = ω_true + b_gyro + noise = b_gyro + noise
  //
  // Measurement model:
  //   z = ω_raw (measured gyro rate)
  //   h = b_gyro (current bias estimate)
  //   Innovation: z - h = ω_raw - b_gyro = ω_true + noise ≈ 0 + noise
  //
  // This directly observes the gyro bias from the raw measurements.
  // But we don't have raw gyro here. Instead, we constrain:
  // The last measured rate (prev_gyro_) should equal the bias.
  //
  // Measurement model: z = prev_gyro_[axis] (the measured rate)
  // h = b_gyro[axis] (the estimated bias)
  // Innovation: z - h = prev_gyro_ - b_gyro = true_rate ≈ 0

  for (int axis = 0; axis < 3; ++axis) {
    // We use the last integrated rate as the "measurement"
    // This is the bias-corrected value, so we need uncorrected.
    // Actually, for ZARU we constrain that b_gyro should equal prev_gyro_
    // when stationary (since ω_true ≈ 0 → ω_meas ≈ b_gyro)
    eskf_scalar z = prev_gyro_(axis); // Bias-corrected rate
    // True measurement is bias-corrected + bias = raw. Since we want raw =
    // bias: z_raw = prev_gyro_ + b_gyro, expect z_raw = b_gyro → prev_gyro_ = 0
    // So we're constraining prev_gyro_ to zero, which is already
    // bias-corrected!

    // Actually simpler: when stationary, bias-corrected rate should be zero.
    // So z = 0, h = prev_gyro_ - but that's already done in prediction.
    // For ZARU, we use indirect observation: the bias should explain the rate.
    // z = 0 (expected true rate), h = -b_gyro (rate due to bias error)
    // But this is tricky. Let's use the standard ZARU formulation:
    // We measure ω_raw and expect it to equal b_gyro when ω_true = 0.
    // So the measurement is: z = ω_raw, H affects b_gyro, h = b_gyro.

    // For our implementation, prev_gyro_ is bias-corrected.
    // So raw = prev_gyro_ + state_.b_gyro
    eskf_scalar omega_raw = prev_gyro_(axis) + state_.b_gyro[axis];
    z = omega_raw;
    eskf_scalar h = state_.b_gyro[axis];

#if !ESKF_USE_CUSTOM_LINALG
    math::RowVector15 H = math::RowVector15::Zero();
    H(0, idx::kGyrBias + axis) = 1;

    math::Vector15 dx = math::Vector15::Zero();
    math::Matrix15 P;
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        P(i, j) = cov_.P[i][j];
      }
    }

    eskf_scalar nis = math::scalarUpdate(dx, P, z, h, H, R[axis]);
    total_nis += nis;

    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        cov_.P[i][j] = P(i, j);
      }
    }

    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx(i, 0);
    }
    injectErrorState(dx_arr);
#else
    eskf_scalar H[kDimError] = {};
    H[idx::kGyrBias + axis] = 1;

    math::Vector15 dx = math::Vector15::Zero();
    eskf_scalar nis = math::scalarUpdate(
        dx, reinterpret_cast<math::Matrix15 &>(cov_.P), z, h, H, R[axis]);
    total_nis += nis;

    eskf_scalar dx_arr[kDimError];
    for (int i = 0; i < kDimError; ++i) {
      dx_arr[i] = dx.data[i][0];
    }
    injectErrorState(dx_arr);
#endif
  }

  last_nis_ = total_nis;
  checkNumericalHealth();

  // Log ZARU event
  getEskfLogger().logEvent(EskfEventType::ZaruApplied, state_.timestamp_us,
                           static_cast<float>(total_nis));
}

// ============================================================
// Error State Injection
// ============================================================

void EskfCore::injectErrorState(const eskf_scalar dx[kDimError]) {
  // Position
  for (int i = 0; i < 3; ++i) {
    state_.p[i] += dx[idx::kPos + i];
  }

  // Velocity
  for (int i = 0; i < 3; ++i) {
    state_.v[i] += dx[idx::kVel + i];
  }

  // Attitude: q ← exp(δθ/2) ⊗ q  (LEFT MULTIPLICATION)
  eskf_scalar dtheta[3] = {dx[idx::kAtt], dx[idx::kAtt + 1], dx[idx::kAtt + 2]};
  eskf_scalar dq[4];
  math::quatFromRotationVector(dq, dtheta);

  eskf_scalar q_new[4];
  math::quatMultiply(q_new, dq, state_.q);
  for (int i = 0; i < 4; ++i) {
    state_.q[i] = q_new[i];
  }
  math::quatNormalize(state_.q);

  // Accelerometer bias
  if (!freezeAccelBiasInFlight()) {
    for (int i = 0; i < 3; ++i) {
      state_.b_acc[i] += dx[idx::kAccBias + i];
    }
  }

  // Gyroscope bias
  if (!freezeGyroBiasInFlight()) {
    for (int i = 0; i < 3; ++i) {
      state_.b_gyro[i] += dx[idx::kGyrBias + i];
    }
  }

  // Barometer bias
  state_.b_baro += dx[idx::kBarBias];
}

// ============================================================
// Jacobian Computation
// ============================================================

void EskfCore::computeF(eskf_scalar F[kDimError][kDimError],
                        const eskf_scalar accel_body[3], eskf_scalar dt) {
  // Initialize F as identity
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      F[i][j] = (i == j) ? 1 : 0;
    }
  }

  // Get rotation matrix body→NED
  eskf_scalar R_bn[3][3];
  math::quatToDcm(R_bn, state_.q);

  // Rotate acceleration to NED
  eskf_scalar accel_ned[3];
  math::quatRotateVector(accel_ned, state_.q, accel_body);

  // Skew-symmetric matrix of accel_ned
  eskf_scalar accel_skew[3][3];
  math::skewSymmetric(accel_skew, accel_ned);

  // === F matrix blocks ===
  // Error state: [δp, δv, δθ, δb_a, δb_g]

  // δp row (0-2):
  // d(δp)/d(δv) = I * dt
  for (int i = 0; i < 3; ++i) {
    F[idx::kPos + i][idx::kVel + i] = dt;
  }

  // δv row (3-5):
  // d(δv)/d(δθ) = -[R*a]× * dt = -[accel_ned]× * dt
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      F[idx::kVel + i][idx::kAtt + j] = -accel_skew[i][j] * dt;
    }
  }

  // d(δv)/d(δb_a) = -R * dt
  if (!freezeAccelBiasInFlight()) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        F[idx::kVel + i][idx::kAccBias + j] = -R_bn[i][j] * dt;
      }
    }
  }

  // δθ row (6-8):
  // d(δθ)/d(δb_g) = -R_bn * dt
  // NOTE: We use Global Error definition for attitude. Bias (Body) must be
  // rotated to NED.
  if (!freezeGyroBiasInFlight()) {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        F[idx::kAtt + i][idx::kGyrBias + j] = -R_bn[i][j] * dt;
      }
    }
  }

  // Bias rows (9-14): random walk, stay at identity (already set)
}

void EskfCore::propagateCovariance(const eskf_scalar F[kDimError][kDimError],
                                   eskf_scalar dt) {
  // P = F * P * F' + Q_d
  // Q_d = G * Q_c * G' * dt (discretized process noise)

  // For simplicity, use P = F*P*F' + Q_d where Q_d is diagonal

  // Temporary matrices
  eskf_scalar FP[kDimError][kDimError];
  eskf_scalar FPFt[kDimError][kDimError];

  // FP = F * P (exploit sparse F structure, row-major access)
  for (int i = 0; i < 3; ++i) {
    const int row = idx::kPos + i;
    const int vrow = idx::kVel + i;
    for (int j = 0; j < kDimError; ++j) {
      FP[row][j] = cov_.P[row][j] + F[row][vrow] * cov_.P[vrow][j];
    }
  }
  for (int i = 0; i < 3; ++i) {
    const int row = idx::kVel + i;
    for (int j = 0; j < kDimError; ++j) {
      eskf_scalar sum = cov_.P[row][j];
      for (int k = 0; k < 3; ++k) {
        sum += F[row][idx::kAtt + k] * cov_.P[idx::kAtt + k][j];
        sum += F[row][idx::kAccBias + k] * cov_.P[idx::kAccBias + k][j];
      }
      FP[row][j] = sum;
    }
  }
  for (int i = 0; i < 3; ++i) {
    const int row = idx::kAtt + i;
    for (int j = 0; j < kDimError; ++j) {
      eskf_scalar sum = cov_.P[row][j];
      for (int k = 0; k < 3; ++k) {
        sum += F[row][idx::kGyrBias + k] * cov_.P[idx::kGyrBias + k][j];
      }
      FP[row][j] = sum;
    }
  }

  // Bias rows are identity (copy through)
  std::memcpy(FP[idx::kAccBias], cov_.P[idx::kAccBias],
              sizeof(eskf_scalar) * kDimError);
  std::memcpy(FP[idx::kAccBias + 1], cov_.P[idx::kAccBias + 1],
              sizeof(eskf_scalar) * kDimError);
  std::memcpy(FP[idx::kAccBias + 2], cov_.P[idx::kAccBias + 2],
              sizeof(eskf_scalar) * kDimError);
  std::memcpy(FP[idx::kGyrBias], cov_.P[idx::kGyrBias],
              sizeof(eskf_scalar) * kDimError);
  std::memcpy(FP[idx::kGyrBias + 1], cov_.P[idx::kGyrBias + 1],
              sizeof(eskf_scalar) * kDimError);
  std::memcpy(FP[idx::kGyrBias + 2], cov_.P[idx::kGyrBias + 2],
              sizeof(eskf_scalar) * kDimError);
  std::memcpy(FP[idx::kBarBias], cov_.P[idx::kBarBias],
              sizeof(eskf_scalar) * kDimError);

  // FPFt = FP * F' (sparse, row-wise with F row sparsity)
  // Only compute upper triangle (j >= i) then mirror.
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < 3; ++j) {
      const int col = idx::kPos + j;
      const int vcol = idx::kVel + j;
      if (col >= i) {
        FPFt[i][col] = FP[i][col] + F[col][vcol] * FP[i][vcol];
      }
    }
    for (int j = 0; j < 3; ++j) {
      const int col = idx::kVel + j;
      if (col >= i) {
        eskf_scalar sum = FP[i][col];
        for (int k = 0; k < 3; ++k) {
          sum += FP[i][idx::kAtt + k] * F[col][idx::kAtt + k];
          sum += FP[i][idx::kAccBias + k] * F[col][idx::kAccBias + k];
        }
        FPFt[i][col] = sum;
      }
    }
    for (int j = 0; j < 3; ++j) {
      const int col = idx::kAtt + j;
      if (col >= i) {
        eskf_scalar sum = FP[i][col];
        for (int k = 0; k < 3; ++k) {
          sum += FP[i][idx::kGyrBias + k] * F[col][idx::kGyrBias + k];
        }
        FPFt[i][col] = sum;
      }
    }
    for (int j = 0; j < 3; ++j) {
      const int acc_col = idx::kAccBias + j;
      const int gyr_col = idx::kGyrBias + j;
      if (acc_col >= i) {
        FPFt[i][acc_col] = FP[i][acc_col];
      }
      if (gyr_col >= i) {
        FPFt[i][gyr_col] = FP[i][gyr_col];
      }
    }
    if (idx::kBarBias >= i) {
      FPFt[i][idx::kBarBias] = FP[i][idx::kBarBias];
    }
  }

  for (int i = 0; i < kDimError; ++i) {
    for (int j = i + 1; j < kDimError; ++j) {
      FPFt[j][i] = FPFt[i][j];
    }
  }

  // Add process noise Q_d to diagonal
  // Q_d = diag(0, 0, 0,                   // position (driven by velocity)
  //            σ_a² * dt, σ_a² * dt, σ_a² * dt,  // velocity
  //            σ_g² * dt, σ_g² * dt, σ_g² * dt,  // attitude
  //            σ_ba² * dt, σ_ba² * dt, σ_ba² * dt,  // accel bias
  //            σ_bg² * dt, σ_bg² * dt, σ_bg² * dt)  // gyro bias

  eskf_scalar q_vel = Q_.accel_noise * Q_.accel_noise * dt;
  eskf_scalar q_att = Q_.gyro_noise * Q_.gyro_noise * dt;
  const bool freeze_accel_bias = freezeAccelBiasInFlight();
  const bool freeze_gyro_bias = freezeGyroBiasInFlight();
  eskf_scalar q_ba = freeze_accel_bias ? 0 : Q_.accel_bias_walk * dt;
  eskf_scalar q_bg = freeze_gyro_bias ? 0 : Q_.gyro_bias_walk * dt;
  eskf_scalar q_baro = Q_.baro_bias_walk * dt;

  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_.P[i][j] = FPFt[i][j];
    }
  }

  // Add process noise to diagonal
  // Position noise (indirect through velocity integration)
  // Fix: Use 1/3 factor for piecewise constant acceleration model
  cov_.P[0][0] += q_vel * dt * dt / 3;
  cov_.P[1][1] += q_vel * dt * dt / 3;
  cov_.P[2][2] += q_vel * dt * dt / 3;

  // Velocity noise
  cov_.P[3][3] += q_vel;
  cov_.P[4][4] += q_vel;
  cov_.P[5][5] += q_vel;

  // Attitude noise
  cov_.P[6][6] += q_att;
  cov_.P[7][7] += q_att;
  cov_.P[8][8] += q_att;

  // Accel bias noise
  cov_.P[9][9] += q_ba;
  cov_.P[10][10] += q_ba;
  cov_.P[11][11] += q_ba;

  // Gyro bias noise
  cov_.P[12][12] += q_bg;
  cov_.P[13][13] += q_bg;
  cov_.P[14][14] += q_bg;

  // Baro bias noise
  cov_.P[15][15] += q_baro;
}

// ============================================================
// Utility Functions
// ============================================================

void EskfCore::flushDeferredCovariancePropagation() {
#if ESKF_COVARIANCE_DECIMATION > 1
  if (cov_decimation_counter_ == 0) {
    return;
  }

  // Apply accumulated covariance transport/state transition before any
  // asynchronous update computes gain/gating against P.
  propagateCovariance(cov_decimation_F_acc_, 0);
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_.P[i][j] += cov_decimation_Q_acc_[i][j];
    }
  }

  cov_decimation_counter_ = 0;
  cov_decimation_dt_acc_s_ = 0;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_decimation_F_acc_[i][j] = (i == j) ? 1 : 0;
      cov_decimation_Q_acc_[i][j] = 0;
    }
  }
#endif
}

void EskfCore::checkNumericalHealth() {
  // Check quaternion
  if (!math::quatIsFinite(state_.q)) {
    if (!diverged_) {
      diverged_ = true;
      mode_ = FilterMode::Diverged;
      getEskfLogger().logEvent(EskfEventType::FilterDiverged,
                               state_.timestamp_us, 0.0f);
    }
    return;
  }

  // Check quaternion norm
  eskf_scalar qnorm = math::quatNorm(state_.q);
  if (std::abs(qnorm - 1) > 0.01) {
    // Renormalize
    math::quatNormalize(state_.q);
    getEskfLogger().logEvent(EskfEventType::QuaternionRenormalized,
                             state_.timestamp_us, static_cast<float>(qnorm));
  }

  // Check position/velocity for NaN
  for (int i = 0; i < 3; ++i) {
    if (!math::isFinite(state_.p[i]) || !math::isFinite(state_.v[i])) {
      if (!diverged_) {
        diverged_ = true;
        mode_ = FilterMode::Diverged;
        getEskfLogger().logEvent(EskfEventType::FilterDiverged,
                                 state_.timestamp_us, 1.0f);
      }
      return;
    }
  }

  // Check covariance diagonal for positivity
  for (int i = 0; i < kDimError; ++i) {
    if (!math::isFinite(cov_.P[i][i]) || cov_.P[i][i] < 0) {
      if (!diverged_) {
        diverged_ = true;
        mode_ = FilterMode::Diverged;
        getEskfLogger().logEvent(EskfEventType::FilterDiverged,
                                 state_.timestamp_us, 2.0f);
      }
      return;
    }
  }

  // Check NIS for divergence
  // Track consecutive high NIS values to detect sustained model mismatch.
  // Note: Occasional high NIS is expected during events like baro re-entry
  // after transonic flight, so we only flag divergence after sustained high
  // values.
  // NIS-based divergence is "soft": baro corrections continue with inflated R
  // so that b_baro can slowly adapt through process noise.  Recovery clears
  // the flag after sustained low NIS.
  if (last_nis_ > cfg_.nis_divergence_threshold) {
    consecutive_high_nis_count_++;
    consecutive_low_nis_count_ = 0;
    if (consecutive_high_nis_count_ == 1) {
      // Log first warning
      getEskfLogger().logEvent(EskfEventType::NisDivergenceWarning,
                               state_.timestamp_us,
                               static_cast<float>(last_nis_));
    }
    if (consecutive_high_nis_count_ >= cfg_.nis_max_consecutive_high) {
      if (!nis_soft_diverged_) {
        nis_soft_diverged_ = true;
        // Set diverged_ for status reporting, but correctBaroAltitude
        // checks nis_soft_diverged_ to allow continued soft corrections.
        diverged_ = true;
        mode_ = FilterMode::Diverged;
        getEskfLogger().logEvent(EskfEventType::FilterDiverged,
                                 state_.timestamp_us, 3.0f);
      }
    }
  } else {
    consecutive_high_nis_count_ = 0; // Reset on normal NIS
    // Recovery path: if NIS returns to normal after soft divergence,
    // clear the diverged state so normal baro corrections resume.
    if (nis_soft_diverged_) {
      consecutive_low_nis_count_++;
      if (consecutive_low_nis_count_ >= cfg_.nis_max_consecutive_high) {
        nis_soft_diverged_ = false;
        diverged_ = false;
        mode_ = FilterMode::Flight;
        consecutive_low_nis_count_ = 0;
        getEskfLogger().logEvent(EskfEventType::NisDivergenceWarning,
                                 state_.timestamp_us, -1.0f); // Negative = recovery
      }
    }
  }
}

eskf_scalar EskfCore::gravityMagnitude() const {
  // Use Down position (positive down in NED)
  eskf_scalar altitude = -state_.p[2];
  // Pass g_local_ configuration
  return math::gravityMagnitude(altitude, g_local_);
}

void EskfCore::gravityNed(eskf_scalar g_ned[3]) const {
  // Gravity points down in NED frame
  g_ned[0] = 0;
  g_ned[1] = 0;
  g_ned[2] = gravityMagnitude(); // Positive down
}

Checkpoint EskfCore::createCheckpoint() const {
  Checkpoint cp;
  cp.state = state_;
  cp.cov = cov_;
  cp.timestamp_us = state_.timestamp_us;
  cp.heading_aligned = heading_aligned_;
  cp.heading_initialized = heading_initialized_;
  cp.consecutive_heading_rejects = consecutive_heading_rejects_;
  cp.prev_gyro[0] = prev_gyro_(0);
  cp.prev_gyro[1] = prev_gyro_(1);
  cp.prev_gyro[2] = prev_gyro_(2);
  cp.prev_accel_body[0] = prev_accel_body_(0);
  cp.prev_accel_body[1] = prev_accel_body_(1);
  cp.prev_accel_body[2] = prev_accel_body_(2);
  cp.prev_vel[0] = prev_vel_(0);
  cp.prev_vel[1] = prev_vel_(1);
  cp.prev_vel[2] = prev_vel_(2);
  cp.first_run = first_run_;
  cp.cov_decimation_counter = cov_decimation_counter_;
#if ESKF_COVARIANCE_DECIMATION > 1
  cp.cov_decimation_dt_acc_s = cov_decimation_dt_acc_s_;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cp.cov_decimation_F_acc[i][j] = cov_decimation_F_acc_[i][j];
      cp.cov_decimation_Q_acc[i][j] = cov_decimation_Q_acc_[i][j];
    }
  }
#endif
  cp.gps_ned_offset_n = gps_ned_offset_n_;
  cp.gps_ned_offset_e = gps_ned_offset_e_;
  return cp;
}

void EskfCore::restoreCheckpoint(const Checkpoint &cp) {
  state_ = cp.state;
  cov_ = cp.cov;
  heading_aligned_ = cp.heading_aligned;
  heading_initialized_ = cp.heading_initialized;
  consecutive_heading_rejects_ = cp.consecutive_heading_rejects;
  prev_gyro_(0) = cp.prev_gyro[0];
  prev_gyro_(1) = cp.prev_gyro[1];
  prev_gyro_(2) = cp.prev_gyro[2];
  prev_accel_body_(0) = cp.prev_accel_body[0];
  prev_accel_body_(1) = cp.prev_accel_body[1];
  prev_accel_body_(2) = cp.prev_accel_body[2];
  prev_vel_(0) = cp.prev_vel[0];
  prev_vel_(1) = cp.prev_vel[1];
  prev_vel_(2) = cp.prev_vel[2];
  first_run_ = cp.first_run;
  cov_decimation_counter_ = cp.cov_decimation_counter;
#if ESKF_COVARIANCE_DECIMATION > 1
  cov_decimation_dt_acc_s_ = cp.cov_decimation_dt_acc_s;
  for (int i = 0; i < kDimError; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_decimation_F_acc_[i][j] = cp.cov_decimation_F_acc[i][j];
      cov_decimation_Q_acc_[i][j] = cp.cov_decimation_Q_acc[i][j];
    }
  }
#endif
  gps_ned_offset_n_ = cp.gps_ned_offset_n;
  gps_ned_offset_e_ = cp.gps_ned_offset_e;
  last_nis_ = 0;
  // Don't reset diverged_ flag - let caller decide
}

// ============================================================
// GPS Packet Rejection - Innovation Computation (No State Update)
// ============================================================

eskf_scalar EskfCore::computeGpsVelocityInnovation(
    const eskf_scalar vel_ned[3], const eskf_scalar R[3],
    const eskf_scalar lever_arm_body[3], eskf_scalar innovations_out[3],
    eskf_scalar S_out[3]) {
  flushDeferredCovariancePropagation();

  // Get body rate estimate for lever arm correction (same as in
  // correctGpsVelocity)
  eskf_scalar gyro_unbiased[3];
  if (first_run_) {
    gyro_unbiased[0] = 0;
    gyro_unbiased[1] = 0;
    gyro_unbiased[2] = 0;
  } else {
    gyro_unbiased[0] = prev_gyro_(0);
    gyro_unbiased[1] = prev_gyro_(1);
    gyro_unbiased[2] = prev_gyro_(2);
  }

  // ω × r (lever-arm velocity in body frame)
  math::Vector3 w, r;
  w(0) = gyro_unbiased[0];
  w(1) = gyro_unbiased[1];
  w(2) = gyro_unbiased[2];
  r(0) = lever_arm_body[0];
  r(1) = lever_arm_body[1];
  r(2) = lever_arm_body[2];
  math::Vector3 v_arm_body = w.cross(r);

  // Rotate lever-arm velocity to NED
  eskf_scalar v_arm_body_arr[3] = {v_arm_body(0), v_arm_body(1), v_arm_body(2)};
  eskf_scalar v_arm_ned[3];
  math::quatRotateVector(v_arm_ned, state_.q, v_arm_body_arr);

  // Corrected measurement (virtual measurement at CG)
  eskf_scalar vel_cg_meas[3];
  for (int i = 0; i < 3; ++i) {
    vel_cg_meas[i] = vel_ned[i] - v_arm_ned[i];
  }

  // Build Jacobian terms to mirror correctGpsVelocity() model:
  // H_vel = I
  // H_att = -[v_arm_ned]x
  // H_bg  = R_bn * [r]x
  eskf_scalar v_arm_skew[3][3];
  math::skewSymmetric(v_arm_skew, v_arm_ned);
  eskf_scalar r_skew[3][3];
  math::skewSymmetric(r_skew, lever_arm_body);
  eskf_scalar R_bn[3][3];
  math::quatToDcm(R_bn, state_.q);
  eskf_scalar R_r_skew[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      R_r_skew[i][j] = 0;
      for (int k = 0; k < 3; ++k) {
        R_r_skew[i][j] += R_bn[i][k] * r_skew[k][j];
      }
    }
  }

  const bool decouple_lever_arm_attitude =
      cfg_.disable_gps_vel_lever_arm_attitude_jacobian;
  const bool freeze_gyro_bias = freezeGyroBiasInFlight();

  eskf_scalar total_chi2 = 0;

  for (int axis = 0; axis < 3; ++axis) {
    // Innovation: z - h
    innovations_out[axis] = vel_cg_meas[axis] - state_.v[axis];

    // Full scalar innovation variance S = H * P * H' + R for this axis
    eskf_scalar H[kDimError] = {};
    H[idx::kVel + axis] = 1;
    H[idx::kAtt + 0] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][0];
    H[idx::kAtt + 1] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][1];
    H[idx::kAtt + 2] = decouple_lever_arm_attitude ? 0 : -v_arm_skew[axis][2];
    H[idx::kGyrBias + 0] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][0];
    H[idx::kGyrBias + 1] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][1];
    H[idx::kGyrBias + 2] =
      (decouple_lever_arm_attitude || freeze_gyro_bias) ? 0
                                : R_r_skew[axis][2];

    eskf_scalar HP[kDimError];
    for (int j = 0; j < kDimError; ++j) {
      eskf_scalar sum = 0;
      for (int i = 0; i < kDimError; ++i) {
        sum += H[i] * cov_.P[i][j];
      }
      HP[j] = sum;
    }

    eskf_scalar hph = 0;
    for (int j = 0; j < kDimError; ++j) {
      hph += HP[j] * H[j];
    }

    S_out[axis] = hph + R[axis];

    if (S_out[axis] > 1e-10) {
      total_chi2 +=
          (innovations_out[axis] * innovations_out[axis]) / S_out[axis];
    }
  }

  return total_chi2;
}

eskf_scalar EskfCore::computeGpsPositionInnovation(
    const eskf_scalar pos_ned[3], const eskf_scalar R[3],
    eskf_scalar innovations_out[3], eskf_scalar S_out[3]) {
  flushDeferredCovariancePropagation();

  eskf_scalar total_chi2 = 0;

  for (int axis = 0; axis < 3; ++axis) {
    // Innovation: z - h
    innovations_out[axis] = pos_ned[axis] - state_.p[axis];

    // Innovation variance: S = H * P * H' + R
    // For position, H has 1 at pos[axis] position
    S_out[axis] = cov_.P[idx::kPos + axis][idx::kPos + axis] + R[axis];

    // Contribution to Chi-Squared
    if (S_out[axis] > 1e-10) {
      total_chi2 +=
          (innovations_out[axis] * innovations_out[axis]) / S_out[axis];
    }
  }

  return total_chi2;
}

void EskfCore::setGpsVelocityGateDebug(eskf_scalar chi2, eskf_scalar threshold,
                                       bool accepted) {
  last_gps_vel_chi2_ = chi2;
  last_gps_vel_chi2_threshold_ = threshold;
  last_gps_vel_accepted_ = accepted;
}

void EskfCore::setMode(FilterMode m) {
  const bool changed = (m != mode_);
  if (changed) {
    // Log mode change before updating
    getEskfLogger().logEvent(EskfEventType::ModeChanged, state_.timestamp_us,
                             static_cast<float>(static_cast<uint8_t>(m)));
  }
  mode_ = m;

  if (changed && m == FilterMode::Flight) {
    freezeFlightBiasCovariance();
  }
}

void EskfCore::inflatePositionCovariance(eskf_scalar variance) {
  flushDeferredCovariancePropagation();
  if (!std::isfinite(variance) || variance <= 0) {
    return;
  }

  // True inflation floor: only raise uncertainty, never shrink it.
  for (int i = 0; i < 3; ++i) {
    const int idx_pos = idx::kPos + i;
    if (cov_.P[idx_pos][idx_pos] < variance) {
      cov_.P[idx_pos][idx_pos] = variance;
    }
  }
}

void EskfCore::inflateVelocityCovariance(eskf_scalar variance) {
  flushDeferredCovariancePropagation();
  if (!std::isfinite(variance) || variance <= 0) {
    return;
  }

  // True inflation floor: only raise uncertainty, never shrink it.
  for (int i = 0; i < 3; ++i) {
    const int idx_vel = idx::kVel + i;
    if (cov_.P[idx_vel][idx_vel] < variance) {
      cov_.P[idx_vel][idx_vel] = variance;
    }
  }
}

void EskfCore::inflateBaroBiasCovariance(eskf_scalar variance) {
  flushDeferredCovariancePropagation();
  if (!std::isfinite(variance) || variance <= 0) {
    return;
  }

  if (cov_.P[idx::kBarBias][idx::kBarBias] < variance) {
    cov_.P[idx::kBarBias][idx::kBarBias] = variance;
  }
}

void EskfCore::resetPositionCovariance(eskf_scalar variance) {
  flushDeferredCovariancePropagation();
  if (!std::isfinite(variance) || variance < 0) {
    return;
  }

  // Hard-reset block for state teleports (position overwrite).
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_.P[idx::kPos + i][j] = 0;
      cov_.P[j][idx::kPos + i] = 0;
    }
  }
  for (int i = 0; i < 3; ++i) {
    cov_.P[idx::kPos + i][idx::kPos + i] = variance;
  }
}

void EskfCore::resetVelocityCovariance(eskf_scalar variance) {
  flushDeferredCovariancePropagation();
  if (!std::isfinite(variance) || variance < 0) {
    return;
  }

  // Hard-reset block for state overwrites (velocity overwrite).
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < kDimError; ++j) {
      cov_.P[idx::kVel + i][j] = 0;
      cov_.P[j][idx::kVel + i] = 0;
    }
  }
  for (int i = 0; i < 3; ++i) {
    cov_.P[idx::kVel + i][idx::kVel + i] = variance;
  }
}

void EskfCore::decoupleAttitudeFromVelocity() {
  flushDeferredCovariancePropagation();
  // Zero cross-terms: P[att, vel/pos], P[vel/pos, att], P[gyrbias, vel/pos], P[vel/pos, gyrbias]
  // Symmetry: P[i][j] = P[j][i]
  constexpr int att_indices[] = {idx::kAtt, idx::kAtt+1, idx::kAtt+2,
                                  idx::kGyrBias, idx::kGyrBias+1, idx::kGyrBias+2};
  constexpr int vp_indices[] = {idx::kPos, idx::kPos+1, idx::kPos+2,
                                 idx::kVel, idx::kVel+1, idx::kVel+2};
  for (int a : att_indices) {
    for (int v : vp_indices) {
      cov_.P[a][v] = 0;
      cov_.P[v][a] = 0;
    }
  }
}

// ============================================================
// Logging Snapshot Creation
// ============================================================

StateSnapshot EskfCore::createStateSnapshot() const {
  StateSnapshot snap;

  // Convert to float for storage efficiency
  for (int i = 0; i < 3; ++i) {
    snap.p[i] = static_cast<float>(state_.p[i]);
    snap.v[i] = static_cast<float>(state_.v[i]);
    snap.b_acc[i] = static_cast<float>(state_.b_acc[i]);
    snap.b_gyro[i] = static_cast<float>(state_.b_gyro[i]);
  }
  for (int i = 0; i < 4; ++i) {
    snap.q[i] = static_cast<float>(state_.q[i]);
  }
  snap.b_baro = static_cast<float>(state_.b_baro);
  // Yaw extracted from current quaternion in NED convention.
  const eskf_scalar qw = state_.q[0], qx = state_.q[1];
  const eskf_scalar qy = state_.q[2], qz = state_.q[3];
  snap.yaw_rad =
      static_cast<float>(std::atan2(2 * (qw * qz + qx * qy),
                                    1 - 2 * (qy * qy + qz * qz)));
  snap.heading_meas_rad = static_cast<float>(last_heading_meas_rad_);
  snap.heading_innovation_rad = static_cast<float>(last_heading_innovation_rad_);
  snap.heading_gate_threshold = static_cast<float>(last_heading_gate_threshold_);
  snap.gps_vel_chi2 = static_cast<float>(last_gps_vel_chi2_);
  snap.gps_vel_chi2_threshold = static_cast<float>(last_gps_vel_chi2_threshold_);
  snap.timestamp_us = state_.timestamp_us;
  snap.mode = static_cast<uint8_t>(mode_);

  // Pack flags: bit0 = heading_aligned, bit1 = heading_initialized
  snap.flags =
      (heading_aligned_ ? 0x01 : 0x00) | (heading_initialized_ ? 0x02 : 0x00);
  snap.heading_update_result = static_cast<uint8_t>(last_heading_update_result_);
  snap.gps_vel_accepted = last_gps_vel_accepted_ ? 1 : 0;

  return snap;
}

CovarianceSnapshot EskfCore::createCovarianceSnapshot() const {
  CovarianceSnapshot snap;

  for (int i = 0; i < kDimError; ++i) {
    snap.P_diag[i] = static_cast<float>(cov_.P[i][i]);
  }
  snap.timestamp_us = state_.timestamp_us;

  return snap;
}

} // namespace eskf
