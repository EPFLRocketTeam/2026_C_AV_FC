#pragma once
// ESKF Core Data Structures
// Part of Phase 1: Standalone Core ESKF Library

#include "eskf_config.hpp"
#include <stdint.h>

namespace eskf {

// ============================================================
// Nominal State Vector (16 dimensions)
// ============================================================

/// Nominal state used for integration and output.
/// Quaternion uses Hamilton convention: [w, x, y, z] with w as scalar.
struct State {
  eskf_scalar p[3];      ///< Position in NED frame (m)
  eskf_scalar v[3];      ///< Velocity in NED frame (m/s)
  eskf_scalar q[4];      ///< Quaternion [w, x, y, z] body→NED (Hamilton)
  eskf_scalar b_acc[3];  ///< Accelerometer bias (m/s²)
  eskf_scalar b_gyro[3]; ///< Gyroscope bias (rad/s)
  eskf_scalar b_baro;    ///< Barometer bias (m)
  uint64_t timestamp_us; ///< Timestamp of this state (µs)

  /// Initialize to identity state (zero position/velocity, identity quaternion)
  void setIdentity() {
    for (int i = 0; i < 3; ++i) {
      p[i] = 0;
      v[i] = 0;
      b_acc[i] = 0;
      b_gyro[i] = 0;
    }
    q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0; // Identity quaternion
    b_baro = 0;
    timestamp_us = 0;
  }
};

// ============================================================
// Error-State Covariance (16×16)
// ============================================================

/// Error-state covariance matrix P.
/// Order: [δp(3), δv(3), δθ(3), δb_a(3), δb_g(3), δb_baro(1)]
/// The 16D error state includes baro bias estimation.
struct Covariance {
  eskf_scalar P[kDimError][kDimError];

  /// Initialize to identity matrix scaled by given variance
  void setIdentity(eskf_scalar variance = 1.0) {
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        P[i][j] = (i == j) ? variance : 0;
      }
    }
  }

  /// Set diagonal elements from array
  void setDiagonal(const eskf_scalar diag[kDimError]) {
    for (int i = 0; i < kDimError; ++i) {
      for (int j = 0; j < kDimError; ++j) {
        P[i][j] = (i == j) ? diag[i] : 0;
      }
    }
  }

  /// Get diagonal element
  eskf_scalar diag(int i) const {
    return (i >= 0 && i < kDimError) ? P[i][i] : 0;
  }
};

// ============================================================
// Checkpoint for GPS Rewind
// ============================================================

/// Complete filter state snapshot for time-machine replay.
struct Checkpoint {
  State state;
  Covariance cov;
  uint64_t timestamp_us;
  // Replay correctness: heading state machine flags must track the
  // checkpointed state to avoid mismatch after rewind restores.
  bool heading_aligned = false;
  bool heading_initialized = false;
  int32_t consecutive_heading_rejects = 0;
  // Integration internals required for deterministic rewind replay.
  eskf_scalar prev_gyro[3] = {0, 0, 0};
  eskf_scalar prev_accel_body[3] = {0, 0, 0};
  eskf_scalar prev_vel[3] = {0, 0, 0};
  bool first_run = true;
  uint32_t cov_decimation_counter = 0;
#if ESKF_COVARIANCE_DECIMATION > 1
  eskf_scalar cov_decimation_dt_acc_s = 0;
  eskf_scalar cov_decimation_F_acc[kDimError][kDimError] = {{0}};
  eskf_scalar cov_decimation_Q_acc[kDimError][kDimError] = {{0}};
#endif
  // Cumulative position offset applied by forceYaw heading snaps.
  // GPS NED values must be shifted by this amount to stay frame-consistent
  // with the ESKF position after heading alignment.
  eskf_scalar gps_ned_offset_n = 0;
  eskf_scalar gps_ned_offset_e = 0;
};

// ============================================================
// Sensor Observations
// ============================================================

/// IMU frame after preprocessing (CG-corrected, body frame).
struct ImuFrame {
  eskf_scalar accel[3];  ///< Acceleration in body frame (m/s²)
  eskf_scalar gyro[3];   ///< Angular velocity in body frame (rad/s)
  uint64_t timestamp_us; ///< Measurement timestamp (µs)
};

/// GPS observation (already converted to NED frame).
struct GpsObservation {
  eskf_scalar pos_ned[3];  ///< Position in NED frame (m)
  eskf_scalar vel_ned[3];  ///< Velocity in NED frame (m/s)
  eskf_scalar pos_var[3];  ///< Position variance diagonal (m²)
  eskf_scalar vel_var[3];  ///< Velocity variance diagonal ((m/s)²)
  uint64_t timestamp_us;   ///< Measurement timestamp (may be in past!)
  bool pos_valid;          ///< Position data is valid
  bool vel_valid;          ///< Velocity data is valid
};

/// Barometer observation.
struct BaroObservation {
  eskf_scalar altitude_m;  ///< Measured altitude (m, positive up)
  eskf_scalar variance;    ///< Measurement variance (m²)
  uint64_t timestamp_us;   ///< Measurement timestamp (µs)
};

/// Magnetometer observation (body frame).
struct MagObservation {
  eskf_scalar field_body[3]; ///< Magnetic field in body frame (µT)
  eskf_scalar variance;      ///< Measurement variance (µT²)
  uint64_t timestamp_us;     ///< Measurement timestamp (µs)
};

// ============================================================
// Process and Measurement Noise Parameters
// ============================================================

/// Process noise configuration (continuous-time spectral densities).
/// These get discretized during prediction: Q_d ≈ Q_c * dt
struct ProcessNoise {
  eskf_scalar accel_noise;      ///< Accel noise (m/s²/√Hz)
  eskf_scalar gyro_noise;       ///< Gyro noise (rad/s/√Hz)
  eskf_scalar accel_bias_walk;  ///< Accel bias random walk ((m/s²)²/s)
  eskf_scalar gyro_bias_walk;   ///< Gyro bias random walk ((rad/s)²/s)
  eskf_scalar baro_bias_walk;   ///< Baro bias random walk (m²/s)

  /// Default noise values for consumer-grade MEMS IMU
  static ProcessNoise defaults() {
    ProcessNoise q{};
    q.accel_noise = 0.1;        // 0.1 m/s²/√Hz
    q.gyro_noise = 0.01;        // 0.01 rad/s/√Hz (~0.57 deg/s/√Hz)
    q.accel_bias_walk = 1e-5;   // (m/s²)²/s
    q.gyro_bias_walk = 1e-7;    // (rad/s)²/s
    q.baro_bias_walk = 1e-4;    // m²/s
    return q;
  }
};

/// Initial uncertainty configuration (standard deviations).
/// P₀ diagonal is set to σ² for each group.
struct InitialCovariance {
  eskf_scalar pos;         ///< Position σ (m)
  eskf_scalar vel;         ///< Velocity σ (m/s)
  eskf_scalar tilt;        ///< Roll/Pitch σ (rad)
  eskf_scalar heading;     ///< Yaw σ (rad)
  eskf_scalar accel_bias;  ///< Accel bias σ (m/s²)
  eskf_scalar gyro_bias;   ///< Gyro bias σ (rad/s)
  eskf_scalar baro_bias;   ///< Baro bias σ (m)

  /// Default initial uncertainties (conservative)
  static InitialCovariance defaults() {
    InitialCovariance p0{};
    p0.pos = 3.0;           // 3 m
    p0.vel = 0.1;           // 0.1 m/s
    p0.tilt = 0.01;         // ~0.5 deg
    p0.heading = 0.5;       // ~30 deg
    p0.accel_bias = 0.1;    // 0.1 m/s²
    p0.gyro_bias = 0.01;    // 0.01 rad/s (~0.5 deg/s)
    p0.baro_bias = 1.0;     // 1 m
    return p0;
  }

  /// Build diagonal P₀ array from this configuration
  void toDiagonal(eskf_scalar diag[kDimError]) const {
    // Position (0-2)
    diag[0] = pos * pos;
    diag[1] = pos * pos;
    diag[2] = pos * pos;
    // Velocity (3-5)
    diag[3] = vel * vel;
    diag[4] = vel * vel;
    diag[5] = vel * vel;
    // Attitude: roll/pitch (6-7), yaw (8)
    diag[6] = tilt * tilt;
    diag[7] = tilt * tilt;
    diag[8] = heading * heading;
    // Accel bias (9-11)
    diag[9] = accel_bias * accel_bias;
    diag[10] = accel_bias * accel_bias;
    diag[11] = accel_bias * accel_bias;
    // Gyro bias (12-14)
    diag[12] = gyro_bias * gyro_bias;
    diag[13] = gyro_bias * gyro_bias;
    diag[14] = gyro_bias * gyro_bias;
    // Baro bias (15)
    diag[15] = baro_bias * baro_bias;
  }
};

// ============================================================
// Filter Status
// ============================================================

/// Filter operational mode
enum class FilterMode : uint8_t {
  Settling = 0,  ///< Pre-flight calibration/settling
  Flight = 1,    ///< Active flight estimation
  Diverged = 2   ///< Filter detected divergence (NaN/NIS exceeded)
};

/// Correction flags for logging
struct CorrectionFlags {
  bool gps_pos : 1;
  bool gps_vel : 1;
  bool baro : 1;
  bool mag : 1;
  bool sideslip : 1;
  uint8_t reserved : 3;

  CorrectionFlags() : gps_pos(false), gps_vel(false), baro(false),
                      mag(false), sideslip(false), reserved(0) {}
};

/// Result of a heading update attempt
enum class HeadingUpdateResult : uint8_t {
  Ignored = 0,     ///< Not fused (e.g. diverged, invalid)
  Fused = 1,       ///< Successfully fused via EKF update
  Snapped = 2,     ///< Hard snap (initialization)
  Rejected = 3,    ///< Rejected by gating
  Resurrected = 4  ///< Hard snap due to consecutive rejections
};

} // namespace eskf
