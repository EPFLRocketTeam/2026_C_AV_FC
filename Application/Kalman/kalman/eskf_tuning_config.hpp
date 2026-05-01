#pragma once
// ESKF Tuning Configuration - Runtime-injectable algorithm parameters
// Part of Phase 1: Standalone Core ESKF Library
//
// This struct contains all algorithm tuning parameters that may vary
// between flights or need adjustment during replay. Buffer sizes and
// other structural constants remain in eskf_sizes.hpp.

#include <cstdint>
#include <cmath>
#include "Application/Kalman/kalman/eskf_tuning_defaults.hpp"

namespace eskf {

enum class GpsHeadingBootstrapMode : uint8_t {
  CogDirect = 0,
  VelocityAngleDelta = 1,
};

/// Runtime-configurable tuning parameters for ESKF algorithm.
/// All members have sensible defaults matching original macro values.
struct TuningConfig {
  // ============================================================
  // Feature Flags
  // ============================================================
  bool use_gps_rewind = ESKF_USE_GPS_REWIND;
  bool use_central_diff = ESKF_USE_CENTRAL_DIFF;
  bool enable_sideslip = false;
  bool use_coning_compensation = ESKF_USE_CONING_COMPENSATION;
  bool use_lever_arm_averaging = ESKF_USE_LEVER_ARM_AVERAGING;
  bool enable_gps_cog_heading = ESKF_ENABLE_GPS_COG_HEADING;
  GpsHeadingBootstrapMode gps_heading_bootstrap_mode =
      static_cast<GpsHeadingBootstrapMode>(ESKF_GPS_HEADING_BOOTSTRAP_MODE);
  bool enable_gps_cog_heading_fusion = ESKF_ENABLE_GPS_COG_HEADING_FUSION;
  bool disable_gps_after_alignment = ESKF_DISABLE_GPS_AFTER_ALIGNMENT;
  bool disable_gps_vel_lever_arm_attitude_jacobian =
      ESKF_DISABLE_GPS_VEL_LEVER_ARM_ATTITUDE_JACOBIAN;
  bool freeze_accel_bias_in_flight = ESKF_FREEZE_ACCEL_BIAS_IN_FLIGHT;
  bool freeze_gyro_bias_in_flight = ESKF_FREEZE_GYRO_BIAS_IN_FLIGHT;
  uint8_t covariance_decimation = ESKF_COVARIANCE_DECIMATION;  // Update P every Nth step (1=disabled)
  float predict_max_dt_s = ESKF_PREDICT_MAX_DT_S;              // Clamp per-step predict dt

    // ============================================================
    // Multi-IMU Hardening Policy
    // ============================================================
    float imu_gyro_voting_threshold = ESKF_IMU_GYRO_VOTING_THRESHOLD;
    float imu_accel_voting_threshold = ESKF_IMU_ACCEL_VOTING_THRESHOLD;
    float imu_gyro_hard_fault_threshold = ESKF_IMU_GYRO_HARD_FAULT_THRESHOLD;
    float imu_accel_hard_fault_threshold = ESKF_IMU_ACCEL_HARD_FAULT_THRESHOLD;
    uint16_t imu_hard_fault_suspect_samples = ESKF_IMU_HARD_FAULT_SUSPECT_SAMPLES;
    uint16_t imu_hard_fault_persistence_samples =
      ESKF_IMU_HARD_FAULT_PERSISTENCE_SAMPLES;
    uint16_t imu_recovery_cooldown_samples = ESKF_IMU_RECOVERY_COOLDOWN_SAMPLES;
    uint16_t imu_recovery_confirm_samples = ESKF_IMU_RECOVERY_CONFIRM_SAMPLES;
    float imu_accel_saturation_threshold = ESKF_IMU_ACCEL_SATURATION_THRESHOLD;
    float imu_gyro_saturation_threshold = ESKF_IMU_GYRO_SATURATION_THRESHOLD;
    uint16_t imu_saturation_hard_fault_persistence_samples =
      ESKF_IMU_SATURATION_HARD_FAULT_PERSISTENCE_SAMPLES;
    uint8_t imu_saturation_multi_axis_limit = ESKF_IMU_SATURATION_MULTI_AXIS_LIMIT;
    bool imu_enable_all_soft_reject_salvage =
      ESKF_IMU_ENABLE_ALL_SOFT_REJECT_SALVAGE;
    uint16_t imu_stale_persistence_samples =
      ESKF_IMU_STALE_PERSISTENCE_SAMPLES;
    float imu_stale_accel_delta_threshold =
      ESKF_IMU_STALE_ACCEL_DELTA_THRESHOLD;
    float imu_stale_gyro_delta_threshold =
      ESKF_IMU_STALE_GYRO_DELTA_THRESHOLD;
    bool imu_enable_preflight_tare = ESKF_IMU_ENABLE_PREFLIGHT_TARE;
    uint16_t imu_tare_window_samples = ESKF_IMU_TARE_WINDOW_SAMPLES;
    float imu_tare_accel_gravity = ESKF_IMU_TARE_ACCEL_GRAVITY;
    float imu_boost_soft_threshold_scale = ESKF_IMU_BOOST_SOFT_THRESHOLD_SCALE;
    uint32_t imu_boost_soft_window_us = ESKF_IMU_BOOST_SOFT_WINDOW_US;

    // ============================================================
    // Multi-Baro Hardening Policy
    // ============================================================
    float baro_voting_threshold_pa = ESKF_BARO_VOTING_THRESHOLD_PA;
    float baro_hard_fault_threshold_pa = ESKF_BARO_HARD_FAULT_THRESHOLD_PA;
    float baro_calibration_mismatch_tolerance_pa =
      ESKF_BARO_CALIBRATION_MISMATCH_TOLERANCE_PA;
    uint16_t baro_hard_fault_suspect_samples =
      ESKF_BARO_HARD_FAULT_SUSPECT_SAMPLES;
    uint16_t baro_hard_fault_persistence_samples =
      ESKF_BARO_HARD_FAULT_PERSISTENCE_SAMPLES;
    uint16_t baro_recovery_cooldown_samples = ESKF_BARO_RECOVERY_COOLDOWN_SAMPLES;
    uint16_t baro_recovery_confirm_samples = ESKF_BARO_RECOVERY_CONFIRM_SAMPLES;
    float baro_continuity_max_step_pa = ESKF_BARO_CONTINUITY_MAX_STEP_PA;
    bool baro_enable_all_soft_reject_salvage =
      ESKF_BARO_ENABLE_ALL_SOFT_REJECT_SALVAGE;
    uint16_t baro_stale_persistence_samples =
      ESKF_BARO_STALE_PERSISTENCE_SAMPLES;
    float baro_stale_pressure_delta_threshold_pa =
      ESKF_BARO_STALE_PRESSURE_DELTA_THRESHOLD_PA;
    float baro_stale_temperature_delta_threshold_k =
      ESKF_BARO_STALE_TEMPERATURE_DELTA_THRESHOLD_K;

  // ============================================================
  // Dynamic Baro Variance (Section 6.4.B)
  // ============================================================
  // R_baro = (σ_base + K_aero·v²)² with transonic penalty
  float baro_sigma_base = ESKF_BARO_SIGMA_BASE;            // Base noise floor (m)
  float baro_k_aero = ESKF_BARO_K_AERO;              // Velocity-squared coefficient
  float baro_transonic_low = ESKF_BARO_TRANSONIC_LOW;       // ~Mach 0.7 at sea level (m/s)
  float baro_transonic_high = ESKF_BARO_TRANSONIC_HIGH;      // ~Mach 1.2 at sea level (m/s)
  float baro_transonic_penalty = ESKF_BARO_TRANSONIC_PENALTY; // Massive R inflation in transonic
  float baro_innovation_clamp = ESKF_BARO_INNOVATION_CLAMP;     // Max allowed innovation (m)

  // ============================================================
  // GPS Configuration (Section 6.2, 6.4.B.2)
  // ============================================================
  int32_t gps_delay_us = ESKF_DEFAULT_GPS_DELAY_US;          // GPS delay from PPS (µs, negative = earlier)
  float gps_trust_factor = ESKF_GPS_TRUST_FACTOR;           // Accuracy inflation factor (K_trust)
  float gps_high_g_threshold = ESKF_GPS_HIGH_G_THRESHOLD;       // High-G threshold (g-units)
  float gps_high_g_r_factor = ESKF_GPS_HIGH_G_R_FACTOR;      // R multiplier when above threshold

  // Chi-Squared Gating
  float gps_vel_chi2_threshold = ESKF_GPS_VEL_CHI2_THRESHOLD;    // 3 DOF, ~1% false rejection
  float gps_pos_chi2_threshold = ESKF_GPS_POS_CHI2_THRESHOLD;    // 3 DOF
  // Soft-accept multipliers: allow marginal chi2 packets to be accepted with
  // adaptive R inflation instead of hard rejecting them. These are unitless
  // multipliers applied to the configured chi2 thresholds.
  float gps_vel_soft_accept_multiplier = 20.0f;
  float gps_pos_soft_accept_multiplier = 20.0f;
  uint16_t gps_max_consecutive_rejects = ESKF_GPS_MAX_CONSECUTIVE_REJECTS; // ~5 seconds at 16Hz

  // Covariance inflation upon recovery
  float gps_reset_p_pos = ESKF_GPS_RESET_P_POS;        // m² - "unknown position"
  float gps_reset_p_vel = ESKF_GPS_RESET_P_VEL;          // (m/s)² - "unknown velocity"
  float gps_heading_variance = ESKF_GPS_HEADING_VARIANCE;
  float gps_vel_tumble_gyro_threshold = ESKF_GPS_VEL_TUMBLE_GYRO_THRESHOLD;
  float gps_vel_tumble_r_factor = ESKF_GPS_VEL_TUMBLE_R_FACTOR;

  // ============================================================
  // Sideslip Constraint (Aerodynamic velocity constraint)
  // ============================================================
  float sideslip_r_lateral = 1.0f;           // Measurement noise variance (m/s)²
  float sideslip_min_speed = 15.0f;          // Min NED speed to apply constraint (m/s)
  float sideslip_max_gyro = 0.5f;            // Max angular rate for sideslip validity (rad/s)
  uint16_t sideslip_decimation = 640;        // Apply every N IMU steps (~10Hz at 6.4kHz)

  // ============================================================
  // Heading Alignment (Section 5.4)
  // ============================================================
  float heading_align_min_speed = ESKF_HEADING_ALIGN_MIN_SPEED;   // Min speed for alignment (m/s)
  float heading_align_max_sacc = ESKF_HEADING_ALIGN_MAX_SACC;     // Max GPS speed accuracy (m/s)
  float heading_align_max_gyro = ESKF_HEADING_ALIGN_MAX_GYRO;    // Max body rate magnitude (rad/s, ~20°/s)

  // Heading State Machine
  float launch_rail_heading = ESKF_LAUNCH_RAIL_HEADING;        // Fallback heading (rad from True North)
  bool trust_hardcoded_heading = ESKF_TRUST_HARDCODED_HEADING;    // Use hardcoded heading instead of sensors
  float expected_dip_angle_rad = ESKF_EXPECTED_DIP_ANGLE_RAD;     // Magnetic dip at launch site (rad)
  float dip_angle_threshold_rad = ESKF_DIP_ANGLE_THRESHOLD_RAD;  // Dip validation threshold (~10°)
  uint32_t rail_clear_delay_us = ESKF_RAIL_CLEAR_DELAY_US;   // Pause mag fusion after liftoff (500ms)
  uint16_t heading_resurrect_count = ESKF_HEADING_RESURRECT_COUNT;   // Consecutive rejects before resurrection
  float post_align_yaw_var = ESKF_POST_ALIGN_YAW_VAR;      // Post-alignment yaw variance (rad²)

  // NIS Divergence Detection
  float nis_divergence_threshold = ESKF_NIS_DIVERGENCE_THRESHOLD;
  uint16_t nis_max_consecutive_high = ESKF_NIS_MAX_CONSECUTIVE_HIGH;

  // ============================================================
  // Shadow Filter - Rail (Gated Mahony)
  // ============================================================
  float shadow_rail_kp = ESKF_SHADOW_RAIL_KP;             // Proportional gain
  float shadow_rail_ki = ESKF_SHADOW_RAIL_KI;             // Integral gain
  float shadow_rail_gate = ESKF_SHADOW_RAIL_GATE;           // Accel feedback gate threshold (m/s²)

  // ============================================================
  // Shadow Filter - Flight (Linear State Observer)
  // ============================================================
  float shadow_flight_omega_n = ESKF_SHADOW_FLIGHT_OMEGA_N;     // Natural frequency (rad/s)
  float shadow_flight_zeta = ESKF_SHADOW_FLIGHT_ZETA;       // Damping ratio
  float shadow_aero_blind_speed = ESKF_SHADOW_AERO_BLIND_SPEED;  // Disable baro correction above (m/s)
  float shadow_aero_blind_enter_speed = ESKF_SHADOW_AERO_BLIND_ENTER_SPEED;
  float shadow_aero_blind_exit_speed = ESKF_SHADOW_AERO_BLIND_EXIT_SPEED;
  uint16_t shadow_aero_blind_enter_debounce_ms =
      ESKF_SHADOW_AERO_BLIND_ENTER_DEBOUNCE_MS;
  uint16_t shadow_aero_blind_exit_debounce_ms =
      ESKF_SHADOW_AERO_BLIND_EXIT_DEBOUNCE_MS;
  float shadow_apogee_hysteresis = ESKF_SHADOW_APOGEE_HYSTERESIS;   // Apogee detection hysteresis (m/s)

  // ============================================================
  // Pre-Liftoff Estimation
  // ============================================================
  float gyro_bias_lpf_alpha = ESKF_GYRO_BIAS_LPF_ALPHA;     // Gyro bias low-pass filter alpha
  uint32_t liftoff_rewind_us = ESKF_LIFTOFF_REWIND_US;     // Rewind depth before liftoff (400ms)
  uint32_t liftoff_rejection_us = ESKF_LIFTOFF_REJECTION_US;  // Reject aiding sensors for (500ms)
  uint32_t baro_snapshot_max_age_us = ESKF_BARO_SNAPSHOT_MAX_AGE_US;
  uint32_t rail_checkpoint_interval_us = ESKF_RAIL_CHECKPOINT_INTERVAL_US; // Checkpoint interval (10ms = 100Hz)

  // ============================================================
  // Physics / Launch Site
  // ============================================================
  float launch_latitude_deg = ESKF_LAUNCH_SITE_LATITUDE_DEG;       // Launch site latitude (degrees)
  float launch_longitude_deg = 8.0f;       // Launch site longitude (degrees) - for future WMM
  float local_gravity = 9.80665f;          // Local gravity (m/s²), computed from latitude

  // ============================================================
  // Magnetometer Site-Specific Parameters
  // ============================================================
  // These values depend on the launch site and should be looked up from NOAA:
  // https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
  float mag_declination_rad = ESKF_MAG_DECLINATION_RAD;         // Magnetic declination (+East, rad)
  float mag_expected_magnitude_ut = 50.0f;  // Expected field magnitude (μT)
  float mag_magnitude_threshold_ut = 10.0f; // Magnitude rejection threshold (μT)
  // Note: expected_dip_angle_rad is computed from launch_latitude_deg
  // Note: Dip angle validation is not currently wired - it requires attitude
  // quaternion to rotate mag vector to NED before checking, which VirtualCompass
  // doesn't have access to. If needed, implement in eskf_estimator.cpp.

  /// Compute local gravity from latitude using Somigliana formula
  void computeLocalGravity() {
    constexpr float kGravityEquator = 9.7803253359f;
    constexpr float kPi = 3.14159265358979323846f;
    const float phi = launch_latitude_deg * kPi / 180.0f;
    const float sin_phi = std::sin(phi);
    const float sin_2phi = std::sin(2.0f * phi);
    const float sin2_phi = sin_phi * sin_phi;
    const float sin2_2phi = sin_2phi * sin_2phi;
    local_gravity = kGravityEquator * (1.0f + 0.0053024f * sin2_phi - 0.0000058f * sin2_2phi);
  }

  /// Compute expected dip angle from latitude (dipole approximation)
  void computeExpectedDipAngle() {
    constexpr float kPi = 3.14159265358979323846f;
    const float phi_rad = launch_latitude_deg * kPi / 180.0f;
    expected_dip_angle_rad = std::atan(2.0f * std::tan(phi_rad));
  }
};

/// Get default tuning configuration with computed physics values.
inline TuningConfig getDefaultTuningConfig() {
  TuningConfig cfg;
  cfg.computeLocalGravity();
  cfg.computeExpectedDipAngle();
  return cfg;
}

} // namespace eskf
