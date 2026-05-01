#pragma once
// ReplayableConfig - Algorithm-relevant parameters for SD logging and native replay
//
// This struct aggregates compile-time configuration that affects algorithm behavior,
// allowing native_replay to reproduce flight conditions exactly as they were logged.

#include <cstdint>

// Include config sources
#include "Application/Kalman/kalman/eskf_config.hpp"
#include "Application/Kalman/kalman/eskf_types.hpp"
#include "Application/Kalman/AppLayer/hw_config.hpp"
#include "Application/Kalman/AppLayer/eskf_app_config.hpp"

namespace appcfg {

/// Algorithm-relevant configuration for replay reproducibility.
/// This struct is serialized to the SD log header at startup.
struct ReplayableConfig {
  // ============================================================
  // IMU Configuration
  // ============================================================
  uint16_t imu_odr_hz;           // Primary IMU output data rate
  uint8_t  imu_accel_fsr_g;      // Accelerometer full-scale range (g)
  uint16_t imu_gyro_fsr_dps;     // Gyroscope full-scale range (deg/s)
  uint8_t  imu_count;            // Number of active IMUs (1 or 2)

  // Multi-IMU hardening policy
  float imu_gyro_voting_threshold;
  float imu_accel_voting_threshold;
  float imu_gyro_hard_fault_threshold;
  float imu_accel_hard_fault_threshold;
  uint16_t imu_hard_fault_suspect_samples;
  uint16_t imu_hard_fault_persistence_samples;
  uint16_t imu_recovery_cooldown_samples;
  uint16_t imu_recovery_confirm_samples;
  float imu_accel_saturation_threshold;
  float imu_gyro_saturation_threshold;
  uint16_t imu_saturation_hard_fault_persistence_samples;
  uint8_t imu_saturation_multi_axis_limit;
  uint8_t imu_enable_all_soft_reject_salvage;
  uint8_t imu_enable_preflight_tare;
  uint16_t imu_tare_window_samples;
  float imu_tare_accel_gravity;
  float imu_boost_soft_threshold_scale;
  uint32_t imu_boost_soft_window_us;

  // ============================================================
  // Barometer Configuration
  // ============================================================
  float baro_sigma_base;         // Base noise floor (m)
  float baro_k_aero;             // Velocity-squared coefficient
  float baro_transonic_low;      // Transonic lower bound (m/s)
  float baro_transonic_high;     // Transonic upper bound (m/s)
  float baro_transonic_penalty;  // R inflation in transonic regime
  float baro_innovation_clamp;   // Max allowed innovation (m)

  // Multi-baro hardening policy
  float baro_voting_threshold_pa;
  float baro_hard_fault_threshold_pa;
  float baro_calibration_mismatch_tolerance_pa;
  uint16_t baro_hard_fault_suspect_samples;
  uint16_t baro_hard_fault_persistence_samples;
  uint16_t baro_recovery_cooldown_samples;
  uint16_t baro_recovery_confirm_samples;
  float baro_continuity_max_step_pa;
  uint8_t baro_enable_all_soft_reject_salvage;

  // ============================================================
  // GPS Configuration
  // ============================================================
  float gps_trust_factor;        // Accuracy inflation factor
  float gps_high_g_threshold;    // High-G threshold (g)
  float gps_high_g_r_factor;     // R multiplier above threshold
  float gps_vel_chi2_threshold;  // Velocity chi² rejection threshold
  float gps_pos_chi2_threshold;  // Position chi² rejection threshold
  // Soft-accept multipliers for marginal chi2 packets
  float gps_vel_soft_accept_multiplier;
  float gps_pos_soft_accept_multiplier;
  int32_t gps_delay_us;          // GPS delay from PPS (µs)
  uint8_t gps_max_consecutive_rejects; // Max rejects before covariance reset

  // ============================================================
  // Heading Configuration
  // ============================================================
  float heading_align_min_speed;   // Min speed for GPS heading alignment (m/s)
  float heading_align_max_sacc;    // Max GPS speed accuracy (m/s)
  float heading_align_max_gyro;    // Max body rate for alignment (rad/s)
  float launch_rail_heading;       // Hardcoded heading fallback (rad)
  float expected_dip_angle_rad;    // Magnetic dip at launch site (rad)
  float dip_angle_threshold_rad;   // Dip validation threshold (rad)
  uint8_t trust_hardcoded_heading; // Use hardcoded heading instead of sensors
  uint8_t enable_gps_cog_heading;  // Enable one-shot GPS COG heading alignment
  uint8_t gps_heading_bootstrap_mode; // 0: direct COG, 1: velocity-angle delta
  uint8_t enable_gps_cog_heading_fusion; // Enable continuous GPS COG heading fusion
  uint8_t disable_gps_after_alignment;  // Diagnostic: skip all GPS after heading snap
  uint8_t disable_gps_vel_lever_arm_attitude_jacobian; // Diagnostic: zero H_att/H_bg in GPS vel update
  uint8_t freeze_accel_bias_in_flight;  // Freeze accel bias state in flight mode
  uint8_t freeze_gyro_bias_in_flight;   // Freeze gyro bias state in flight mode
  float gps_vel_tumble_gyro_threshold; // Angular rate threshold for R inflation (rad/s)
  float gps_vel_tumble_r_factor;       // R multiplier when tumbling

  // ============================================================
  // Shadow Filter Configuration
  // ============================================================
  float shadow_rail_kp;            // Mahony proportional gain
  float shadow_rail_ki;            // Mahony integral gain
  float shadow_rail_gate;          // Accel feedback gate threshold (m/s²)
  float shadow_flight_omega_n;     // Observer natural frequency (rad/s)
  float shadow_flight_zeta;        // Observer damping ratio
  float shadow_aero_blind_speed;   // Disable baro correction above (m/s)
  float shadow_apogee_hysteresis;  // Apogee detection hysteresis (m/s)

  // ============================================================
  // Buffer Sizes
  // ============================================================
  uint16_t imu_buffer_size;        // IMU ring buffer capacity
  uint16_t baro_buffer_size;       // Baro ring buffer capacity
  uint16_t event_buffer_size;      // Event ring buffer capacity
  uint16_t checkpoint_buffer_size; // State checkpoint capacity
  uint16_t checkpoint_interval;    // IMU samples between checkpoints

  // ============================================================
  // Timing Configuration
  // ============================================================
  uint32_t liftoff_rewind_us;      // Rewind depth before liftoff (µs)
  uint32_t liftoff_rejection_us;   // Reject aiding sensors for (µs)
  uint32_t rail_clear_delay_us;    // Pause mag fusion after liftoff (µs)

  // ============================================================
  // Precision & Physics
  // ============================================================
  uint8_t force_float32;           // Force float32 computations
  float launch_latitude_deg;       // Launch site latitude (deg)
  float launch_longitude_deg;      // Launch site longitude (deg)
  float local_gravity;             // Computed local gravity (m/s²)
  
  // ============================================================
  // Magnetometer Site-Specific
  // ============================================================
  float mag_declination_rad;       // Magnetic declination (rad)
  float mag_expected_magnitude_ut; // Expected field magnitude (μT)
  float mag_magnitude_threshold_ut;// Magnitude rejection threshold (μT)

  // ============================================================
  // Feature Flags
  // ============================================================
  uint8_t use_gps_rewind;          // Enable GPS time-machine
  uint8_t use_lever_arm_averaging; // Average lever arm over GPS window
  uint8_t use_coning_compensation; // Two-sample coning correction
  uint8_t covariance_decimation;   // Update P every Nth step

  // ============================================================
  // Additional Tuning Parameters
  // ============================================================
  float gps_reset_p_pos;           // (m^2)
  float gps_reset_p_vel;           // (m/s)^2
  float gps_heading_variance;      // (rad)^2
  float post_align_yaw_var;        // (rad)^2
  float nis_divergence_threshold;  // NIS threshold
  uint16_t heading_resurrect_count;// Count
  uint16_t nis_max_consecutive_high; // Count
  float gyro_bias_lpf_alpha;       // Filter alpha
  uint32_t rail_checkpoint_interval_us; // Checkpoint interval

  // Sideslip constraint
  uint8_t enable_sideslip;          // Runtime toggle for sideslip constraint
  float sideslip_r_lateral;        // Measurement noise variance (m/s)²
  float sideslip_min_speed;        // Min NED speed (m/s)
  float sideslip_max_gyro;         // Max angular rate for validity (rad/s)
  uint16_t sideslip_decimation;    // Apply every N IMU steps

  // ============================================================
  // Replay-Configurable Process Noise (ESKF Process Model)
  // ============================================================
  float process_noise_accel_noise;      // m/s^2/sqrt(Hz)
  float process_noise_gyro_noise;       // rad/s/sqrt(Hz)
  float process_noise_accel_bias_walk;  // (m/s^2)^2/s
  float process_noise_gyro_bias_walk;   // (rad/s)^2/s
  float process_noise_baro_bias_walk;   // m^2/s

  // ============================================================
  // Replay-Configurable Initial Covariance (P0)
  // ============================================================
  float initial_cov_pos;           // Position sigma (m)
  float initial_cov_vel;           // Velocity sigma (m/s)
  float initial_cov_tilt;          // Roll/Pitch sigma (rad)
  float initial_cov_heading;       // Yaw sigma (rad)
  float initial_cov_accel_bias;    // Accel bias sigma (m/s^2)
  float initial_cov_gyro_bias;     // Gyro bias sigma (rad/s)
  float initial_cov_baro_bias;     // Baro bias sigma (m)
  
  // ============================================================
  // TuningConfig Conversion
  // ============================================================
  
  /// Convert to TuningConfig for ESKF initialization.
  /// Used by native_replay to configure filter from logged config.
  eskf::TuningConfig toTuningConfig() const {
    eskf::TuningConfig cfg;
    
    // Baro parameters
    cfg.baro_sigma_base = baro_sigma_base;
    cfg.baro_k_aero = baro_k_aero;
    cfg.baro_transonic_low = baro_transonic_low;
    cfg.baro_transonic_high = baro_transonic_high;
    cfg.baro_transonic_penalty = baro_transonic_penalty;
    cfg.baro_innovation_clamp = baro_innovation_clamp;
    cfg.baro_voting_threshold_pa = baro_voting_threshold_pa;
    cfg.baro_hard_fault_threshold_pa = baro_hard_fault_threshold_pa;
    cfg.baro_calibration_mismatch_tolerance_pa =
      baro_calibration_mismatch_tolerance_pa;
    cfg.baro_hard_fault_suspect_samples = baro_hard_fault_suspect_samples;
    cfg.baro_hard_fault_persistence_samples =
      baro_hard_fault_persistence_samples;
    cfg.baro_recovery_cooldown_samples = baro_recovery_cooldown_samples;
    cfg.baro_recovery_confirm_samples = baro_recovery_confirm_samples;
    cfg.baro_continuity_max_step_pa = baro_continuity_max_step_pa;
    cfg.baro_enable_all_soft_reject_salvage =
      baro_enable_all_soft_reject_salvage != 0;

    // IMU hardening parameters
    cfg.imu_gyro_voting_threshold = imu_gyro_voting_threshold;
    cfg.imu_accel_voting_threshold = imu_accel_voting_threshold;
    cfg.imu_gyro_hard_fault_threshold = imu_gyro_hard_fault_threshold;
    cfg.imu_accel_hard_fault_threshold = imu_accel_hard_fault_threshold;
    cfg.imu_hard_fault_suspect_samples = imu_hard_fault_suspect_samples;
    cfg.imu_hard_fault_persistence_samples =
      imu_hard_fault_persistence_samples;
    cfg.imu_recovery_cooldown_samples = imu_recovery_cooldown_samples;
    cfg.imu_recovery_confirm_samples = imu_recovery_confirm_samples;
    cfg.imu_accel_saturation_threshold = imu_accel_saturation_threshold;
    cfg.imu_gyro_saturation_threshold = imu_gyro_saturation_threshold;
    cfg.imu_saturation_hard_fault_persistence_samples =
      imu_saturation_hard_fault_persistence_samples;
    cfg.imu_saturation_multi_axis_limit = imu_saturation_multi_axis_limit;
    cfg.imu_enable_all_soft_reject_salvage =
      imu_enable_all_soft_reject_salvage != 0;
    cfg.imu_enable_preflight_tare = imu_enable_preflight_tare != 0;
    cfg.imu_tare_window_samples = imu_tare_window_samples;
    cfg.imu_tare_accel_gravity = imu_tare_accel_gravity;
    cfg.imu_boost_soft_threshold_scale = imu_boost_soft_threshold_scale;
    cfg.imu_boost_soft_window_us = imu_boost_soft_window_us;
    
    // GPS parameters
    cfg.gps_trust_factor = gps_trust_factor;
    cfg.gps_high_g_threshold = gps_high_g_threshold;
    cfg.gps_high_g_r_factor = gps_high_g_r_factor;
    cfg.gps_vel_chi2_threshold = gps_vel_chi2_threshold;
    cfg.gps_pos_chi2_threshold = gps_pos_chi2_threshold;
    cfg.gps_vel_soft_accept_multiplier = gps_vel_soft_accept_multiplier;
    cfg.gps_pos_soft_accept_multiplier = gps_pos_soft_accept_multiplier;
    cfg.gps_delay_us = gps_delay_us;
    cfg.gps_max_consecutive_rejects = gps_max_consecutive_rejects;
    cfg.gps_reset_p_pos = gps_reset_p_pos;
    cfg.gps_reset_p_vel = gps_reset_p_vel;
    cfg.gps_heading_variance = gps_heading_variance;
    
    // Heading parameters
    cfg.heading_align_min_speed = heading_align_min_speed;
    cfg.heading_align_max_sacc = heading_align_max_sacc;
    cfg.heading_align_max_gyro = heading_align_max_gyro;
    cfg.launch_rail_heading = launch_rail_heading;
    cfg.expected_dip_angle_rad = expected_dip_angle_rad;
    cfg.dip_angle_threshold_rad = dip_angle_threshold_rad;
    cfg.trust_hardcoded_heading = trust_hardcoded_heading != 0;
    cfg.enable_gps_cog_heading = enable_gps_cog_heading != 0;
    cfg.gps_heading_bootstrap_mode = static_cast<eskf::GpsHeadingBootstrapMode>(
      gps_heading_bootstrap_mode);
    cfg.enable_gps_cog_heading_fusion = enable_gps_cog_heading_fusion != 0;
    cfg.disable_gps_after_alignment = disable_gps_after_alignment != 0;
    cfg.disable_gps_vel_lever_arm_attitude_jacobian =
        disable_gps_vel_lever_arm_attitude_jacobian != 0;
    cfg.freeze_accel_bias_in_flight = freeze_accel_bias_in_flight != 0;
    cfg.freeze_gyro_bias_in_flight = freeze_gyro_bias_in_flight != 0;
    cfg.gps_vel_tumble_gyro_threshold = gps_vel_tumble_gyro_threshold;
    cfg.gps_vel_tumble_r_factor = gps_vel_tumble_r_factor;
    cfg.post_align_yaw_var = post_align_yaw_var;
    cfg.heading_resurrect_count = heading_resurrect_count;
    
    // Shadow filter parameters
    cfg.shadow_rail_kp = shadow_rail_kp;
    cfg.shadow_rail_ki = shadow_rail_ki;
    cfg.shadow_rail_gate = shadow_rail_gate;
    cfg.shadow_flight_omega_n = shadow_flight_omega_n;
    cfg.shadow_flight_zeta = shadow_flight_zeta;
    cfg.shadow_aero_blind_speed = shadow_aero_blind_speed;
    cfg.shadow_apogee_hysteresis = shadow_apogee_hysteresis;
    
    // Timing
    cfg.liftoff_rewind_us = liftoff_rewind_us;
    cfg.liftoff_rejection_us = liftoff_rejection_us;
    cfg.rail_clear_delay_us = rail_clear_delay_us;
    cfg.rail_checkpoint_interval_us = rail_checkpoint_interval_us;
    
    // Physics
    cfg.launch_latitude_deg = launch_latitude_deg;
    cfg.launch_longitude_deg = launch_longitude_deg;
    cfg.local_gravity = local_gravity;
    
    // Magnetometer Site-Specific
    cfg.mag_declination_rad = mag_declination_rad;
    cfg.mag_expected_magnitude_ut = mag_expected_magnitude_ut;
    cfg.mag_magnitude_threshold_ut = mag_magnitude_threshold_ut;

    // Misc
    cfg.nis_divergence_threshold = nis_divergence_threshold;
    cfg.nis_max_consecutive_high = nis_max_consecutive_high;
    cfg.gyro_bias_lpf_alpha = gyro_bias_lpf_alpha;

    // Feature Flags (partially mapped from existing)
    cfg.use_gps_rewind = use_gps_rewind != 0;
    cfg.use_lever_arm_averaging = use_lever_arm_averaging != 0;
    cfg.use_coning_compensation = use_coning_compensation != 0;
    cfg.covariance_decimation = covariance_decimation;

    // Sideslip constraint
    cfg.enable_sideslip = enable_sideslip != 0;
    cfg.sideslip_r_lateral = sideslip_r_lateral;
    cfg.sideslip_min_speed = sideslip_min_speed;
    cfg.sideslip_max_gyro = sideslip_max_gyro;
    cfg.sideslip_decimation = sideslip_decimation;
    
    return cfg;
  }

  /// Convert replay-configurable process-noise values.
  eskf::ProcessNoise toProcessNoise() const {
    eskf::ProcessNoise q{};
    q.accel_noise = process_noise_accel_noise;
    q.gyro_noise = process_noise_gyro_noise;
    q.accel_bias_walk = process_noise_accel_bias_walk;
    q.gyro_bias_walk = process_noise_gyro_bias_walk;
    q.baro_bias_walk = process_noise_baro_bias_walk;
    return q;
  }

  /// Convert replay-configurable initial covariance values.
  eskf::InitialCovariance toInitialCovariance() const {
    eskf::InitialCovariance p0{};
    p0.pos = initial_cov_pos;
    p0.vel = initial_cov_vel;
    p0.tilt = initial_cov_tilt;
    p0.heading = initial_cov_heading;
    p0.accel_bias = initial_cov_accel_bias;
    p0.gyro_bias = initial_cov_gyro_bias;
    p0.baro_bias = initial_cov_baro_bias;
    return p0;
  }

} __attribute__((packed));

static_assert(sizeof(ReplayableConfig) <= 384, "ReplayableConfig unexpectedly large");

/// Get compile-time default configuration
inline ReplayableConfig getDefaultConfig() {
  const eskf::ProcessNoise q_defaults = eskf::ProcessNoise::defaults();
  const eskf::InitialCovariance p0_defaults = eskf::InitialCovariance::defaults();

  return ReplayableConfig{
    // IMU Configuration
    .imu_odr_hz = APP_IMU_PRIMARY_ODR_HZ,
    .imu_accel_fsr_g = static_cast<uint8_t>(APP_IMU_ACCEL_FSR_G),
    .imu_gyro_fsr_dps = static_cast<uint16_t>(APP_IMU_GYRO_FSR_DPS),
    .imu_count = ESKF_APP_IMU_COUNT,

    .imu_gyro_voting_threshold =
      static_cast<float>(ESKF_IMU_GYRO_VOTING_THRESHOLD),
    .imu_accel_voting_threshold =
      static_cast<float>(ESKF_IMU_ACCEL_VOTING_THRESHOLD),
    .imu_gyro_hard_fault_threshold =
      static_cast<float>(ESKF_IMU_GYRO_HARD_FAULT_THRESHOLD),
    .imu_accel_hard_fault_threshold =
      static_cast<float>(ESKF_IMU_ACCEL_HARD_FAULT_THRESHOLD),
    .imu_hard_fault_suspect_samples =
      static_cast<uint16_t>(ESKF_IMU_HARD_FAULT_SUSPECT_SAMPLES),
    .imu_hard_fault_persistence_samples =
      static_cast<uint16_t>(ESKF_IMU_HARD_FAULT_PERSISTENCE_SAMPLES),
    .imu_recovery_cooldown_samples =
      static_cast<uint16_t>(ESKF_IMU_RECOVERY_COOLDOWN_SAMPLES),
    .imu_recovery_confirm_samples =
      static_cast<uint16_t>(ESKF_IMU_RECOVERY_CONFIRM_SAMPLES),
    .imu_accel_saturation_threshold =
      static_cast<float>(ESKF_IMU_ACCEL_SATURATION_THRESHOLD),
    .imu_gyro_saturation_threshold =
      static_cast<float>(ESKF_IMU_GYRO_SATURATION_THRESHOLD),
    .imu_saturation_hard_fault_persistence_samples =
      static_cast<uint16_t>(ESKF_IMU_SATURATION_HARD_FAULT_PERSISTENCE_SAMPLES),
    .imu_saturation_multi_axis_limit =
      static_cast<uint8_t>(ESKF_IMU_SATURATION_MULTI_AXIS_LIMIT),
    .imu_enable_all_soft_reject_salvage =
      static_cast<uint8_t>(ESKF_IMU_ENABLE_ALL_SOFT_REJECT_SALVAGE),
    .imu_enable_preflight_tare =
      static_cast<uint8_t>(ESKF_IMU_ENABLE_PREFLIGHT_TARE),
    .imu_tare_window_samples =
      static_cast<uint16_t>(ESKF_IMU_TARE_WINDOW_SAMPLES),
    .imu_tare_accel_gravity =
      static_cast<float>(ESKF_IMU_TARE_ACCEL_GRAVITY),
    .imu_boost_soft_threshold_scale =
      static_cast<float>(ESKF_IMU_BOOST_SOFT_THRESHOLD_SCALE),
    .imu_boost_soft_window_us =
      static_cast<uint32_t>(ESKF_IMU_BOOST_SOFT_WINDOW_US),

    // Barometer Configuration
    .baro_sigma_base = static_cast<float>(ESKF_BARO_SIGMA_BASE),
    .baro_k_aero = static_cast<float>(ESKF_BARO_K_AERO),
    .baro_transonic_low = static_cast<float>(ESKF_BARO_TRANSONIC_LOW),
    .baro_transonic_high = static_cast<float>(ESKF_BARO_TRANSONIC_HIGH),
    .baro_transonic_penalty = static_cast<float>(ESKF_BARO_TRANSONIC_PENALTY),
    .baro_innovation_clamp = static_cast<float>(ESKF_BARO_INNOVATION_CLAMP),
    .baro_voting_threshold_pa = static_cast<float>(ESKF_BARO_VOTING_THRESHOLD_PA),
    .baro_hard_fault_threshold_pa =
      static_cast<float>(ESKF_BARO_HARD_FAULT_THRESHOLD_PA),
    .baro_calibration_mismatch_tolerance_pa =
      static_cast<float>(ESKF_BARO_CALIBRATION_MISMATCH_TOLERANCE_PA),
    .baro_hard_fault_suspect_samples =
      static_cast<uint16_t>(ESKF_BARO_HARD_FAULT_SUSPECT_SAMPLES),
    .baro_hard_fault_persistence_samples =
      static_cast<uint16_t>(ESKF_BARO_HARD_FAULT_PERSISTENCE_SAMPLES),
    .baro_recovery_cooldown_samples =
      static_cast<uint16_t>(ESKF_BARO_RECOVERY_COOLDOWN_SAMPLES),
    .baro_recovery_confirm_samples =
      static_cast<uint16_t>(ESKF_BARO_RECOVERY_CONFIRM_SAMPLES),
    .baro_continuity_max_step_pa =
      static_cast<float>(ESKF_BARO_CONTINUITY_MAX_STEP_PA),
    .baro_enable_all_soft_reject_salvage =
      static_cast<uint8_t>(ESKF_BARO_ENABLE_ALL_SOFT_REJECT_SALVAGE),

    // GPS Configuration
    .gps_trust_factor = static_cast<float>(ESKF_GPS_TRUST_FACTOR),
    .gps_high_g_threshold = static_cast<float>(ESKF_GPS_HIGH_G_THRESHOLD),
    .gps_high_g_r_factor = static_cast<float>(ESKF_GPS_HIGH_G_R_FACTOR),
    .gps_vel_chi2_threshold = static_cast<float>(ESKF_GPS_VEL_CHI2_THRESHOLD),
    .gps_pos_chi2_threshold = static_cast<float>(ESKF_GPS_POS_CHI2_THRESHOLD),
    .gps_vel_soft_accept_multiplier = 20.0f,
    .gps_pos_soft_accept_multiplier = 20.0f,
    .gps_delay_us = ESKF_DEFAULT_GPS_DELAY_US,
    .gps_max_consecutive_rejects = static_cast<uint8_t>(ESKF_GPS_MAX_CONSECUTIVE_REJECTS > 255 ? 255 : ESKF_GPS_MAX_CONSECUTIVE_REJECTS),

    // Heading Configuration
    .heading_align_min_speed = static_cast<float>(ESKF_HEADING_ALIGN_MIN_SPEED),
    .heading_align_max_sacc = static_cast<float>(ESKF_HEADING_ALIGN_MAX_SACC),
    .heading_align_max_gyro = static_cast<float>(ESKF_HEADING_ALIGN_MAX_GYRO),
    .launch_rail_heading = static_cast<float>(ESKF_LAUNCH_RAIL_HEADING),
    .expected_dip_angle_rad = static_cast<float>(ESKF_EXPECTED_DIP_ANGLE_RAD),
    .dip_angle_threshold_rad = static_cast<float>(ESKF_DIP_ANGLE_THRESHOLD_RAD),
    .trust_hardcoded_heading = static_cast<uint8_t>(ESKF_TRUST_HARDCODED_HEADING),
    .enable_gps_cog_heading = static_cast<uint8_t>(ESKF_ENABLE_GPS_COG_HEADING),
    .gps_heading_bootstrap_mode = static_cast<uint8_t>(ESKF_GPS_HEADING_BOOTSTRAP_MODE),
    .enable_gps_cog_heading_fusion = static_cast<uint8_t>(ESKF_ENABLE_GPS_COG_HEADING_FUSION),
    .disable_gps_after_alignment = static_cast<uint8_t>(ESKF_DISABLE_GPS_AFTER_ALIGNMENT),
    .disable_gps_vel_lever_arm_attitude_jacobian = static_cast<uint8_t>(ESKF_DISABLE_GPS_VEL_LEVER_ARM_ATTITUDE_JACOBIAN),
    .freeze_accel_bias_in_flight = static_cast<uint8_t>(ESKF_FREEZE_ACCEL_BIAS_IN_FLIGHT),
    .freeze_gyro_bias_in_flight = static_cast<uint8_t>(ESKF_FREEZE_GYRO_BIAS_IN_FLIGHT),
    .gps_vel_tumble_gyro_threshold = static_cast<float>(ESKF_GPS_VEL_TUMBLE_GYRO_THRESHOLD),
    .gps_vel_tumble_r_factor = static_cast<float>(ESKF_GPS_VEL_TUMBLE_R_FACTOR),

    // Shadow Filter Configuration
    .shadow_rail_kp = static_cast<float>(ESKF_SHADOW_RAIL_KP),
    .shadow_rail_ki = static_cast<float>(ESKF_SHADOW_RAIL_KI),
    .shadow_rail_gate = static_cast<float>(ESKF_SHADOW_RAIL_GATE),
    .shadow_flight_omega_n = static_cast<float>(ESKF_SHADOW_FLIGHT_OMEGA_N),
    .shadow_flight_zeta = static_cast<float>(ESKF_SHADOW_FLIGHT_ZETA),
    .shadow_aero_blind_speed = static_cast<float>(ESKF_SHADOW_AERO_BLIND_SPEED),
    .shadow_apogee_hysteresis = static_cast<float>(ESKF_SHADOW_APOGEE_HYSTERESIS),

    // Buffer Sizes
    .imu_buffer_size = static_cast<uint16_t>(ESKF_IMU_BUFFER_SIZE > 65535 ? 65535 : ESKF_IMU_BUFFER_SIZE),
    .baro_buffer_size = static_cast<uint16_t>(ESKF_BARO_BUFFER_SIZE),
    .event_buffer_size = static_cast<uint16_t>(ESKF_EVENT_BUFFER_SIZE),
    .checkpoint_buffer_size = static_cast<uint16_t>(ESKF_CHECKPOINT_BUFFER_SIZE),
    .checkpoint_interval = static_cast<uint16_t>(ESKF_CHECKPOINT_INTERVAL),

    // Timing Configuration
    .liftoff_rewind_us = ESKF_LIFTOFF_REWIND_US,
    .liftoff_rejection_us = ESKF_LIFTOFF_REJECTION_US,
    .rail_clear_delay_us = ESKF_RAIL_CLEAR_DELAY_US,

    // Precision & Physics
    .force_float32 = static_cast<uint8_t>(ESKF_FORCE_FLOAT32),
    .launch_latitude_deg = static_cast<float>(ESKF_LAUNCH_SITE_LATITUDE_DEG),
    .launch_longitude_deg = 8.0f, // Default (placeholder)
    .local_gravity = static_cast<float>(eskf::constants::kGravityLocal),
    
    // Magnetometer Site-Specific
    .mag_declination_rad = 0.0f,
    .mag_expected_magnitude_ut = 50.0f,
    .mag_magnitude_threshold_ut = 10.0f,

    // Feature Flags
    .use_gps_rewind = static_cast<uint8_t>(ESKF_USE_GPS_REWIND),
    .use_lever_arm_averaging = static_cast<uint8_t>(ESKF_USE_LEVER_ARM_AVERAGING),
    .use_coning_compensation = static_cast<uint8_t>(ESKF_USE_CONING_COMPENSATION),
    .covariance_decimation = static_cast<uint8_t>(ESKF_COVARIANCE_DECIMATION),

    // Additional Tuning Parameters
    .gps_reset_p_pos = static_cast<float>(ESKF_GPS_RESET_P_POS),
    .gps_reset_p_vel = static_cast<float>(ESKF_GPS_RESET_P_VEL),
    .gps_heading_variance = static_cast<float>(ESKF_GPS_HEADING_VARIANCE),
    .post_align_yaw_var = static_cast<float>(ESKF_POST_ALIGN_YAW_VAR),
    .nis_divergence_threshold = static_cast<float>(ESKF_NIS_DIVERGENCE_THRESHOLD),
    .heading_resurrect_count = static_cast<uint16_t>(ESKF_HEADING_RESURRECT_COUNT),
    .nis_max_consecutive_high = static_cast<uint16_t>(ESKF_NIS_MAX_CONSECUTIVE_HIGH),
    .gyro_bias_lpf_alpha = static_cast<float>(ESKF_GYRO_BIAS_LPF_ALPHA),
    .rail_checkpoint_interval_us = static_cast<uint32_t>(ESKF_RAIL_CHECKPOINT_INTERVAL_US),

    // Sideslip constraint
    .enable_sideslip = 0,  // Disabled by default, enable per-experiment
    .sideslip_r_lateral = 1.0f,
    .sideslip_min_speed = 15.0f,
    .sideslip_max_gyro = 0.5f,
    .sideslip_decimation = 640,

    // Process noise
    .process_noise_accel_noise = static_cast<float>(q_defaults.accel_noise),
    .process_noise_gyro_noise = static_cast<float>(q_defaults.gyro_noise),
    .process_noise_accel_bias_walk =
      static_cast<float>(q_defaults.accel_bias_walk),
    .process_noise_gyro_bias_walk =
      static_cast<float>(q_defaults.gyro_bias_walk),
    .process_noise_baro_bias_walk =
      static_cast<float>(q_defaults.baro_bias_walk),

    // Initial covariance
    .initial_cov_pos = static_cast<float>(p0_defaults.pos),
    .initial_cov_vel = static_cast<float>(p0_defaults.vel),
    .initial_cov_tilt = static_cast<float>(p0_defaults.tilt),
    .initial_cov_heading = static_cast<float>(p0_defaults.heading),
    .initial_cov_accel_bias = static_cast<float>(p0_defaults.accel_bias),
    .initial_cov_gyro_bias = static_cast<float>(p0_defaults.gyro_bias),
    .initial_cov_baro_bias = static_cast<float>(p0_defaults.baro_bias),
  };
}

} // namespace appcfg
