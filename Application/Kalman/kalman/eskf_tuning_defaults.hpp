#pragma once
// ESKF Tuning Parameters - Algorithm noise values, thresholds, and gains
// Split from eskf_config.hpp for organization

// ============================================================
// Feature Flags
// ============================================================

#ifndef ESKF_USE_GPS_REWIND
#define ESKF_USE_GPS_REWIND 1 // Enable GPS "Time-Machine" replay
#endif

#ifndef ESKF_GPS_REWIND_DEPTH_MS
#define ESKF_GPS_REWIND_DEPTH_MS                                               \
  500 // Ring buffer history depth (ms)
#endif

#ifndef ESKF_USE_CENTRAL_DIFF
#define ESKF_USE_CENTRAL_DIFF 1 // Savitzky-Golay ω̇ (3 samples behind)
#endif

#ifndef ESKF_CENTRAL_DIFF_ORDER
#define ESKF_CENTRAL_DIFF_ORDER 7 // 7-point kernel (current implementation)
#endif

#ifndef ESKF_USE_BARO
#define ESKF_USE_BARO 1 // Compile-time barometer fusion gate
#endif

#ifndef ESKF_USE_MAG
#define ESKF_USE_MAG 1 // Compile-time magnetometer fusion gate
#endif

#ifndef ESKF_ENABLE_SIDESLIP
#define ESKF_ENABLE_SIDESLIP 1 // Aerodynamic velocity constraint (Section 4.2A)
#endif

#ifndef ESKF_ENABLE_SENSOR_VOTING
#define ESKF_ENABLE_SENSOR_VOTING 0 // Enable voting if >2 sensor available
#endif

// ============================================================
// Multi-IMU Hardening Policy (3+ IMUs)
// ============================================================

#ifndef ESKF_IMU_GYRO_VOTING_THRESHOLD
#define ESKF_IMU_GYRO_VOTING_THRESHOLD 0.5 // rad/s soft voting reject
#endif

#ifndef ESKF_IMU_ACCEL_VOTING_THRESHOLD
#define ESKF_IMU_ACCEL_VOTING_THRESHOLD 5.0 // m/s^2 soft voting reject
#endif

#ifndef ESKF_IMU_GYRO_HARD_FAULT_THRESHOLD
#define ESKF_IMU_GYRO_HARD_FAULT_THRESHOLD 6.0 // rad/s hard fault candidate
#endif

#ifndef ESKF_IMU_ACCEL_HARD_FAULT_THRESHOLD
#define ESKF_IMU_ACCEL_HARD_FAULT_THRESHOLD 25.0 // m/s^2 hard fault candidate
#endif

#ifndef ESKF_IMU_HARD_FAULT_SUSPECT_SAMPLES
#define ESKF_IMU_HARD_FAULT_SUSPECT_SAMPLES 2
#endif

#ifndef ESKF_IMU_HARD_FAULT_PERSISTENCE_SAMPLES
#define ESKF_IMU_HARD_FAULT_PERSISTENCE_SAMPLES 4
#endif

#ifndef ESKF_IMU_RECOVERY_COOLDOWN_SAMPLES
#define ESKF_IMU_RECOVERY_COOLDOWN_SAMPLES 8
#endif

#ifndef ESKF_IMU_RECOVERY_CONFIRM_SAMPLES
#define ESKF_IMU_RECOVERY_CONFIRM_SAMPLES 8
#endif

#ifndef ESKF_IMU_ACCEL_SATURATION_THRESHOLD
#define ESKF_IMU_ACCEL_SATURATION_THRESHOLD 250.0 // m/s^2 axis clipping
#endif

#ifndef ESKF_IMU_GYRO_SATURATION_THRESHOLD
#define ESKF_IMU_GYRO_SATURATION_THRESHOLD 34.0 // rad/s axis clipping
#endif

#ifndef ESKF_IMU_SATURATION_HARD_FAULT_PERSISTENCE_SAMPLES
#define ESKF_IMU_SATURATION_HARD_FAULT_PERSISTENCE_SAMPLES 4
#endif

#ifndef ESKF_IMU_SATURATION_MULTI_AXIS_LIMIT
#define ESKF_IMU_SATURATION_MULTI_AXIS_LIMIT 2
#endif

#ifndef ESKF_IMU_ENABLE_ALL_SOFT_REJECT_SALVAGE
#define ESKF_IMU_ENABLE_ALL_SOFT_REJECT_SALVAGE 1
#endif

#ifndef ESKF_IMU_STALE_PERSISTENCE_SAMPLES
#define ESKF_IMU_STALE_PERSISTENCE_SAMPLES 128
#endif

#ifndef ESKF_IMU_STALE_ACCEL_DELTA_THRESHOLD
#define ESKF_IMU_STALE_ACCEL_DELTA_THRESHOLD 1e-7
#endif

#ifndef ESKF_IMU_STALE_GYRO_DELTA_THRESHOLD
#define ESKF_IMU_STALE_GYRO_DELTA_THRESHOLD 1e-7
#endif

#ifndef ESKF_IMU_ENABLE_PREFLIGHT_TARE
#define ESKF_IMU_ENABLE_PREFLIGHT_TARE 1
#endif

#ifndef ESKF_IMU_TARE_WINDOW_SAMPLES
#define ESKF_IMU_TARE_WINDOW_SAMPLES 200
#endif

#ifndef ESKF_IMU_TARE_ACCEL_GRAVITY
#define ESKF_IMU_TARE_ACCEL_GRAVITY 9.80665
#endif

#ifndef ESKF_IMU_BOOST_SOFT_THRESHOLD_SCALE
#define ESKF_IMU_BOOST_SOFT_THRESHOLD_SCALE 2.0
#endif

#ifndef ESKF_IMU_BOOST_SOFT_WINDOW_US
#define ESKF_IMU_BOOST_SOFT_WINDOW_US 350000
#endif

// ============================================================
// Multi-Baro Hardening Policy (3+ barometers)
// ============================================================

#ifndef ESKF_BARO_VOTING_THRESHOLD_PA
#define ESKF_BARO_VOTING_THRESHOLD_PA 500.0
#endif

#ifndef ESKF_BARO_HARD_FAULT_THRESHOLD_PA
#define ESKF_BARO_HARD_FAULT_THRESHOLD_PA 3000.0
#endif

#ifndef ESKF_BARO_CALIBRATION_MISMATCH_TOLERANCE_PA
#define ESKF_BARO_CALIBRATION_MISMATCH_TOLERANCE_PA 600.0
#endif

#ifndef ESKF_BARO_HARD_FAULT_SUSPECT_SAMPLES
#define ESKF_BARO_HARD_FAULT_SUSPECT_SAMPLES 2
#endif

#ifndef ESKF_BARO_HARD_FAULT_PERSISTENCE_SAMPLES
#define ESKF_BARO_HARD_FAULT_PERSISTENCE_SAMPLES 4
#endif

#ifndef ESKF_BARO_RECOVERY_COOLDOWN_SAMPLES
#define ESKF_BARO_RECOVERY_COOLDOWN_SAMPLES 8
#endif

#ifndef ESKF_BARO_RECOVERY_CONFIRM_SAMPLES
#define ESKF_BARO_RECOVERY_CONFIRM_SAMPLES 8
#endif

#ifndef ESKF_BARO_CONTINUITY_MAX_STEP_PA
#define ESKF_BARO_CONTINUITY_MAX_STEP_PA 200.0
#endif

#ifndef ESKF_BARO_ENABLE_ALL_SOFT_REJECT_SALVAGE
#define ESKF_BARO_ENABLE_ALL_SOFT_REJECT_SALVAGE 1
#endif

#ifndef ESKF_BARO_STALE_PERSISTENCE_SAMPLES
#define ESKF_BARO_STALE_PERSISTENCE_SAMPLES 128
#endif

#ifndef ESKF_BARO_STALE_PRESSURE_DELTA_THRESHOLD_PA
#define ESKF_BARO_STALE_PRESSURE_DELTA_THRESHOLD_PA 1e-6
#endif

#ifndef ESKF_BARO_STALE_TEMPERATURE_DELTA_THRESHOLD_K
#define ESKF_BARO_STALE_TEMPERATURE_DELTA_THRESHOLD_K 1e-6
#endif

#ifndef ESKF_USE_CONING_COMPENSATION
#define ESKF_USE_CONING_COMPENSATION                                           \
  1 // Two-sample coning correction (Section 2.5.A)
#endif

#ifndef ESKF_COVARIANCE_DECIMATION
#define ESKF_COVARIANCE_DECIMATION                                             \
  1 // Update P every Nth step (1=every step). Note: accumulation code uses
    // dense multiplies, so N>1 is currently SLOWER. Needs sparse rewrite.
#endif

#ifndef ESKF_PREDICT_MAX_DT_S
#define ESKF_PREDICT_MAX_DT_S 0.1f // Clamp single predict step to 100ms
#endif

// Use averaged lever arm correction for GPS velocity
#ifndef ESKF_USE_LEVER_ARM_AVERAGING
#define ESKF_USE_LEVER_ARM_AVERAGING 1
#endif

// Use GPS course-over-ground (COG) for one-shot heading alignment.
// Enabled by default to establish initial NED frame at first reliable GPS fix.
#ifndef ESKF_ENABLE_GPS_COG_HEADING
#define ESKF_ENABLE_GPS_COG_HEADING 1
#endif

// One-shot GPS heading bootstrap mode:
// 0 = direct COG snap: yaw <- atan2(vE, vN)
// 1 = velocity-angle delta: yaw <- yaw + (gps_cog - eskf_cog)
// Mode 1 keeps GNSS velocity overwrite and position rotation behavior from forceYaw,
// but avoids directly injecting wind crab angle into yaw during bootstrap.
#ifndef ESKF_GPS_HEADING_BOOTSTRAP_MODE
#define ESKF_GPS_HEADING_BOOTSTRAP_MODE 0
#endif

// Use GPS COG for continuous heading fusion after alignment.
// Enabled with large R to prevent unbounded yaw drift while respecting
// that COG may not perfectly equal body yaw on tumbling rockets.
// Gated by min speed, max sAcc, max accel, and 3σ innovation check.
#ifndef ESKF_ENABLE_GPS_COG_HEADING_FUSION
#define ESKF_ENABLE_GPS_COG_HEADING_FUSION 1
#endif

// Diagnostic: disable all GPS corrections after heading alignment.
// When enabled, the first heading snap happens normally, but all subsequent
// GPS pos/vel/heading updates are skipped.  Pure IMU+baro after snap.
#ifndef ESKF_DISABLE_GPS_AFTER_ALIGNMENT
#define ESKF_DISABLE_GPS_AFTER_ALIGNMENT 0
#endif

// Tumble-rate GPS velocity R inflation
// When angular rate exceeds threshold, inflate R_vel to reduce attitude corruption
// from velocity corrections during tumbling. Corrections still update velocity
// (with reduced gain) but stop pushing attitude through P cross-terms.
#ifndef ESKF_GPS_VEL_TUMBLE_GYRO_THRESHOLD
#define ESKF_GPS_VEL_TUMBLE_GYRO_THRESHOLD 0.5 // rad/s (~29°/s)
#endif

#ifndef ESKF_GPS_VEL_TUMBLE_R_FACTOR
#define ESKF_GPS_VEL_TUMBLE_R_FACTOR 100.0 // R multiplier when tumbling
#endif

// Zero the attitude/gyro-bias terms in the GPS velocity lever-arm Jacobian.
// The lever-arm measurement correction (subtracting ω×r from GPS velocity)
// still applies, but H_att and H_gyrbias are zeroed. Combined with
// P-decoupling during tumbling, this prevents GPS velocity corrections
// from affecting attitude through both direct (H_att) and indirect
// (P_att_vel cross-term) paths.
#ifndef ESKF_DISABLE_GPS_VEL_LEVER_ARM_ATTITUDE_JACOBIAN
#define ESKF_DISABLE_GPS_VEL_LEVER_ARM_ATTITUDE_JACOBIAN 1
#endif

// Freeze IMU bias states during in-flight mode. Turn-on bias is injected at
// liftoff and then held fixed to avoid poorly observable in-flight drift.
#ifndef ESKF_FREEZE_ACCEL_BIAS_IN_FLIGHT
#define ESKF_FREEZE_ACCEL_BIAS_IN_FLIGHT 1
#endif

#ifndef ESKF_FREEZE_GYRO_BIAS_IN_FLIGHT
#define ESKF_FREEZE_GYRO_BIAS_IN_FLIGHT 1
#endif

// Magnetic declination in radians (positive = East declination)
#ifndef ESKF_MAG_DECLINATION_RAD
#define ESKF_MAG_DECLINATION_RAD 0.0
#endif

// ============================================================
// Dynamic Baro Variance (Section 6.4.B)
// ============================================================
// R_baro = (σ_base + K_aero·v²)² with transonic penalty

#ifndef ESKF_BARO_SIGMA_BASE
#define ESKF_BARO_SIGMA_BASE 2.0 // Base noise floor (m)
#endif

#ifndef ESKF_BARO_K_AERO
#define ESKF_BARO_K_AERO 0.001 // Velocity-squared coefficient
#endif

#ifndef ESKF_BARO_TRANSONIC_LOW
#define ESKF_BARO_TRANSONIC_LOW 240.0 // ~Mach 0.7 at sea level (m/s)
#endif

#ifndef ESKF_BARO_TRANSONIC_HIGH
#define ESKF_BARO_TRANSONIC_HIGH 400.0 // ~Mach 1.2 at sea level (m/s)
#endif

#ifndef ESKF_BARO_TRANSONIC_PENALTY
#define ESKF_BARO_TRANSONIC_PENALTY 10000.0 // Massive R inflation in transonic
#endif

// Barometer Innovation Clamping Threshold (meters)
#ifndef ESKF_BARO_INNOVATION_CLAMP
#define ESKF_BARO_INNOVATION_CLAMP 50.0
#endif

// ============================================================
// GPS Configuration (Section 6.2, 6.4.B.2)
// ============================================================

// GPS delay from PPS (microseconds, negative = measurement is earlier than PPS)
#ifndef ESKF_DEFAULT_GPS_DELAY_US
#define ESKF_DEFAULT_GPS_DELAY_US                                              \
  -100000 // -100ms default (velocity lags PPS by 100ms)
#endif

// GPS accuracy inflation factor (K_trust in Section 6.4.B.2)
#ifndef ESKF_GPS_TRUST_FACTOR
#define ESKF_GPS_TRUST_FACTOR 1.5 // Typical: 1.2 to 2.0
#endif

// High-G GPS variance inflation (Section 6.4.B.2)
#ifndef ESKF_GPS_HIGH_G_THRESHOLD
#define ESKF_GPS_HIGH_G_THRESHOLD 3.0 // Threshold in g (g-units)
#endif

#ifndef ESKF_GPS_HIGH_G_R_FACTOR
#define ESKF_GPS_HIGH_G_R_FACTOR 100.0 // R multiplier when above threshold
#endif

// ============================================================
// GPS Packet Rejection (Chi-Squared Gating)
// ============================================================

// Chi-Squared threshold for GPS velocity (3 DOF)
#ifndef ESKF_GPS_VEL_CHI2_THRESHOLD
#define ESKF_GPS_VEL_CHI2_THRESHOLD 12.0 // ~1% false rejection rate
#endif

// Chi-Squared threshold for GPS position (3 DOF)
#ifndef ESKF_GPS_POS_CHI2_THRESHOLD
#define ESKF_GPS_POS_CHI2_THRESHOLD 12.0
#endif

// Consecutive GPS velocity rejections before covariance inflation
#ifndef ESKF_GPS_MAX_CONSECUTIVE_REJECTS
#define ESKF_GPS_MAX_CONSECUTIVE_REJECTS 80 // ~5 seconds at 16Hz
#endif

// Covariance inflation values upon recovery trigger
#ifndef ESKF_GPS_RESET_P_POS
#define ESKF_GPS_RESET_P_POS 10000.0 // m² - "unknown position"
#endif

#ifndef ESKF_GPS_RESET_P_VEL
#define ESKF_GPS_RESET_P_VEL 100.0 // (m/s)² - "unknown velocity"
#endif

// ============================================================
// Launch Site Configuration
// ============================================================

#ifndef ESKF_LAUNCH_SITE_LATITUDE_DEG
#define ESKF_LAUNCH_SITE_LATITUDE_DEG                                          \
  46.848199 // Launch site: Payerne, Switzerland
#endif

// ============================================================
// Heading Alignment (Section 5.4)
// ============================================================

#ifndef ESKF_HEADING_ALIGN_MIN_SPEED
#define ESKF_HEADING_ALIGN_MIN_SPEED                                             \
  15.0 // Min horizontal speed for GNSS heading alignment (m/s)
#endif

#ifndef ESKF_HEADING_ALIGN_MAX_SACC
#define ESKF_HEADING_ALIGN_MAX_SACC 1.0 // Max GPS speed accuracy (m/s)
#endif

#ifndef ESKF_HEADING_ALIGN_MAX_GYRO
#define ESKF_HEADING_ALIGN_MAX_GYRO                                            \
  0.35 // Max body rate magnitude (rad/s, ~20°/s)
#endif

// ============================================================
// Heading State Machine (Section 5.4 Extended)
// ============================================================

// Launch rail heading fallback (radians from True North)
#ifndef ESKF_LAUNCH_RAIL_HEADING
#define ESKF_LAUNCH_RAIL_HEADING 0.0
#endif

// Trust hardcoded launch rail heading instead of sensor-based initialization
#ifndef ESKF_TRUST_HARDCODED_HEADING
#define ESKF_TRUST_HARDCODED_HEADING 0 // Default: use sensors
#endif

// Expected magnetic dip angle (inclination) at launch site (radians)
#ifndef ESKF_EXPECTED_DIP_ANGLE_RAD
#include <cmath>
#define ESKF_EXPECTED_DIP_ANGLE_RAD                                            \
  std::atan(2.0 * std::tan(ESKF_LAUNCH_SITE_LATITUDE_DEG *                     \
                           3.14159265358979323846 / 180.0))
#endif

// Dip angle validation threshold (radians)
#ifndef ESKF_DIP_ANGLE_THRESHOLD_RAD
#define ESKF_DIP_ANGLE_THRESHOLD_RAD 0.174 // ~10 degrees
#endif

// Rail clear delay (microseconds) - pause mag fusion after liftoff
#ifndef ESKF_RAIL_CLEAR_DELAY_US
#define ESKF_RAIL_CLEAR_DELAY_US 500000 // 500ms
#endif

// Consecutive reject threshold before "resurrection" snap
#ifndef ESKF_HEADING_RESURRECT_COUNT
#define ESKF_HEADING_RESURRECT_COUNT 50
#endif

// Continuous GPS heading fusion variance (radians²)
// R = 0.005 (~4° std dev) gives strong Kalman gain to anchor yaw near COG.
// The 3σ innovation gate in processHeadingUpdate rejects outliers.
// Additional protection from min-speed, max-sAcc, and high-G gates.
#ifndef ESKF_GPS_HEADING_VARIANCE
#define ESKF_GPS_HEADING_VARIANCE 0.001
#endif

// Post-Alignment Yaw Variance (radians^2)
// After a hard heading snap from GPS COG, yaw is known to the accuracy
// of the snap source. An inflated P_yaw drives large P_att_vel cross-terms
// through the process model, giving GPS velocity corrections excessive
// Kalman gain on attitude — the root cause of yaw drift during tumbling.
// Setting this to match roll/pitch uncertainty (0.0001) prevents the issue.
#ifndef ESKF_POST_ALIGN_YAW_VAR
#define ESKF_POST_ALIGN_YAW_VAR 0.0001 // (0.01 rad)^2 ~ 0.6° std dev
#endif

// NIS Divergence Threshold
#ifndef ESKF_NIS_DIVERGENCE_THRESHOLD
#define ESKF_NIS_DIVERGENCE_THRESHOLD 100.0
#endif

// NIS Consecutive High Threshold
#ifndef ESKF_NIS_MAX_CONSECUTIVE_HIGH
#define ESKF_NIS_MAX_CONSECUTIVE_HIGH 10
#endif

// ============================================================
// Shadow Filter Configuration (Section 5.1)
// ============================================================

// --- Rail Shadow Filter (Gated Mahony) ---
#ifndef ESKF_SHADOW_RAIL_KP
#define ESKF_SHADOW_RAIL_KP 1.0
#endif

#ifndef ESKF_SHADOW_RAIL_KI
#define ESKF_SHADOW_RAIL_KI 0.1
#endif

#ifndef ESKF_SHADOW_RAIL_GATE
#define ESKF_SHADOW_RAIL_GATE 0.5
#endif

// --- Flight Shadow Filter (Linear State Observer) ---
#ifndef ESKF_SHADOW_FLIGHT_OMEGA_N
#define ESKF_SHADOW_FLIGHT_OMEGA_N 3.14
#endif

#ifndef ESKF_SHADOW_FLIGHT_ZETA
#define ESKF_SHADOW_FLIGHT_ZETA 0.707
#endif

#ifndef ESKF_SHADOW_AERO_BLIND_SPEED
#define ESKF_SHADOW_AERO_BLIND_SPEED 100.0
#endif

#ifndef ESKF_SHADOW_AERO_BLIND_ENTER_SPEED
#define ESKF_SHADOW_AERO_BLIND_ENTER_SPEED ESKF_SHADOW_AERO_BLIND_SPEED
#endif

#ifndef ESKF_SHADOW_AERO_BLIND_EXIT_SPEED
#define ESKF_SHADOW_AERO_BLIND_EXIT_SPEED 90.0
#endif

#ifndef ESKF_SHADOW_AERO_BLIND_ENTER_DEBOUNCE_MS
#define ESKF_SHADOW_AERO_BLIND_ENTER_DEBOUNCE_MS 30
#endif

#ifndef ESKF_SHADOW_AERO_BLIND_EXIT_DEBOUNCE_MS
#define ESKF_SHADOW_AERO_BLIND_EXIT_DEBOUNCE_MS 60
#endif

#ifndef ESKF_SHADOW_APOGEE_HYSTERESIS
#define ESKF_SHADOW_APOGEE_HYSTERESIS 0.5
#endif

// --- Pre-Liftoff Gyro Bias Estimation ---
#ifndef ESKF_GYRO_BIAS_LPF_ALPHA
#define ESKF_GYRO_BIAS_LPF_ALPHA 0.9999
#endif

// --- Liftoff Transition ---
#ifndef ESKF_LIFTOFF_REWIND_US
#define ESKF_LIFTOFF_REWIND_US 400000 // 400ms
#endif

#ifndef ESKF_LIFTOFF_REJECTION_US
#define ESKF_LIFTOFF_REJECTION_US 500000 // 500ms
#endif

#ifndef ESKF_BARO_SNAPSHOT_MAX_AGE_US
#define ESKF_BARO_SNAPSHOT_MAX_AGE_US 50000 // 50ms stale snapshot rejection
#endif

// --- RailShadow Checkpoint Buffer ---
#ifndef ESKF_RAIL_CHECKPOINT_INTERVAL_US
#define ESKF_RAIL_CHECKPOINT_INTERVAL_US 10000 // 10ms = 100Hz
#endif

// ============================================================
// Logging Configuration
// ============================================================

#ifndef ESKF_LOG_STATE_HZ
#define ESKF_LOG_STATE_HZ 50 // State logging rate
#endif

#ifndef ESKF_LOG_COVARIANCE_HZ
#define ESKF_LOG_COVARIANCE_HZ 5 // P-diagonal logging rate
#endif

#ifndef ESKF_LOG_REWIND_FRAMES
#define ESKF_LOG_REWIND_FRAMES 0
#endif
