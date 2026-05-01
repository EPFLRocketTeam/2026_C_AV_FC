// Virtual IMU Preprocessing Pipeline - Multi-Sensor Voting Architecture
// Part of Phase 2: ESKF Pre-Processing Layer
//
// Implements N-sensor fusion with voting-based outlier rejection:
// - Gyro voting: median + threshold rejection
// - Accel voting: "rough" projection to common point, then median check
// - Final fusion: uses "smooth" derivative on averaged raw data
//
// See Kalman_filter_planification.md Section 2.2

#pragma once
#include "../eskf_config.hpp"
#include "../eskf_math.hpp"
#include "../eskf_types.hpp"
#include "sensor_calibration.hpp"

namespace eskf {

// ============================================================
// Sensor Status Flags
// ============================================================

/// Sensor health status (per-sensor)
enum class SensorStatus : uint8_t {
  OK = 0,        // Sensor functional, data available
  SOFT_FAIL = 1, // Outlier rejected by voting (temporary sample-level reject)
  HARD_FAIL = 2, // Sensor produced a hard-fault sample this frame
  INHIBITED = 3, // Sensor excluded by persistent hard-fault logic
  RECOVERING = 4 // Sensor in cooldown/verification before re-enable
};

/// Persistent IMU health lifecycle state.
enum class SensorHealthState : uint8_t {
  ACTIVE = 0,
  SUSPECT = 1,
  INHIBITED = 2,
  RECOVERING = 3,
};

/// Runtime policy that can change with mission phase.
struct VirtualImuRuntimePolicy {
  /// Multiplier applied to soft voting thresholds only.
  eskf_scalar soft_threshold_scale = 1.0;

  /// True while within configured boost/liftoff vibration window.
  bool boost_phase = false;

  /// True once estimator has entered in-flight mode.
  /// Preflight-only adaptations (such as windowed tare estimation) stop
  /// updating and hold their last completed preflight window values.
  bool in_flight = false;
};

// ============================================================
// CG Model Callback
// ============================================================

/// Callback function type for dynamic CG position
/// Returns CG position in body frame based on current state (e.g., mass
/// consumed)
/// @param user_data  User-provided context pointer
/// @param timestamp_us Sample timestamp for interpolation (microseconds)
/// @param cg_out     Output: CG position [3] in body frame (meters)
using CgModelCallback =
  void (*)(void *user_data, uint64_t timestamp_us, eskf_scalar cg_out[3]);

// ============================================================
// Per-IMU Configuration
// ============================================================

/// Configuration for a single IMU sensor
struct ImuSensorConfig {
  /// Rotation matrix: sensor frame → body frame (3×3, row-major)
  eskf_scalar to_body[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  /// Sensor position in body frame (meters, for lever-arm calculation)
  eskf_scalar position[3] = {0, 0, 0};

  /// Enable/disable this sensor
  bool enabled = false;
};

// ============================================================
// Virtual IMU Configuration
// ============================================================

/// Configuration for the Virtual IMU preprocessing pipeline
struct VirtualImuConfig {
  /// Per-sensor configurations (up to ESKF_MAX_IMUS)
  ImuSensorConfig imus[ESKF_MAX_IMUS];

  /// Number of configured IMUs (0 to ESKF_MAX_IMUS)
  size_t imu_count = 0;

  /// Calibration data pointer (optional, null = no calibration)
  /// Pointer to array of ImuCalibration[ESKF_MAX_IMUS]
  /// Must remain valid for lifetime of VirtualImu
  const ImuCalibration *calibration = nullptr;

  /// CG model callback (for dynamic lever-arm calculation)
  CgModelCallback cg_callback = nullptr;
  void *cg_user_data = nullptr;

  /// Static CG position (used if cg_callback is null)
  eskf_scalar static_cg[3] = {0, 0, 0};

  /// Voting thresholds
  eskf_scalar gyro_voting_threshold =
      0.5; // rad/s (reject if |ω_i - median| > this)
  eskf_scalar accel_voting_threshold =
      5.0; // m/s² (reject if |a_i - median| > this)

    /// Hard-fault thresholds (strict, not boost-adaptive)
    eskf_scalar gyro_hard_fault_threshold =
      6.0; // rad/s (hard fault candidate when |ω_i - median| > this)
    eskf_scalar accel_hard_fault_threshold =
      25.0; // m/s² (hard fault candidate when |a_i - median| > this)

    /// Persistence / hysteresis for health transitions
    uint16_t hard_fault_suspect_samples = 2;
    uint16_t hard_fault_persistence_samples = 4;
    uint16_t recovery_cooldown_samples = 8;
    uint16_t recovery_confirm_samples = 8;

    /// Saturation/clipping policy
    eskf_scalar accel_saturation_threshold =
      250.0; // m/s² (absolute axis clipping/saturation threshold)
    eskf_scalar gyro_saturation_threshold =
      34.0; // rad/s (~1950 dps, absolute axis clipping threshold)
    uint16_t saturation_hard_fault_persistence_samples = 4;
    uint8_t saturation_multi_axis_limit =
      2; // multi-axis clipping is hard-fault eligible even in boost

    /// Deterministic continuity salvage when all sensors are soft-rejected.
    bool enable_all_soft_reject_salvage = true;

    /// Optional omega-dot norm clip before lever-arm correction (rad/s^2).
    /// Set <= 0 to disable clipping.
    eskf_scalar omega_dot_max_norm = 300.0;

    /// Optional max norm for lever-arm correction vector (m/s^2).
    /// Set <= 0 to disable clipping.
    eskf_scalar lever_arm_correction_max_norm = 35.0;

  /// Per-sensor stale/frozen sample detection.
  /// If accel and gyro both remain unchanged (within thresholds) for
  /// stale_persistence_samples consecutive samples, the sample is treated as a
  /// hard-fault observation for persistent-health transitions.
  uint16_t stale_persistence_samples = 128;
  eskf_scalar stale_accel_delta_threshold = static_cast<eskf_scalar>(1e-7);
  eskf_scalar stale_gyro_delta_threshold = static_cast<eskf_scalar>(1e-7);

  /// Preflight per-IMU windowed tare.
  /// Applied after calibration/rotation and before stale/voting/fusion.
  /// Gyro tare is a direct body-frame mean per window.
  /// Accel tare uses norm mismatch projected onto accel direction:
  ///   a_bias_sample = (||a|| - tare_accel_gravity) * (a / ||a||)
  /// The last completed preflight window is held for flight.
  bool enable_preflight_tare = true;
  uint16_t tare_window_samples = 200;
  eskf_scalar tare_accel_gravity = static_cast<eskf_scalar>(9.80665);

  /// Enable voting (auto-disabled if only 1 sensor)
  bool voting_enabled = true;

  /// Use centered Savitzky-Golay ω̇ (3 samples behind) when true
  bool use_central_diff = ESKF_USE_CENTRAL_DIFF;
};

// ============================================================
// Virtual IMU Output
// ============================================================

/// Extended IMU frame with voting status
struct VirtualImuOutput {
  ImuFrame frame;                         // CG-corrected accel/gyro
  SensorStatus imu_status[ESKF_MAX_IMUS]; // Per-sensor status after voting
  // Per-IMU calibrated + rotated body-frame values before voting.
  // Index 0..imu_count-1 correspond to configured sensors.
  eskf_scalar imu_accel_body[ESKF_MAX_IMUS][3];
  eskf_scalar imu_gyro_body[ESKF_MAX_IMUS][3];
  size_t valid_imu_count;                 // Number of valid sensors
  eskf_scalar effective_centroid[3];      // Centroid of valid sensors
  eskf_scalar nav_accel[3];               // Pre-lever-arm accel (body frame)
  eskf_scalar nav_gyro[3];                // Pre-lever-arm gyro (body frame)
  eskf_scalar omega_dot[3];               // Smoothed angular acceleration
  eskf_scalar omega_dot_unclamped[3];     // Raw smoothed omega-dot
  eskf_scalar tangential_correction[3];   // omega_dot x r term
  eskf_scalar centripetal_correction[3];  // omega x (omega x r) term
  eskf_scalar lever_arm_correction[3];    // Applied correction (tangential + centripetal)
  eskf_scalar lever_arm_correction_unclamped[3]; // Unclamped correction
  eskf_scalar cg[3];                      // CG used for lever-arm calc
  eskf_scalar lever_arm[3];               // centroid - cg
  eskf_scalar omega_dot_norm = 0;         // |omega_dot| after clipping
  eskf_scalar omega_dot_unclamped_norm = 0; // |omega_dot| before clipping
  eskf_scalar lever_arm_correction_norm = 0; // |applied correction|
  eskf_scalar lever_arm_correction_unclamped_norm = 0; // |raw correction|
  bool degraded_output = false;           // True if degraded-but-usable frame
  bool continuity_salvage_used = false;   // True if all-soft-reject salvage chosen
  bool saturation_detected = false;       // Any clipping/saturation observed
  bool omega_dot_clipped = false;         // True when omega-dot clipping was applied
  bool lever_arm_correction_clipped = false; // True when correction norm was clipped
  SensorHealthState imu_health[ESKF_MAX_IMUS]; // Persistent health state
  uint16_t imu_hard_fault_counter[ESKF_MAX_IMUS]; // Persistence counter snapshot
};

// ============================================================
// Virtual IMU Class
// ============================================================

/// Virtual IMU Preprocessing Pipeline
///
/// Fuses N IMUs into a single high-fidelity virtual sensor:
/// 1. Gyro voting: median + threshold → fused ω
/// 2. ω̇_rough for accel voting (discarded after)
/// 3. Accel voting: project to center, median check
/// 4. ω̇_smooth on fused gyro (for navigation)
/// 5. Final lever-arm correction from effective centroid to CG
class VirtualImu {
public:
  VirtualImu() = default;

  /// Configure the preprocessor with sensor geometry
  void configure(const VirtualImuConfig &cfg);

  /// Set runtime policy (phase-aware soft-threshold adaptation).
  void setRuntimePolicy(const VirtualImuRuntimePolicy &policy);

  /// Reset internal state (call on filter reset)
  void reset();

  /// Process a batch of raw IMU samples into CG-corrected frames.
  ///
  /// @param accel_data   Raw accel data [imu_count × count × 3]
  /// @param gyro_data    Raw gyro data [imu_count × count × 3]
  /// @param temp_data    Per-sample temperature [imu_count × count] in Kelvin
  /// (null = no thermal cal)
  /// @param statuses     Per-sensor status flags [imu_count]
  /// @param count        Number of samples per sensor
  /// @param t0_us        Timestamp of first sample in batch
  /// @param out          Output buffer for CG-corrected frames
  /// @param out_cap      Capacity of output buffer
  /// @param sample_dt_us Sample period in microseconds (from actual IMU
  /// timestamps)
  /// @return             Number of valid output frames written
  size_t process(const eskf_sensor_t *const accel_data[ESKF_MAX_IMUS],
                 const eskf_sensor_t *const gyro_data[ESKF_MAX_IMUS],
                 const eskf_scalar *const temp_data[ESKF_MAX_IMUS],
                 const SensorStatus statuses[ESKF_MAX_IMUS], size_t count,
                 uint64_t t0_us, VirtualImuOutput *out, size_t out_cap,
                 uint32_t sample_dt_us,
                 const eskf_scalar *gyro_bias_body = nullptr);

  /// Number of samples the output lags behind input due to central difference
  size_t lookAheadSamples() const;

private:
  VirtualImuConfig cfg_{};
  VirtualImuRuntimePolicy runtime_policy_{};

  // History buffer for derivative computation
  static constexpr size_t kHistorySize = 7;
  eskf_scalar omega_history_[kHistorySize][3] = {};
  eskf_scalar accel_history_[kHistorySize][3] = {};
  eskf_scalar imu_accel_body_history_[kHistorySize][ESKF_MAX_IMUS][3] = {};
  eskf_scalar imu_gyro_body_history_[kHistorySize][ESKF_MAX_IMUS][3] = {};
  uint64_t timestamp_history_[kHistorySize] = {};
  SensorStatus status_history_[kHistorySize][ESKF_MAX_IMUS] = {};
  SensorHealthState health_history_[kHistorySize][ESKF_MAX_IMUS] = {};
  uint16_t hard_fault_counter_history_[kHistorySize][ESKF_MAX_IMUS] = {};
  bool degraded_history_[kHistorySize] = {};
  bool salvage_history_[kHistorySize] = {};
  bool saturation_history_[kHistorySize] = {};
  size_t history_count_ = 0;

  // Previous omega for backward difference (rough path)
  eskf_scalar prev_omega_[3] = {0, 0, 0};
  bool has_prev_omega_ = false;

  // Persistent health lifecycle state
  SensorHealthState health_state_[ESKF_MAX_IMUS] = {};
  uint16_t hard_fault_counter_[ESKF_MAX_IMUS] = {};
  uint16_t healthy_counter_[ESKF_MAX_IMUS] = {};
  uint16_t saturation_counter_[ESKF_MAX_IMUS] = {};
  uint16_t stale_counter_[ESKF_MAX_IMUS] = {};
  bool has_prev_sample_[ESKF_MAX_IMUS] = {};
  eskf_scalar prev_accel_body_[ESKF_MAX_IMUS][3] = {};
  eskf_scalar prev_gyro_body_[ESKF_MAX_IMUS][3] = {};

  // Per-IMU preflight windowed tare state.
  bool tare_frozen_ = false;
  uint16_t tare_window_count_[ESKF_MAX_IMUS] = {};
  eskf_scalar tare_window_gyro_sum_[ESKF_MAX_IMUS][3] = {};
  eskf_scalar tare_window_accel_sum_[ESKF_MAX_IMUS][3] = {};
  eskf_scalar gyro_tare_body_[ESKF_MAX_IMUS][3] = {};
  eskf_scalar accel_tare_body_[ESKF_MAX_IMUS][3] = {};

  // Pre-computed PCB center (centroid of enabled sensors)
  // Computed once in configure() to avoid per-sample calculation
  eskf_scalar pcb_center_[3] = {0, 0, 0};

  /// Get current CG position (from callback or static)
  void getCurrentCG(uint64_t timestamp_us, eskf_scalar cg_out[3]) const;

  /// Compute median of N 3D vectors (per-axis)
  void computeMedian3D(eskf_scalar median_out[3], const eskf_scalar data[][3],
                       const bool valid[], size_t count) const;

  /// Vote on gyro data, mark outliers as SOFT_FAIL
  void voteGyro(eskf_scalar fused_gyro[3], const eskf_scalar gyro_body[][3],
                SensorStatus statuses[], size_t count) const;

  /// Vote on accel data using rough ω̇, mark outliers as SOFT_FAIL
  void voteAccel(const eskf_scalar accel_body[][3], const eskf_scalar omega[3],
                 const eskf_scalar omega_dot_rough[3], SensorStatus statuses[],
                 size_t count) const;

  /// Compute effective centroid of valid sensors
  void computeEffectiveCentroid(eskf_scalar centroid_out[3],
                                const SensorStatus statuses[],
                                size_t count) const;

  /// Average raw data from valid sensors
  void averageValidSensors(eskf_scalar avg_accel[3], eskf_scalar avg_gyro[3],
                           const eskf_scalar accel_body[][3],
                           const eskf_scalar gyro_body[][3],
                           const SensorStatus statuses[], size_t count) const;

  /// Rotate vector from sensor frame to body frame
  void rotateToBody(eskf_scalar out[3], const eskf_scalar R[3][3],
                    const eskf_scalar sensor[3]) const;

  /// Compute ω̇ using backward difference (rough, for voting)
  void computeOmegaDotRough(eskf_scalar omega_dot[3],
                            const eskf_scalar omega_current[3], eskf_scalar dt);

  /// Compute ω̇ using Savitzky-Golay (smooth, for navigation)
  void computeOmegaDotSmooth(eskf_scalar omega_dot[3], eskf_scalar dt) const;

  /// Apply lever-arm correction: a_cg = a_sens - (ω̇ × r + ω × (ω × r))
  void applyLeverArmCorrection(eskf_scalar accel_cg[3],
                               const eskf_scalar accel_sens[3],
                               const eskf_scalar omega[3],
                               const eskf_scalar omega_dot[3],
                               const eskf_scalar lever_arm[3],
                               eskf_scalar tangential_out[3],
                               eskf_scalar centripetal_out[3],
                               eskf_scalar correction_unclamped_out[3],
                               eskf_scalar correction_applied_out[3],
                               bool *correction_clipped) const;
};

} // namespace eskf
