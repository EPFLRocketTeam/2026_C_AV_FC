#pragma once
// Shadow Filter Classes for ESKF System
// Part of Phase 1: Standalone Core ESKF Library
//
// Implements auxiliary filters for initialization and safety:
// - RailShadowFilter: Gated Mahony filter for pre-flight orientation
// - FlightShadowFilter: Linear state observer for apogee detection watchdog
//
// Reference: Kalman_filter_planification.md Section 5.1

#include "eskf_config.hpp"
#include "eskf_math.hpp"
#include "preprocessor/pressure_altitude.hpp"

namespace eskf {

// ============================================================
// RailShadow Checkpoint (for liftoff injection)
// ============================================================

/// Snapshot of RailShadowFilter state for liftoff injection.
/// Saved periodically to enable rewinding to pre-liftoff state.
struct RailShadowCheckpoint {
  eskf_scalar q[4];             ///< Gravity-aligned quaternion (arbitrary yaw)
  eskf_scalar gyro_bias[3];     ///< Gyro bias from LPF (NOT Mahony integral!)
  eskf_scalar heading_estimate; ///< Mag heading (radians)
  eskf_scalar heading_variance; ///< Heading uncertainty (rad²)
  bool heading_initialized;     ///< True if heading is valid
  uint64_t timestamp_us;        ///< When this state was captured

  // Ground reference (for barometer initialization)
  eskf_scalar ground_pressure_pa;              ///< Virtual ground pressure (Pa)
  eskf_scalar ground_temperature_k;            ///< Ground temperature (K)
  eskf_scalar baro_offsets_pa[ESKF_MAX_BAROS]; ///< Per-sensor tare offsets
  bool ground_reference_valid; ///< True if ground calibration complete
};

/// Debug state for detailed analysis
struct RailShadowDebug {
  eskf_scalar expected_accel_body[3];
  eskf_scalar error[3];
  eskf_scalar integral[3];
  eskf_scalar correction[3];
};

// ============================================================
// Rail Shadow Filter (Pre-Flight Gated Mahony)
// ============================================================

/// Gated Mahony complementary filter for determining "Down" vector on the pad.
///
/// This filter runs in parallel with the main ESKF during ground operations.
/// It provides a robust orientation estimate that rejects handling noise by
/// gating accelerometer feedback based on gravity magnitude.
///
/// At liftoff, the quaternion from this filter seeds the main ESKF.
class RailShadowFilter {
public:
  RailShadowFilter() = default;

  /// Initialize with tuning configuration.
  void init(const TuningConfig &cfg);

  /// Reset filter to identity quaternion and zero integral.
  void reset();

  /// Update filter with IMU measurement.
  /// @param accel_body Accelerometer reading in body frame (m/s²)
  /// @param gyro_body Gyroscope reading in body frame (rad/s)
  /// @param dt Time step (seconds)
  /// @param timestamp_us Current timestamp for checkpoint saving
  void update(const eskf_scalar accel_body[3], const eskf_scalar gyro_body[3],
              eskf_scalar dt, uint64_t timestamp_us = 0);

  /// Get current orientation quaternion (body → NED, Hamilton convention).
  /// Format: [w, x, y, z] where w is scalar.
  /// NOTE: This returns the GRAVITY-BASED quaternion with arbitrary yaw.
  /// Use getCombinedQuaternion() for the full orientation including heading.
  const eskf_scalar *quaternion() const { return q_; }

  /// Copy quaternion to output array.
  void getQuaternion(eskf_scalar q_out[4]) const;

  /// Get combined quaternion with heading applied.
  /// Combines gravity-based roll/pitch with heading estimate (or rail
  /// fallback).
  /// @param q_out Output quaternion [w, x, y, z]
  void getCombinedQuaternion(eskf_scalar q_out[4]) const;

  /// Returns true if the last update used accelerometer feedback.
  bool isGateOpen() const { return gate_open_; }

  /// Set local gravity magnitude (call after GPS fix for Somigliana
  /// correction).
  void setLocalGravity(eskf_scalar g) { g_local_ = g; }

  /// Update heading state from validated magnetometer reading.
  /// Uses simple scalar Kalman filter independent of gravity orientation.
  /// @param heading_rad Tilt-compensated heading (radians)
  /// @param R Measurement variance (radians²)
  void updateHeading(eskf_scalar heading_rad, eskf_scalar R);

  /// Check if heading has been initialized (first valid mag received).
  bool isHeadingInitialized() const { return heading_initialized_; }

  /// Current heading variance (rad^2).
  eskf_scalar headingVariance() const { return heading_variance_; }

  // --- Gyro Bias Estimation (LPF) ---

  /// Get current gyro bias estimate from low-pass filter.
  /// This is estimated directly from raw gyro (stationary assumption).
  const eskf_scalar *gyroBias() const { return gyro_bias_lpf_; }

  // --- Checkpoint Management ---

  /// Save current state as a checkpoint (call periodically from IMU
  /// processing).
  /// @param timestamp_us Current timestamp
  void saveCheckpoint(uint64_t timestamp_us);

  /// Find the most recent checkpoint before given timestamp.
  /// @param timestamp_us Target timestamp
  /// @return Pointer to checkpoint or nullptr if none available
  const RailShadowCheckpoint *findCheckpointBefore(uint64_t timestamp_us) const;

  /// Get current state as a checkpoint (fallback when buffer empty).
  /// @param timestamp_us Timestamp to assign to the checkpoint
  /// @return Checkpoint with current state
  RailShadowCheckpoint currentAsCheckpoint(uint64_t timestamp_us = 0) const;

  /// Compute combined quaternion from a checkpoint (for liftoff
  /// initialization). Reconstructs the full orientation from checkpoint's
  /// gravity quaternion and heading.
  /// @param cp Checkpoint to use
  /// @param q_out Output quaternion [w, x, y, z]
  static void
  getCombinedQuaternionFromCheckpoint(const RailShadowCheckpoint &cp,
                                      eskf_scalar q_out[4]);

  /// Initialize quaternion from first accelerometer sample.
  /// Computes shortest-arc rotation to align body "up" with NED up [0, 0, -1].
  /// Call once at startup to avoid ~90° initial error from identity quaternion.
  /// @param accel_body First valid accelerometer reading in body frame (m/s²)
  void initializeFromAccel(const eskf_scalar accel_body[3]);

  // --- Ground Reference Accumulator ---

  /// Update ground reference with barometer samples.
  /// Called pre-liftoff to accumulate ground P/T for each sensor.
  /// Samples are accumulated in 1-second windows; completed windows stored in
  /// ring buffer.
  /// @param pressures Raw pressure array [baro_count] (Pa)
  /// @param temps Raw temperature array [baro_count] (K)
  /// @param valid Per-sensor validity flags [baro_count]
  /// @param baro_count Number of barometers
  /// @param timestamp_us Current timestamp
  void updateBaro(const eskf_scalar *pressures, const eskf_scalar *temps,
                  const bool *valid, size_t baro_count, uint64_t timestamp_us);

  /// Get the oldest complete ground reference (for liftoff initialization).
  /// Uses oldest window to avoid motor vibration contamination.
  /// @return Ground reference with virtual ground P/T and per-sensor tare
  /// offsets
  const GroundReference &getOldestGroundReference() const;

  /// Build a best-effort ground reference from the current active window.
  /// Intended for early-liftoff fallback when no complete 1-second window
  /// exists yet.
  /// @param out Filled with virtual ground pressure/temperature and per-sensor
  /// offsets when enough samples are available
  /// @return true when fallback reference was produced
  bool estimateGroundReferenceFromActiveWindow(GroundReference &out) const;

  /// Check if ground reference calibration is complete.
  bool isGroundReferenceValid() const { return ground_ref_.valid; }

  /// Number of complete pre-flight 1-second ground-reference windows.
  uint16_t groundReferenceWindowCount() const {
    return static_cast<uint16_t>(window_count_);
  }

  /// Get debug state
  const RailShadowDebug &getDebugState() const { return debug_state_; }

private:
  TuningConfig cfg_;                ///< Runtime tuning parameters
  eskf_scalar q_[4] = {1, 0, 0, 0}; ///< Quaternion [w, x, y, z] (gravity-based)
  eskf_scalar integral_[3] = {0, 0, 0}; ///< PI controller integral term
  eskf_scalar g_local_ = constants::kGravityLocal; ///< Local gravity magnitude
  bool gate_open_ = false; ///< State of accelerometer feedback gate

  // Heading follower state (independent from gravity orientation)
  eskf_scalar heading_estimate_ = 0; ///< Best heading estimate (radians)
  eskf_scalar heading_variance_ =
      1000.0; ///< Heading variance (starts huge = unknown)
  bool heading_initialized_ = false; ///< First valid heading received?

  // Gyro bias LPF state (observes ALL 3 axes, unlike Mahony integral!)
  eskf_scalar gyro_bias_lpf_[3] = {0, 0, 0}; ///< Low-pass filtered gyro bias

  // Checkpoint ring buffer for liftoff rewind
  RailShadowCheckpoint checkpoint_buffer_[ESKF_RAIL_CHECKPOINT_BUFFER_SIZE];
  size_t checkpoint_head_ = 0;
  size_t checkpoint_count_ = 0;
  uint64_t last_checkpoint_us_ = 0;

  // --- Ground Reference 1-Second Window Accumulator ---
  static constexpr size_t kGroundRefWindowCount = 5;
  static constexpr uint64_t kGroundRefWindowUs = 1000000; ///< 1 second

  struct GroundRefWindow {
    eskf_scalar pressure_sum[ESKF_MAX_BAROS] = {0};
    eskf_scalar temperature_sum = 0;
    uint32_t sample_count[ESKF_MAX_BAROS] = {0};
    uint32_t temp_sample_count = 0;
    uint64_t window_start_us = 0;
    bool complete = false;
  };

  GroundRefWindow current_window_;
  bool current_window_active_ = false;
  GroundRefWindow window_buffer_[kGroundRefWindowCount];
  size_t window_head_ = 0;
  size_t window_count_ = 0;
  GroundReference ground_ref_;

  RailShadowDebug debug_state_;

  void finalizeCurrentWindow();
  void computeGroundReference();
};

// ============================================================
// Flight Shadow Filter (Apogee Watchdog Observer)
// ============================================================

/// Linear state observer for vertical dynamics (apogee detection watchdog).
///
/// This filter maintains its own dead-reckoning attitude (pure gyro
/// integration) independent of the main ESKF. If the ESKF diverges, this filter
/// continues to provide a mathematically impossible-to-diverge backup for
/// apogee detection.
///
/// State vector: [z, vz] where z is vertical position (NED Down, negative = up)
/// and vz is vertical velocity (NED Down, negative = ascending).
class FlightShadowFilter {
public:
  FlightShadowFilter() = default;

  /// Initialize with tuning configuration.
  void init(const TuningConfig &cfg);

  /// Reset filter with initial quaternion from Rail Shadow.
  /// @param initial_q Quaternion from Rail Shadow at liftoff [w, x, y, z]
  void reset(const eskf_scalar initial_q[4]);

  /// High-rate IMU prediction step.
  /// Updates dead-reckoning attitude and integrates vertical dynamics.
  /// @param accel_body Accelerometer reading in body frame (m/s²)
  /// @param gyro_body Gyroscope reading in body frame (rad/s)
  /// @param dt Time step (seconds)
  /// @param timestamp_us Optional timestamp for event logging (0 = no logging)
  void predict(const eskf_scalar accel_body[3], const eskf_scalar gyro_body[3],
               eskf_scalar dt, uint64_t timestamp_us = 0);

  /// Barometer correction step (observer update).
  /// Applies innovation-based correction to position and velocity states.
  /// @param z_baro Measured barometric altitude (m, positive up → converted to
  /// NED)
  /// @param dt_baro Time since last baro measurement (seconds)
  void correctBaro(eskf_scalar z_baro, eskf_scalar dt_baro);

  /// Barometer correction with Innovation Transport (Section 3.4.A).
  /// Innovation is calculated using snapshot from trigger time.
  /// @param measured_alt Measured barometric altitude (m, positive up)
  /// @param predicted_alt_at_trigger Altitude from filter at trigger time
  /// @param dt_baro Time since last baro measurement (seconds)
  void correctBaroWithSnapshot(eskf_scalar measured_alt,
                               eskf_scalar predicted_alt_at_trigger,
                               eskf_scalar dt_baro);

  /// Save current altitude for Innovation Transport (call at baro trigger
  /// time). Also captures timestamp for automatic dt calculation.
  /// @param trigger_timestamp_us Timestamp when conversion was commanded
  void saveAltitudeSnapshot(uint64_t trigger_timestamp_us);

  /// Barometer correction using previously saved snapshot (Innovation
  /// Transport). Call saveAltitudeSnapshot() at trigger time, then this when
  /// measurement ready.
  /// @param measured_alt Measured barometric altitude (m, positive up)
  /// @param measurement_ready_us Timestamp when measurement was retrieved
  void correctBaroFromSnapshot(eskf_scalar measured_alt,
                               uint64_t measurement_ready_us);

  /// Get current vertical position (NED Down: negative = above origin).
  eskf_scalar altitude() const { return z_; }

  /// Get current altitude (positive up) for consumers expecting standard
  /// convention.
  eskf_scalar altitudeUp() const { return -z_; }

  /// Get current vertical velocity (NED Down: negative = ascending).
  eskf_scalar velocity() const { return v_; }

  /// Returns true if currently in aero-blind mode (ignoring barometer).
  bool isAeroBlind() const { return is_aero_blind_; }

  /// Returns true when a re-engagement snap is pending/latched.
  bool wasAeroBlind() const { return was_aero_blind_; }

  /// Aero-blind entry debounce accumulator (seconds).
  eskf_scalar aeroBlindEnterAccumS() const { return aero_blind_enter_accum_s_; }

  /// Aero-blind exit debounce accumulator (seconds).
  eskf_scalar aeroBlindExitAccumS() const { return aero_blind_exit_accum_s_; }

  /// Last position snap magnitude applied when exiting aero-blind (meters).
  eskf_scalar lastReengageSnapDeltaM() const {
    return last_reengage_snap_delta_m_;
  }

  /// Timestamp of the last IMU predict update (0 when unavailable).
  uint64_t lastPredictTimestampUs() const { return last_predict_timestamp_us_; }

  /// Force aero-blind exit when IMU prediction is stale/unavailable.
  /// This arms the usual one-shot re-engagement snap on next baro correction.
  void forceExitAeroBlindForImuOutage();

  /// Set local gravity magnitude.
  void setLocalGravity(eskf_scalar g) { g_local_ = g; }

  /// Get current dead-reckoning quaternion (for debugging).
  const eskf_scalar *quaternion() const { return q_; }

private:
  TuningConfig cfg_; ///< Runtime tuning parameters

  // Dead-reckoning attitude (independent of ESKF)
  eskf_scalar q_[4] = {1, 0, 0, 0}; ///< Body→NED quaternion [w, x, y, z]

  // Vertical state
  eskf_scalar z_ = 0; ///< Vertical position (NED Down, m)
  eskf_scalar v_ = 0; ///< Vertical velocity (NED Down, m/s)

  // Configuration
  eskf_scalar g_local_ = constants::kGravityLocal;

  // Aero-blind state machine
  bool is_aero_blind_ = false;
  bool was_aero_blind_ = false; ///< For re-engagement snap logic
  eskf_scalar aero_blind_enter_accum_s_ = 0;
  eskf_scalar aero_blind_exit_accum_s_ = 0;
  eskf_scalar last_reengage_snap_delta_m_ = 0;
  uint64_t last_predict_timestamp_us_ = 0;

  // Innovation Transport snapshot (Section 3.4.A)
  eskf_scalar saved_alt_snapshot_ = 0;  ///< Altitude at baro trigger time
  uint64_t snapshot_timestamp_us_ = 0;  ///< Trigger timestamp
  uint64_t last_baro_timestamp_us_ = 0; ///< Last correction timestamp for dt
  bool has_alt_snapshot_ = false;       ///< True if snapshot is valid
  eskf_scalar last_known_dt_baro_ =
      0.01; ///< Cached dt for fallback (default 10ms)
};

} // namespace eskf
