#pragma once
// ESKF Logger Interface - Abstract logging for portable Kalman filter
// Part of Phase 1: Standalone Core ESKF Library
//
// Design Goals:
// 1. Keep kalman folder standalone and portable across projects
// 2. Allow injection of project-specific logging backends
// 3. Provide null implementation as default (zero overhead when unused)
//
// Usage:
//   // In your project's application layer:
//   class MySdLogger : public eskf::IEskfLogger { ... };
//   MySdLogger logger(&sdLogWriter);
//   eskf.setLogger(&logger);
//
// The kalman code calls logger methods at appropriate points.
// If no logger is set, NullEskfLogger is used (does nothing).

#include "eskf_config.hpp"
#include "eskf_types.hpp"
#include <stdint.h>

namespace eskf {

// ============================================================
// Event Types for Diagnostic Logging
// ============================================================

/// Categories of events the ESKF can log
enum class EskfEventType : uint8_t {
  // --- Initialization Events ---
  FilterReset = 0x01,       ///< Filter was reset to defaults
  FilterInitialized = 0x02, ///< Filter initialized with state/covariance

  // --- Mode Transitions ---
  ModeChanged = 0x10,        ///< FilterMode changed (Settling→Flight, etc.)
  HeadingAligned = 0x11,     ///< One-shot GPS heading alignment performed
  HeadingSnapped = 0x12,     ///< Heading hard-snap (mag or resurrection)
  HeadingInitialized = 0x13, ///< First heading measurement received

  // --- Sensor Corrections ---
  GpsPosCorrection = 0x20,   ///< GPS position update applied
  GpsVelCorrection = 0x21,   ///< GPS velocity update applied
  BaroCorrection = 0x22,     ///< Barometer update applied
  MagCorrection = 0x23,      ///< Magnetometer heading update applied
  SideslipCorrection = 0x24, ///< Sideslip constraint applied
  ZuptApplied = 0x25,        ///< Zero Velocity Update applied
  ZaruApplied = 0x26,        ///< Zero Angular Rate Update applied
  GpsHeadingCorrection = 0x27, ///< GPS COG heading update applied

  // --- GPS Packet Rejection (Chi² gating + ingress gate drops) ---
  GpsVelRejected = 0x30, ///< GPS velocity failed Chi² gate (packet rejected)
  GpsPosRejected = 0x31, ///< GPS position failed Chi² gate (pos only rejected)
  GpsCovarianceReset = 0x32, ///< Covariance inflated after prolonged rejection
  GpsRejectedByWindow = 0x33, ///< GPS packet dropped by rejection window gate

  // --- Rewind Events ---
  RewindStarted = 0x40,      ///< GPS time-machine rewind initiated
  RewindCompleted = 0x41,    ///< Rewind replay completed
  RewindNoCheckpoint = 0x42, ///< Rewind fell back to oldest checkpoint
  RewindDataGap = 0x43,      ///< Rewind detected missing IMU data
  RewindGapCovInflated = 0x44, ///< Covariance inflated after rewind data gap

  // --- Liftoff Events ---
  LiftoffSnapInjected = 0x50,  ///< Rail Shadow state injected at liftoff
  RejectionWindowStart = 0x51, ///< Aiding sensor rejection window started
  RejectionWindowEnd = 0x52,   ///< Rejection window ended

  // --- Health/Diagnostic Events ---
  NisDivergenceWarning = 0x60,   ///< Consecutive high NIS values detected
  FilterDiverged = 0x61,         ///< Filter flagged as diverged (NaN or NIS)
  QuaternionRenormalized = 0x62, ///< Quaternion drift corrected
  PredictDtClamped = 0x63,       ///< Predict dt was clamped to max limit

  // --- Buffer Events ---
  ImuBufferOverflow = 0x70,   ///< IMU ring buffer dropped samples
  BaroBufferOverflow = 0x71,  ///< Baro ring buffer dropped samples
  EventBufferOverflow = 0x72, ///< Event ring buffer dropped samples

  // --- Barometer Specific ---
  BaroTransonicBlind = 0x80,    ///< Baro in transonic penalty mode
  BaroInnovationClamped = 0x81, ///< Baro innovation was clamped
  BaroSnapshotDropped = 0x82,   ///< Pending baro snapshot dropped as stale

  // --- Shadow Filter Events ---
  GroundRefCalibrated =
      0x90, ///< Ground reference calibration complete (pre-flight)
  AeroBlindEntered = 0x91, ///< Flight Shadow entered aero-blind mode
  AeroBlindExited = 0x92,  ///< Flight Shadow exited aero-blind mode
  RailShadowHeadingConverged =
      0x93, ///< Rail Shadow heading variance below threshold

  // --- Extended Health Events ---
  ImuHealthTransition = 0x94,   ///< VirtualImu health FSM transition
  BaroHealthTransition = 0x95,  ///< VirtualBaro health FSM transition
  GnssHealthTransition = 0x96,  ///< GNSS stale/recovering transition
  MagHealthTransition = 0x97,   ///< Magnetometer stale/recovering transition
  ImuContinuitySalvage = 0x98,  ///< All-soft-reject IMU salvage used
  BaroContinuitySalvage = 0x99, ///< All-soft-reject baro salvage used
  GroundRefFallbackUsed = 0x9A, ///< Liftoff used active-window ground fallback
  TurnOnAccelBiasLatched = 0x9B, ///< Turn-on accel bias estimate became valid
  GnssFixDropped = 0x9C,        ///< GNSS sample skipped due unusable fix
  BaroQueueFlushed = 0x9D,      ///< Unread baro queue flushed after state reset

  // --- Critical Snapshot Markers ---
  RewindStateSnapshotPre = 0xA0,  ///< Forced state snapshot before rewind
  RewindStateSnapshotPost = 0xA1, ///< Forced state snapshot after rewind replay
  HeadingStateSnapshotPre = 0xA2, ///< Forced state snapshot before heading snap
  HeadingStateSnapshotPost = 0xA3, ///< Forced state snapshot after heading snap
};

// ============================================================
// Logged State Snapshot
// ============================================================

/// Compact state snapshot for logging (uses float for storage efficiency)
struct StateSnapshot {
  float p[3];      ///< Position NED (m)
  float v[3];      ///< Velocity NED (m/s)
  float q[4];      ///< Quaternion [w,x,y,z]
  float b_acc[3];  ///< Accel bias (m/s²)
  float b_gyro[3]; ///< Gyro bias (rad/s)
  float b_baro;    ///< Baro bias (m)
  // Debug telemetry for yaw/GNSS heading behavior.
  float yaw_rad;                  ///< Current yaw extracted from quaternion
  float heading_meas_rad;         ///< Last heading measurement applied/gated
  float heading_innovation_rad;   ///< Last heading innovation (wrapped)
  float heading_gate_threshold;   ///< Last heading gate threshold (rad²)
  float gps_vel_chi2;             ///< Last GPS velocity Chi² value
  float gps_vel_chi2_threshold;   ///< Last GPS velocity Chi² threshold
  uint64_t timestamp_us;
  uint8_t mode;  ///< FilterMode cast to uint8_t
  uint8_t flags; ///< Heading aligned (bit0), heading init (bit1)
  uint8_t heading_update_result; ///< HeadingUpdateResult (latest)
  uint8_t gps_vel_accepted;      ///< Last GPS velocity gate decision (0/1)
};

/// Covariance diagonal snapshot for logging
struct CovarianceSnapshot {
  float P_diag[kDimError]; ///< Diagonal elements of P matrix
  uint64_t timestamp_us;
};

/// GPS rejection event details
enum class GpsRejectReason : uint8_t {
  Unknown = 0,
  VelocityChi2 = 1,
  PositionChi2 = 2,
  CovarianceResetThreshold = 3,
  RejectionWindow = 4,
  HistoryUnavailable = 5,
  RewindCheckpointUnavailable = 6,
};

struct GpsRejectionInfo {
  float chi2_vel;       ///< Velocity Chi² value
  float chi2_pos;       ///< Position Chi² value (if applicable)
  float threshold;      ///< Rejection threshold used
  uint32_t consecutive; ///< Consecutive rejection count
  GpsRejectReason reason; ///< Rejection reason code for schema/analysis
};

/// Rewind event details
struct RewindInfo {
  uint64_t target_timestamp_us;     ///< Target timestamp
  uint64_t rewind_origin_timestamp_us; ///< Kalman time when rewind triggered
  uint64_t checkpoint_timestamp_us; ///< Checkpoint used
  uint32_t imu_replayed;            ///< Number of IMU samples replayed
  uint32_t baro_replayed;           ///< Number of baro samples replayed
  uint32_t events_replayed;         ///< Number of events replayed
  uint32_t catchup_duration_us;     ///< catchUp duration for rewind completion
  uint32_t pending_gap_dt_us;       ///< Injected gap compensation duration
  bool had_data_gap;                ///< IMU data gap detected
};

// ============================================================
// Shadow Filter State Snapshots
// ============================================================

/// Rail Shadow Filter state snapshot (pre-flight logging)
struct RailShadowSnapshot {
  float q[4];             ///< Combined quaternion [w,x,y,z] (gravity + heading)
  float heading_rad;      ///< Heading estimate (radians)
  float heading_variance; ///< Heading uncertainty (rad²)
  float gyro_bias[3];     ///< LPF gyro bias estimate (rad/s)
  float ground_pressure_pa; ///< Ground reference pressure (Pa)
  uint16_t window_count;  ///< Completed ground-reference windows
  uint64_t timestamp_us;
  uint8_t
      flags; ///< bit0: gate_open, bit1: heading_init, bit2: ground_ref_valid
};

/// Flight Shadow Filter state snapshot (in-flight logging)
struct FlightShadowSnapshot {
  float altitude_m;   ///< Current altitude (positive up, m)
  float velocity_mps; ///< Vertical velocity (NED down, m/s)
  float q[4];         ///< Dead-reckoning quaternion [w,x,y,z]
  float aero_blind_enter_accum_s; ///< Debounce accumulator for enter threshold
  float aero_blind_exit_accum_s;  ///< Debounce accumulator for exit threshold
  float last_reengage_snap_delta_m; ///< Last position snap magnitude (m)
  uint64_t timestamp_us;
  uint8_t flags; ///< bit0: aero_blind, bit1: was_aero_blind
};

// ============================================================
// IMU Pipeline Debug Snapshot (pre-flight diagnosis)
// ============================================================

/// Debug snapshot for tracing IMU processing into Rail Shadow orientation.
/// This is intended for offline analysis of static tilt errors.
struct ImuPipelineSnapshot {
  float accel_body[3];      ///< Pre-lever-arm accel (body frame)
  float gyro_body[3];       ///< Pre-lever-arm gyro (body frame)
  float omega_dot[3];       ///< Smoothed angular acceleration (rad/s²)
  float accel_cg[3];        ///< CG-corrected accel used by filter (m/s²)
  float gyro_cg[3];         ///< CG gyro (rad/s) (same as frame.gyro)
  float centroid[3];        ///< Effective centroid of valid IMUs (m)
  float cg[3];              ///< Current CG used for lever arm (m)
  float lever_arm[3];       ///< centroid - cg (m)
  float nav_gyro_unbiased[3];   ///< nav_gyro after bias removal (rad/s)
  float gyro_bias_body[3];      ///< Bias used for lever-arm correction (rad/s)
  float lever_arm_correction[3]; ///< accel_cg - accel_body (m/s²)
  float omega_dot_unclamped[3]; ///< Smoothed omega-dot before clipping (rad/s²)
  float tangential_correction[3]; ///< omega_dot x r term (m/s²)
  float centripetal_correction[3]; ///< omega x (omega x r) term (m/s²)
  float lever_arm_correction_unclamped[3]; ///< Raw tangential+centripetal (m/s²)
  float omega_dot_norm;         ///< |omega_dot| after clipping (rad/s²)
  float omega_dot_unclamped_norm; ///< |omega_dot| before clipping (rad/s²)
  float lever_arm_correction_norm; ///< |applied correction| (m/s²)
  float lever_arm_correction_unclamped_norm; ///< |raw correction| (m/s²)
  uint8_t omega_dot_clipped;    ///< 1 if omega-dot clipping applied
  uint8_t lever_arm_correction_clipped; ///< 1 if correction clipping applied
  float rail_q_gravity[4];  ///< Rail Shadow gravity quaternion [w,x,y,z]
  float rail_q_combined[4]; ///< Rail Shadow combined quaternion [w,x,y,z]

  // Debug fields
  float rail_expected_accel[3];
  float rail_error[3];
  float rail_integral[3];
  float rail_correction[3];

  float imu0_accel_body[3]; ///< IMU0 accel sample used for this frame
  float imu0_gyro_body[3];  ///< IMU0 gyro sample used for this frame
  float imu1_accel_body[3]; ///< IMU1 accel sample used for this frame
  float imu1_gyro_body[3];  ///< IMU1 gyro sample used for this frame

  float accel_norm;                  ///< |accel_cg| (m/s²)
  float dt_s;                        ///< Sample period used (s)
  uint8_t imu0_present;              ///< 1 if imu0_* fields are valid
  uint8_t imu1_present;              ///< 1 if imu1_* fields are valid
  uint8_t valid_imu_count;           ///< Number of valid IMUs
  uint8_t gate_open;                 ///< Rail Shadow gate state (0/1)
  uint8_t imu_status[ESKF_MAX_IMUS]; ///< Per-IMU status (SensorStatus)
  uint64_t timestamp_us;             ///< Sample timestamp
};

// ============================================================
// IMU In-Flight Debug Snapshot
// ============================================================

/// Debug snapshot for tracing in-flight IMU integration steps.
/// accel_ned is computed using the pre-update attitude (q before predict).
struct ImuDynamicsSnapshot {
  float accel_body[3]; ///< Bias-corrected accel (body frame, m/s²)
  float gyro_body[3];  ///< Bias-corrected gyro (body frame, rad/s)
  float accel_ned[3];  ///< Specific force in NED (pre-update attitude)
  float gravity_ned[3]; ///< Gravity vector in NED (m/s²)
  float vel_inc_ned[3]; ///< Delta-v in NED (m/s)
  float q[4];           ///< Attitude quaternion after predict [w,x,y,z]
  float dt_s;           ///< Sample period used (s)
  uint64_t timestamp_us; ///< Sample timestamp
};

// ============================================================
// Abstract Logger Interface
// ============================================================

/// Abstract interface for ESKF logging.
/// Implement this to connect the kalman filter to your logging system.
/// All methods are designed to be fast and non-blocking.
class IEskfLogger {
public:
  virtual ~IEskfLogger() = default;

  // ============================================================
  // State Logging (called at ESKF_LOG_STATE_HZ rate)
  // ============================================================

  /// Log full state snapshot.
  /// Called periodically based on ESKF_LOG_STATE_HZ configuration.
  /// @param snapshot Complete state snapshot
  virtual void logState(const StateSnapshot &snapshot) = 0;

  /// Log a state snapshot bypassing periodic rate limiting.
  /// Used around critical events (rewind, heading snaps) for deterministic
  /// before/after visibility in logs.
  /// @param snapshot Complete state snapshot
  virtual void logStateCritical(const StateSnapshot &snapshot) {
    logState(snapshot);
  }

  // ============================================================
  // Covariance Logging (called at ESKF_LOG_COVARIANCE_HZ rate)
  // ============================================================

  /// Log covariance diagonal.
  /// Called periodically based on ESKF_LOG_COVARIANCE_HZ configuration.
  /// @param snapshot Covariance diagonal snapshot
  virtual void logCovariance(const CovarianceSnapshot &snapshot) = 0;

  // ============================================================
  // Event Logging (called on significant events)
  // ============================================================

  /// Log a diagnostic event with optional scalar payload.
  /// @param event Event type
  /// @param timestamp_us Event timestamp
  /// @param value Optional scalar value (interpretation depends on event)
  virtual void logEvent(EskfEventType event, uint64_t timestamp_us,
                        float value = 0.0f) = 0;

  /// Log GPS rejection event with details.
  /// @param event GpsVelRejected, GpsPosRejected, or GpsCovarianceReset
  /// @param timestamp_us Event timestamp
  /// @param info Rejection details
  virtual void logGpsRejection(EskfEventType event, uint64_t timestamp_us,
                               const GpsRejectionInfo &info) = 0;

  /// Log rewind event with details.
  /// @param event RewindStarted, RewindCompleted, etc.
  /// @param timestamp_us Event timestamp
  /// @param info Rewind details
  virtual void logRewind(EskfEventType event, uint64_t timestamp_us,
                         const RewindInfo &info) = 0;

  // ============================================================
  // Correction Logging (optional, for detailed debugging)
  // ============================================================

  /// Log innovation and NIS after a correction.
  /// Override to enable detailed correction logging.
  /// @param event Correction event type (GpsPosCorrection, BaroCorrection,
  /// etc.)
  /// @param timestamp_us Correction timestamp
  /// @param innovation Measurement innovation (z - h)
  /// @param nis Normalized Innovation Squared
  virtual void logCorrection(EskfEventType event, uint64_t timestamp_us,
                             float innovation, float nis) {
    // Default: only log the event
    logEvent(event, timestamp_us, nis);
  }

  // ============================================================
  // Shadow Filter Logging (pre-flight and in-flight)
  // ============================================================

  /// Log Rail Shadow Filter state (pre-flight orientation estimator).
  /// Called periodically during pre-flight phase.
  /// @param snapshot Rail Shadow state snapshot
  virtual void logRailShadow(const RailShadowSnapshot &snapshot) = 0;

  /// Log Flight Shadow Filter state (apogee detection backup).
  /// Called periodically during in-flight phase.
  /// @param snapshot Flight Shadow state snapshot
  virtual void logFlightShadow(const FlightShadowSnapshot &snapshot) = 0;

  // ============================================================
  // IMU Pipeline Debug Logging (pre-flight diagnostics)
  // ============================================================

  /// Log detailed IMU processing snapshot for debugging static tilt errors.
  /// @param snapshot IMU pipeline debug snapshot
  virtual void logImuPipeline(const ImuPipelineSnapshot &snapshot) {
    (void)snapshot;
  }

  /// Log in-flight IMU integration debug snapshot.
  /// @param snapshot IMU dynamics debug snapshot
  virtual void logImuDynamics(const ImuDynamicsSnapshot &snapshot) {
    (void)snapshot;
  }
};

// ============================================================
// Null Logger (Default - Zero Overhead)
// ============================================================

/// Null logger that does nothing.
/// Used as default when no logger is configured.
/// All methods are inlined and optimized away by the compiler.
class NullEskfLogger : public IEskfLogger {
public:
  void logState(const StateSnapshot &) override {}
  void logStateCritical(const StateSnapshot &) override {}
  void logCovariance(const CovarianceSnapshot &) override {}
  void logEvent(EskfEventType, uint64_t, float) override {}
  void logGpsRejection(EskfEventType, uint64_t,
                       const GpsRejectionInfo &) override {}
  void logRewind(EskfEventType, uint64_t, const RewindInfo &) override {}
  void logCorrection(EskfEventType, uint64_t, float, float) override {}
  void logRailShadow(const RailShadowSnapshot &) override {}
  void logFlightShadow(const FlightShadowSnapshot &) override {}
  void logImuPipeline(const ImuPipelineSnapshot &) override {}
  void logImuDynamics(const ImuDynamicsSnapshot &) override {}
};

// ============================================================
// Rate Limiter Helper
// ============================================================

/// Helper class to implement rate-limited logging.
/// Use in your logging hooks to enforce ESKF_LOG_STATE_HZ etc.
class LogRateLimiter {
public:
  /// @param interval_us Minimum interval between logs (microseconds)
  explicit LogRateLimiter(uint64_t interval_us) : interval_us_(interval_us) {}

  /// Check if enough time has passed since last log.
  /// @param now_us Current timestamp
  /// @return true if should log now
  bool shouldLog(uint64_t now_us) {
    if (now_us - last_log_us_ >= interval_us_) {
      last_log_us_ = now_us;
      return true;
    }
    return false;
  }

  /// Reset the limiter (e.g., after filter reset)
  void reset() { last_log_us_ = 0; }

private:
  uint64_t interval_us_;
  uint64_t last_log_us_ = 0;
};

/// Factory function to create rate limiter from Hz configuration.
inline LogRateLimiter makeRateLimiter(uint32_t hz) {
  return LogRateLimiter(hz > 0 ? 1000000ULL / hz : 0);
}

// ============================================================
// Global Logger Instance
// ============================================================

/// Get the currently configured logger.
/// Returns NullEskfLogger if none configured.
IEskfLogger &getEskfLogger();

/// Set the logger instance.
/// @param logger Pointer to logger (nullptr restores NullEskfLogger)
/// @note The caller owns the logger and must ensure it outlives the filter.
void setEskfLogger(IEskfLogger *logger);

} // namespace eskf
