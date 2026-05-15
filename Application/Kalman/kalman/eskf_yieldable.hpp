#pragma once
// ESKF Yieldable Wrapper - Unified Timestamp-Controlled Replay
// Part of the GPS "Time-Machine" replay feature.
//
// Design principles:
// 1. All pushX() methods ONLY buffer data - no immediate processing
// 2. catchUp() is the ONLY place where events are processed
// 3. The filter has a processing timestamp that advances via catchUp()
// 4. GPS rewind works by setting the timestamp backwards before catchUp()
//
// This unified design simplifies control flow and ensures all sensors
// are processed in correct temporal order.

#include "eskf_core.hpp"
#include "eskf_config.hpp"
#include <stdint.h>

namespace eskf {

// ============================================================
// Buffer Configuration
// ============================================================
// Buffer sizes are defined in eskf_config.hpp:
// - ESKF_IMU_BUFFER_SIZE (3200 = 500ms @ 6.4kHz)
// - ESKF_BARO_BUFFER_SIZE (50 = 500ms @ 100Hz)
// - ESKF_EVENT_BUFFER_SIZE (256 = replay/rewind burst headroom)
// - ESKF_CHECKPOINT_BUFFER_SIZE (25 = 500ms @ 50Hz)
// - ESKF_CHECKPOINT_INTERVAL (128 = 20ms @ 6.4kHz)

#ifndef ESKF_CATCHUP_BUDGET_US
#define ESKF_CATCHUP_BUDGET_US 2000  // 2ms default time budget
#endif

// ============================================================
// Event Types
// ============================================================

/// Types of sparse events that can be processed during replay
enum class EventType : uint8_t {
  GpsPosition,   ///< GPS position correction (deprecated, use GpsPacket)
  GpsVelocity,   ///< GPS velocity correction (deprecated, use GpsPacket)
  GpsPacket,     ///< Combined GPS pos+vel with packet rejection gating
  MagHeading,    ///< Magnetometer heading correction
  LiftoffSnap,   ///< Rail Shadow quaternion injection
  Sideslip       ///< Aerodynamic sideslip constraint
};

// ============================================================
// Ring Buffer Entry Structures
// ============================================================

/// IMU ring buffer entry
struct ImuEntry {
  ImuFrame imu;
  eskf_scalar dt;
};

/// Barometer ring buffer entry (Innovation Transport pattern, Section 3.4.A)
/// 
/// The timestamp is the trigger time (when the app commanded conversion).
/// predicted_alt_at_trigger is the ESKF's altitude estimate at trigger time.
/// When measurement arrives, innovation is: y = alt_m - predicted_alt_at_trigger
/// This innovation is then applied to the CURRENT state (not trigger-time state).
struct BaroEntry {
  eskf_scalar alt_m;                    ///< Measured altitude (filled when ready)
  eskf_scalar R;                        ///< Measurement variance
  uint64_t timestamp_us;                ///< Trigger timestamp
  eskf_scalar predicted_alt_at_trigger; ///< Altitude snapshot for innovation calc
  bool ready;                           ///< True when measurement has been retrieved
  bool dropped;                         ///< True if stale completion was already accounted
};

/// Sparse event buffer entry (GPS, Mag, Liftoff, Sideslip)
struct EventEntry {
  EventType type;
  uint64_t timestamp_us;
  
  union {
    struct { eskf_scalar pos[3]; eskf_scalar R[3]; } gps_pos;
    struct { eskf_scalar vel[3]; eskf_scalar R[3]; eskf_scalar lever_arm[3]; } gps_vel;
    struct {
      eskf_scalar pos[3];         ///< Position in NED (m)
      eskf_scalar R_pos[3];       ///< Position variance per axis (m²)
      eskf_scalar vel[3];         ///< Velocity in NED (m/s)
      eskf_scalar R_vel[3];       ///< Velocity variance per axis ((m/s)²)
      eskf_scalar lever_arm[3];   ///< GPS antenna lever arm from CG (m)
      bool has_position;          ///< True if position data valid
      bool has_velocity;          ///< True if velocity data valid
    } gps_packet;
    struct { 
      eskf_scalar q[4];              ///< Combined quaternion (gravity + heading)
      eskf_scalar b_gyro[3];         ///< Gyro bias from LPF
      eskf_scalar b_acc[3];          ///< Accel bias from preflight turn-on estimator
      eskf_scalar heading_variance;  ///< Uncertainty in yaw (rad²)
      bool heading_initialized;      ///< True if heading is observable
      eskf_scalar ground_altitude_m; ///< Ground ISA altitude for b_baro init
      bool ground_reference_valid;   ///< True if ground calibration complete
    } liftoff;
    struct { eskf_scalar heading; eskf_scalar R; } mag;
    struct { eskf_scalar R_lateral; } sideslip;
  } data;
};

/// Full rewind checkpoint including yieldable-local GNSS state.
/// Core checkpoint alone is insufficient for deterministic replay because
/// packet-level logic also depends on local flags/counters.
struct RewindCheckpoint {
  Checkpoint core;                  ///< Core state/covariance/integration snapshot
  uint64_t timestamp_us = 0;        ///< Convenience copy for rewind search
  uint32_t consecutive_gps_rejects = 0;
  uint32_t imu_since_checkpoint = 0; ///< IMU counter for deterministic checkpoint spacing
  bool has_fused_first_pos = false;
  uint64_t last_gps_position_accept_us = 0;
  eskf_scalar last_accel_magnitude_g = 1.0;
};

// ============================================================
// Liftoff Initialization Data
// ============================================================

/// Complete state data for ESKF initialization at liftoff.
/// Populated from RailShadowCheckpoint at rewind timestamp.
struct LiftoffInitData {
  eskf_scalar q[4] = {1, 0, 0, 0}; ///< Combined quaternion (gravity + heading)
  eskf_scalar b_gyro[3] = {0, 0, 0}; ///< Gyro bias from LPF
  eskf_scalar b_acc[3] = {0, 0, 0}; ///< Accel bias from turn-on estimator
  eskf_scalar heading_variance = 0; ///< Uncertainty in yaw (rad²)
  bool heading_initialized = false; ///< True if heading is observable
  uint64_t liftoff_us = 0;          ///< Liftoff timestamp (for rejection window)
  
  // Ground reference for baro initialization
  eskf_scalar ground_altitude_m = 0; ///< Ground ISA altitude for b_baro init
  bool ground_reference_valid = false; ///< True if ground calibration complete
};

// ============================================================
// Configuration
// ============================================================

/// Configuration for yieldable replay operation
struct YieldableConfig {
  uint32_t catchupBudgetUs = ESKF_CATCHUP_BUDGET_US;
  
  /// GPS measurement delay from PPS (negative = measurement is earlier than PPS)
  /// Applies to BOTH position and velocity. Determined via rocking test calibration.
  /// Typical: -50000 to -200000 µs
  int64_t gpsDelayUs = ESKF_DEFAULT_GPS_DELAY_US;
};

/// Statistics for monitoring buffer usage and rewind operations (I2, I3)
struct RewindStats {
  // Buffer overflow counts
  uint32_t imu_drops = 0;         ///< IMU samples lost to overflow
  uint32_t baro_drops = 0;        ///< Baro samples lost to overflow
  uint32_t event_drops = 0;       ///< Events lost to overflow
  
  // Buffer high-water marks
  size_t imu_max_usage = 0;       ///< Max IMU buffer entries used
  size_t baro_max_usage = 0;      ///< Max Baro buffer entries used
  size_t event_max_usage = 0;     ///< Max Event buffer entries used
  
  // Rewind statistics
  uint32_t rewind_count = 0;            ///< Number of rewinds performed
  uint64_t rewind_total_depth_us = 0;   ///< Sum of rewind depths (for avg calc)
  uint32_t rewind_no_checkpoint_count = 0; ///< Rewinds without checkpoint
  uint32_t rewind_data_gap_count = 0;  ///< Rewinds with data gap warning
  
  // Event ordering
  uint32_t out_of_order_events = 0;  ///< Non-monotonic timestamp events

  // catchUp budget behavior
  uint32_t catchup_budget_yields = 0; ///< Times catchUp yielded on budget
  
  // GPS packet rejection statistics
  uint32_t gps_vel_rejects = 0;       ///< Velocity Chi² failures (entire packet rejected)
  uint32_t gps_pos_rejects = 0;       ///< Position Chi² failures (position only rejected)
  uint32_t gps_covariance_resets = 0; ///< Covariance inflations due to prolonged rejection

  // Stale-entry protection
  uint32_t stale_skips = 0;           ///< Stale ring-buffer entries skipped by catchUp
  
  void reset() { *this = RewindStats{}; }
};

#if APP_TARGET_NATIVE
/// Reset the native time origin (for testing between runs)
void resetTimeOrigin();
#endif

// ============================================================
// Main Class
// ============================================================

/// Unified timestamp-controlled Kalman filter wrapper.
///
/// IMPORTANT: All pushX() methods ONLY buffer data. You MUST call
/// catchUp() to actually process events. This ensures:
/// 1. All sensors are processed in correct temporal order
/// 2. Time budget is always respected
/// 3. GPS rewind works seamlessly
///
/// Usage:
///   1. Call pushImu() / pushBaro() / etc. as sensors arrive (buffering only)
///   2. Call catchUp(now) each loop iteration to process events
///   3. GPS arrival triggers automatic rewind to GPS velocity timestamp
class EskfYieldable {
 public:
  EskfYieldable() = default;

#if APP_TARGET_NATIVE
  using TestNowMicrosFn = uint32_t (*)(void*);
#endif

  /// Initialize with tuning configuration.
  /// Must be called before any sensor input.
  void init(const TuningConfig& tuning_cfg);

  /// Initialize the underlying filter.
  void initialize(const State& initial_state,
                  const InitialCovariance& P0,
                  const ProcessNoise& Q);

  /// Configure time budget and GPS timing.
  void configure(const YieldableConfig& cfg) { cfg_ = cfg; }

#if APP_TARGET_NATIVE
  /// Install a deterministic clock provider for native unit tests.
  void setTestNowMicros(TestNowMicrosFn fn, void* ctx = nullptr) {
    test_now_micros_fn_ = fn;
    test_now_micros_ctx_ = ctx;
  }

  /// Clear deterministic clock provider and return to platform clock.
  void clearTestNowMicros() {
    test_now_micros_fn_ = nullptr;
    test_now_micros_ctx_ = nullptr;
  }
#endif

  // ============================================================
  // Sensor Input (Push to Ring Buffers - NO PROCESSING)
  // ============================================================

  /// Buffer IMU frame for later processing.
  /// Call catchUp() to actually process.
  void pushImu(const ImuFrame& imu, eskf_scalar dt);

  /// Reserve a barometer slot (trigger time known, measurement pending).
  /// Call setBaroMeasurement() when value is retrieved.
  /// Note: Also captures altitude snapshot for Innovation Transport.
  /// @return Slot index to use with setBaroMeasurement()
  size_t reserveBaro(uint64_t trigger_timestamp_us);
  
  /// Set barometer measurement value (called when ADC completes).
  /// Uses Innovation Transport: innovation calculated from trigger-time snapshot.
  /// @param slot Index returned by reserveBaro()
  /// @param alt_m Altitude in meters (positive up)
  /// @param R Measurement variance (m²)
  void setBaroMeasurement(size_t slot, eskf_scalar alt_m, eskf_scalar R);

  /// Trigger barometer sampling - captures altitude snapshot (simpler API).
  /// Use with completeBaro() instead of reserveBaro()/setBaroMeasurement().
  /// @param trigger_timestamp_us Timestamp when conversion was commanded
  void triggerBaro(uint64_t trigger_timestamp_us);
  
  /// Complete barometer measurement using saved snapshot (simpler API).
  /// Uses Innovation Transport and auto-computes R from velocity.
  /// Must call triggerBaro() first.
  /// @param alt_m Altitude in meters (positive up)
  void completeBaro(eskf_scalar alt_m);

  /// Buffer magnetometer heading observation.
  void pushMagHeading(eskf_scalar heading_rad, eskf_scalar R, 
                      uint64_t timestamp_us);

  /// Buffer GPS position observation (uses PPS timestamp).
  /// @param pps_timestamp_us PPS timestamp (position measurement time)
  /// @param pos_ned Position in NED frame (m)
  /// @param R_pos Position variance per axis (m²)
  void pushGpsPosition(uint64_t pps_timestamp_us,
                       const eskf_scalar pos_ned[3],
                       const eskf_scalar R_pos[3]);

  /// Buffer GPS velocity observation (timestamp = PPS + offset from rocking test).
  /// Also triggers rewind to velocity timestamp if needed.
  /// @param pps_timestamp_us PPS timestamp (reference)
  /// @param vel_ned Velocity in NED frame (m/s)
  /// @param R_vel Velocity variance per axis ((m/s)²)
  /// @param lever_arm_body GPS antenna lever arm from CG in body frame (m)
  void pushGpsVelocity(uint64_t pps_timestamp_us,
                       const eskf_scalar vel_ned[3],
                       const eskf_scalar R_vel[3],
                       const eskf_scalar lever_arm_body[3]);

  /// Buffer combined GPS packet (position + velocity) with packet rejection gating.
  /// This is the preferred API that enables Chi-Squared gating before state update.
  /// Velocity failure rejects entire packet; position failure rejects only position.
  /// @param pps_timestamp_us PPS timestamp (reference for timing offset)
  /// @param pos_ned Position in NED frame (m) - can be nullptr if no position
  /// @param R_pos Position variance per axis (m²)
  /// @param vel_ned Velocity in NED frame (m/s) - can be nullptr if no velocity
  /// @param R_vel Velocity variance per axis ((m/s)²)
  /// @param lever_arm_body GPS antenna lever arm from CG in body frame (m)
  void pushGpsPacket(uint64_t pps_timestamp_us,
                     const eskf_scalar* pos_ned, const eskf_scalar* R_pos,
                     const eskf_scalar* vel_ned, const eskf_scalar* R_vel,
                     const eskf_scalar lever_arm_body[3]);

  /// Buffer sideslip constraint observation.
  void pushSideslip(eskf_scalar R_lateral, uint64_t timestamp_us);

  // ============================================================
  // Liftoff Transition
  // ============================================================

  /// Inject Rail Shadow state at liftoff (Phase 3).
  /// This rewinds to the checkpoint timestamp, stores a liftoff snap event,
  /// and fully initializes the ESKF when the event is processed during replay.
  /// @param init_data Complete liftoff state from RailShadowCheckpoint
  /// @param rewind_to_ts Timestamp to rewind to (before actual liftoff)
  void injectLiftoffSnap(const LiftoffInitData& init_data, uint64_t rewind_to_ts);

  // ============================================================
  // Event Processing
  // ============================================================

  /// Process buffered events up to target timestamp.
  /// This is the ONLY place where events are actually processed.
  /// Call this each loop iteration with the current time.
  /// @param target_timestamp_us Target timestamp to catch up to
  /// @param budget_us Time budget in microseconds (0 = use config)
  /// @return true if caught up to target, false if yielded (more work needed)
  bool catchUp(uint64_t target_timestamp_us, uint32_t budget_us = 0);

  /// Check if filter is behind real-time (has buffered events to process).
  bool isBehind(uint64_t now_us) const { 
    return kalman_timestamp_us_ < now_us; 
  }

  /// Get current filter processing timestamp.
  uint64_t kalmanTimestamp() const { return kalman_timestamp_us_; }

  // ============================================================
  // Accessors
  // ============================================================

  EskfCore& core() { return core_; }
  const EskfCore& core() const { return core_; }
  const State& state() const { return core_.state(); }

  /// Get statistics from last catchUp operation.
  uint32_t lastCatchUpDurationUs() const { return last_catchup_duration_us_; }
  uint32_t lastEventsProcessed() const { return last_events_processed_; }
  uint64_t totalEventsProcessed() const { return total_events_processed_; }

  /// Get current buffer depths (for debugging/monitoring).
  size_t imuBufferCount() const {
    return static_cast<size_t>(
        (imu_push_seq_ > ESKF_IMU_BUFFER_SIZE)
            ? ESKF_IMU_BUFFER_SIZE
            : imu_push_seq_);
  }
  size_t baroBufferCount() const { return baro_count_; }
  size_t eventBufferCount() const { return event_count_; }
  /// Pending (unprocessed) entries in IMU ring buffer.
  size_t imuReadIdx() const {
    return (imu_push_seq_ > imu_read_seq_)
               ? static_cast<size_t>(imu_push_seq_ - imu_read_seq_)
               : 0u;
  }

  /// Raw sequence counters and kalman time (for liftoff diagnostics).
  uint64_t imuPushSeq() const { return imu_push_seq_; }
  uint64_t imuReadSeq() const { return imu_read_seq_; }
  uint64_t kalmanTimestampUs() const { return kalman_timestamp_us_; }
  uint64_t imuBufferTimestamp(size_t slot) const {
    return imu_buffer_[slot].imu.timestamp_us;
  }
  
  /// Check if filter is in hibernation (pre-liftoff) mode.
  /// When hibernating, the ESKF is not running - only buffers are filling.
  bool isHibernating() const { return hibernating_; }
  
  /// Get rewind/buffer statistics (I2, I3)
  const RewindStats& stats() const { return stats_; }
  void resetStats() { stats_.reset(); }

  // ============================================================
  // Logging Helpers
  // ============================================================

  /// Log current state (rate-limited by logger's internal limiter).
  /// Call this periodically (e.g., after catchUp) to log state snapshots.
  void logStateIfDue();

  /// Log current covariance (rate-limited by logger's internal limiter).
  /// Call this periodically (e.g., after catchUp) to log covariance snapshots.
  void logCovarianceIfDue();

 private:
  EskfCore core_;
  YieldableConfig cfg_;
  TuningConfig tuning_cfg_;  // Runtime algorithm tuning parameters
  
  // Filter's current processing timestamp
  uint64_t kalman_timestamp_us_ = 0;
  
  // --- IMU Ring Buffer (sequence-number based) ---
  ImuEntry imu_buffer_[ESKF_IMU_BUFFER_SIZE];
  uint64_t imu_push_seq_ = 0;  // Monotonic push counter (slot = seq % SIZE)
  uint64_t imu_read_seq_ = 0;  // Next sequence to consume in catchUp
  
  // --- Baro Ring Buffer ---
  BaroEntry baro_buffer_[ESKF_BARO_BUFFER_SIZE];
  size_t baro_head_ = 0;
  size_t baro_count_ = 0;
  size_t baro_read_idx_ = 0;
  
  // --- Event Ring Buffer (GPS, Mag, Liftoff, Sideslip) ---
  EventEntry event_buffer_[ESKF_EVENT_BUFFER_SIZE];
  size_t event_head_ = 0;
  size_t event_count_ = 0;
  size_t event_read_idx_ = 0;
  
  // --- Checkpoint for Rewind ---
  RewindCheckpoint oldest_checkpoint_;     ///< Fallback if periodic buffer doesn't go far enough
  bool has_checkpoint_ = false;
  
  // --- Periodic Checkpoints (for efficient rewind) ---
  RewindCheckpoint checkpoint_buffer_[ESKF_CHECKPOINT_BUFFER_SIZE];
  size_t checkpoint_head_ = 0;
  size_t checkpoint_count_ = 0;
  uint32_t imu_since_checkpoint_ = 0;      ///< IMU samples since last checkpoint
  
  // --- Statistics ---
  uint32_t last_catchup_duration_us_ = 0;
  uint32_t last_events_processed_ = 0;
  uint64_t total_events_processed_ = 0;
  RewindStats stats_;  ///< Monitoring statistics (I2, I3)
  
  // --- Pending Baro Trigger (for simpler triggerBaro/completeBaro API) ---
  size_t pending_baro_slot_ = 0;
  bool has_pending_baro_ = false;
  
  // --- Rewind state tracking (m3) ---
  bool in_rewind_ = false;
  eskf_scalar pending_rewind_gap_dt_s_ = 0;  ///< Added once to first IMU step after data-gap rewind
  uint32_t rewind_imu_replayed_ = 0;
  uint32_t rewind_baro_replayed_ = 0;
  uint32_t rewind_events_replayed_ = 0;
  uint64_t rewind_trigger_timestamp_us_ = 0;
  uint64_t rewind_origin_timestamp_us_ = 0;
  uint64_t rewind_checkpoint_timestamp_us_ = 0;
  uint32_t pending_rewind_gap_dt_us_ = 0;
  
  // --- Event timestamp validation (m4) ---
  uint64_t last_event_timestamp_ = 0;

  // --- Liftoff event survivability reserve ---
  bool liftoff_snap_pending_ = false;
  
  // --- Aiding Sensor Rejection Window (Phase 4) ---
  uint64_t rejection_end_us_ = 0;  ///< Reject GPS/baro/mag until this timestamp
  
  // --- GPS Packet Rejection State ---
  uint32_t consecutive_gps_rejects_ = 0;  ///< Consecutive velocity rejections
  
  // --- First GPS Position Reset Flag ---
  // After heading alignment, the first GPS position is used to reset (teleport)
  // the position state to clear quadratic integration error from heading drift.
  bool has_fused_first_pos_ = false;
  uint64_t last_gps_position_accept_us_ = 0;
  
  // --- High-G GPS Rejection State ---
  // Track last acceleration magnitude (in g) for GPS R inflation
  eskf_scalar last_accel_magnitude_g_ = 1.0;
  
  // --- Buffer Overflow Logging Rate Limiter ---
  uint64_t last_imu_overflow_log_us_ = 0;       ///< Rate limit overflow events to 1Hz
  uint64_t last_baro_overflow_log_us_ = 0;
  uint64_t last_event_overflow_log_us_ = 0;
  
  // --- Pre-Liftoff Hibernation Mode ---
  /// When true, ESKF is not running (pre-liftoff). Only RailShadowFilter is active.
  /// Buffers fill for liftoff rewind, but no ESKF checkpoints are created.
  /// Cleared by injectLiftoffSnap() to begin normal operation.
  bool hibernating_ = true;

#if APP_TARGET_NATIVE
  // Optional test-only clock hook for deterministic catchUp budget tests.
  TestNowMicrosFn test_now_micros_fn_ = nullptr;
  void* test_now_micros_ctx_ = nullptr;
#endif
  
  // ============================================================
  // Internal Methods
  // ============================================================
  
  uint32_t nowMicros() const;
  void rewindTo(uint64_t timestamp_us, bool liftoff_rewind = false);
  
  template<typename T, size_t N>
  size_t binarySearchBuffer(const T* buffer, size_t count, size_t head,
                            uint64_t timestamp_us) const;
  
  static uint64_t getTimestamp(const ImuEntry& e) { return e.imu.timestamp_us; }
  static uint64_t getTimestamp(const BaroEntry& e) { return e.timestamp_us; }
  static uint64_t getTimestamp(const EventEntry& e) { return e.timestamp_us; }

  size_t selectNextEventOffset() const;
  
  uint64_t peekNextImuTimestamp() const;
  uint64_t peekNextBaroTimestamp() const;
  uint64_t peekNextEventTimestamp() const;
  EventType peekNextEventType() const;
  
  void processNextImu();
  void processNextBaro();
  void processNextEvent();
  
  void pushEvent(const EventEntry& e);
  uint64_t gpsEventTimestampFromPps(uint64_t pps_timestamp_us) const;
  void logGpsDroppedByRejectionWindow(uint64_t gps_timestamp_us);
  void logGpsDroppedByHistoryHorizon(uint64_t gps_timestamp_us,
                                     uint64_t oldest_imu_timestamp_us);
  void logGpsDroppedByCheckpointHorizon(uint64_t gps_timestamp_us,
                                        uint64_t required_min_timestamp_us);
  bool isGpsTimestampOlderThanRetainedImuHistory(
      uint64_t gps_timestamp_us,
      uint64_t *oldest_imu_timestamp_us = nullptr) const;
  bool isGpsTimestampOutsideRewindCheckpointHorizon(
      uint64_t gps_timestamp_us,
      uint64_t oldest_imu_timestamp_us,
      uint64_t *required_min_timestamp_us = nullptr) const;
  void discardStalePendingBaro();
  void flushUnreadBaroAfterStateReset(uint64_t context_ts);
  void saveCheckpointNow();

  RewindCheckpoint captureRewindCheckpoint() const;
  void restoreRewindCheckpoint(const RewindCheckpoint& cp);
  
  /// Compute averaged lever arm velocity in NED frame over the window
  /// @param lever_arm_body GPS antenna lever arm in body frame (m)
  /// @param gps_timestamp_us GPS measurement timestamp
  /// @param window_us Duration of averaging window (microseconds)
  /// @param avg_lever_vel_ned Output averaged lever arm velocity in NED (m/s)
  /// @return true if averaging was successful (enough samples), false to use fallback
  bool computeAveragedLeverArmVelocity(
      const eskf_scalar lever_arm_body[3],
      uint64_t gps_timestamp_us,
      uint64_t window_us,
      eskf_scalar avg_lever_vel_ned[3]) const;
  
  template<size_t N>
  size_t oldestIndex(size_t count, size_t head) const {
    return (count < N) ? 0 : head;
  }
  
  template<size_t N>
  size_t bufferIndex(size_t offset, size_t count, size_t head) const {
    size_t oldest = oldestIndex<N>(count, head);
    return (oldest + offset) % N;
  }
};

} // namespace eskf
