#pragma once
// ESKF-Based State Estimator
// Application-level wrapper integrating the Kalman filter subsystem

#include "Application/Kalman/kalman/target_platform.hpp"

#if APP_TARGET_NATIVE || APP_TARGET_STM32

#include "Application/Kalman/AppLayer/state_estimator.hpp"
#include "Application/Kalman/AppLayer/eskf_app_config.hpp"
#include "Application/Kalman/AppLayer/log_sink.hpp"
#include "Application/Kalman/AppLayer/replayable_config.hpp"
#include "Application/Kalman/AppLayer/calibration_config.hpp"

#include "Application/Kalman/kalman/eskf_yieldable.hpp"
#include "Application/Kalman/kalman/shadow_filter.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_imu.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_baro.hpp"
#include "Application/Kalman/kalman/preprocessor/virtual_compass.hpp"
#include "Application/Kalman/kalman/preprocessor/pressure_altitude.hpp"
#include "Application/Kalman/kalman/descent_nav_filter.hpp"
#include <vector>

namespace app {

/// ESKF-based state estimator
///
/// Integrates the Kalman filter subsystem with the application layer:
/// - VirtualImu for multi-sensor IMU fusion
/// - VirtualBaro for barometer fusion
/// - VirtualCompass for magnetometer fusion
/// - EskfYieldable for GPS rewind and time-budgeted processing
/// - Shadow filters for pre-flight and apogee detection backup
class EskfEstimator : public IStateEstimator {
public:
  /// Configuration for the ESKF estimator
  struct Config : public appcfg::ReplayableConfig {
    /// Catchup time budget per tick (microseconds)
    uint32_t catchup_budget_us = ESKF_APP_CATCHUP_BUDGET_US;

    // Inherits launch_latitude_deg and all other algorithm config from ReplayableConfig
  };

  explicit EskfEstimator(hal::ILogSink* log = nullptr);
  ~EskfEstimator() override = default;

  /// Configure the estimator (call before first use)
  void configure(const Config& cfg);

  /// Configure calibration data (call before first use, after configure())
  void configureCalibration(const appcfg::CalibrationConfig& cfg);

  /// Native replay override for active synthetic sensor counts.
  /// Leaves STM32 runtime behavior unchanged when not used.
  void configureReplaySensorCounts(size_t imu_count, size_t baro_count);

  // --- IStateEstimator interface ---
  void reset() override;
  void onLiftoff(uint32_t liftoff_ms) override;
  void processImuBatch(const ImuBatch& batch) override;
  void processBaroBatch(const BaroBatch& batch) override;

  /// Command mode barometer: call when triggering conversion to save ESKF snapshot
  void onBaroTrigger(uint64_t trigger_timestamp_us);

  /// Command mode barometer: process single sample with Innovation Transport
  void processBaroSample(const BaroSample& sample);

  EstimatorOutput output() const override;

  // --- Extended interface ---
  void processGpsSample(const sensors::gnss::GnssSample& sample) override;
  void processMagSample(const sensors::mmc5983ma::MmcSample& sample) override;
  void onTick(uint64_t now_us) override;
  void onFlightStateChange(flight_computer::State state,
                           uint8_t reason = 0) override;

  // --- Accessors for advanced usage ---
  const eskf::EskfYieldable& filter() const { return filter_; }
  const eskf::RailShadowFilter& railShadow() const { return rail_shadow_; }
  const eskf::FlightShadowFilter& flightShadow() const { return flight_shadow_; }

  /// Get apogee detection output from flight shadow filter
  /// @return True if apogee detected (velocity >= 0 in NED Down)
  bool isApogeeDetected() const;

  // --- Apogee Detection Support (for ConsensusApogeeDetector) ---

  /// Get ESKF vertical velocity (NED Down, positive = falling)
  eskf_scalar eskfVelocityDown() const;

  /// Get Shadow Filter vertical velocity (NED Down, positive = falling)
  eskf_scalar shadowVelocityDown() const;

  /// Check if ESKF has diverged (NaN, infinite values, or NIS failure)
  bool isEskfDiverged() const;

  /// Check if in coast phase (past burnout - body-X acceleration < 0)
  bool isCoastPhase() const;

  /// Get body-frame X-axis acceleration (for high-accel lockout check)
  eskf_scalar bodyAccelX() const;

  /// Check if in-flight (liftoff has occurred)
  bool inFlight() const { return in_flight_; }

  /// Rewind/buffer stats from EskfYieldable.
  const eskf::RewindStats &rewindStats() const { return filter_.stats(); }

  /// Total number of catchUp budget yields since last reset.
  uint32_t catchupYieldCount() const { return catchup_yield_count_; }

  /// Last catchUp execution duration (us).
  uint32_t lastCatchupDurationUs() const { return filter_.lastCatchUpDurationUs(); }

  /// Configured catchUp budget (us) used for budgeted onTick processing.
  uint32_t catchupBudgetUs() const { return cfg_.catchup_budget_us; }

  /// Last catchUp processed event count.
  uint32_t lastCatchupEventsProcessed() const {
    return filter_.lastEventsProcessed();
  }

  /// ESKF numerical-health diagnostics.
  eskf_scalar eskfLastNis() const { return filter_.core().lastNIS(); }
  uint8_t eskfConsecutiveHighNisCount() const {
    return filter_.core().consecutiveHighNisCount();
  }
  uint32_t predictDtClampedCount() const {
    return filter_.core().predictDtClampedCount();
  }

  struct LiftoffSnapshot {
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float b_gyro[3] = {0.0f, 0.0f, 0.0f};
    float b_acc[3] = {0.0f, 0.0f, 0.0f};
    float heading_variance = 0.0f;
    float ground_pressure_pa = 0.0f;
    float ground_temperature_k = 0.0f;
    float ground_isa_altitude_m = 0.0f;
    float ground_altitude_m = 0.0f;
    float baro_offsets_pa[ESKF_MAX_BAROS] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool heading_initialized = false;
    bool ground_reference_valid = false;
    bool used_ground_ref_fallback = false;
    uint64_t timestamp_us = 0;
  };

  bool getLiftoffSnapshot(LiftoffSnapshot &out) const;

  /// Get current descent filter state when descent mode is active.
  /// @return true when descent state is available.
  bool getDescentState(DescentNavFilter::State &out) const;

  /// Get descent filter update/reject/clamp counters.
  /// @return true when descent stats are available.
  bool getDescentStats(DescentNavFilter::Stats &out) const;


private:
  Config cfg_{};
  appcfg::CalibrationConfig calib_cfg_{};
  bool calib_cfg_set_ = false;  ///< True if configureCalibration() was called
  hal::ILogSink* log_ = nullptr;

  // Kalman filter components
  eskf::EskfYieldable filter_;
  eskf::VirtualImu virtual_imu_;
  eskf::VirtualBaro virtual_baro_;
  eskf::VirtualCompass virtual_compass_;

  // Shadow filters
  eskf::RailShadowFilter rail_shadow_;
  eskf::FlightShadowFilter flight_shadow_;

  // Internal state
  bool initialized_ = false;
  bool in_flight_ = false;
  bool rail_shadow_initialized_ = false;  ///< True after first accel init
  uint32_t liftoff_ms_ = 0;
  uint64_t liftoff_us_ = 0;

  // GPS origin for NED conversion (set on first valid GPS fix)
  bool gps_origin_set_ = false;
  double gps_origin_lat_ = 0;  // radians
  double gps_origin_lon_ = 0;  // radians
  float gps_origin_alt_ = 0;   // meters MSL

  struct PreflightGpsAnchor {
    bool valid = false;
    uint64_t timestamp_us = 0;
    double lat_rad = 0;
    double lon_rad = 0;
    float alt_m = 0;
  };
  PreflightGpsAnchor latest_preflight_gps_anchor_;

  // Cached output
  mutable EstimatorOutput cached_output_{};
  mutable bool output_dirty_ = true;

  // Descent-only baro+GNSS estimator (independent of IMU/mag updates).
  DescentNavFilter descent_filter_;
  bool descent_mode_active_ = false;

  struct LatestGnssForDescent {
    bool valid = false;
    uint64_t timestamp_us = 0;
    float pos_ned[3] = {0.0f, 0.0f, 0.0f};
    float vel_ned[3] = {0.0f, 0.0f, 0.0f};
    float var_pos_ned[3] = {0.0f, 0.0f, 0.0f};
    float var_vel_ned[3] = {0.0f, 0.0f, 0.0f};
  };
  LatestGnssForDescent latest_descent_gnss_{};

  struct LatestBaroForDescent {
    bool valid = false;
    uint64_t timestamp_us = 0;
    float down_position_m = 0.0f;
    float variance = 0.0f;
  };
  LatestBaroForDescent latest_descent_baro_{};
  bool descent_waiting_initial_gnss_snap_ = false;
  uint64_t descent_last_gnss_fuse_us_ = 0;

  // Internal buffer for VirtualImu output
  static constexpr size_t kVirtualImuBufferSize = 64;
  eskf::ImuFrame imu_buffer_[kVirtualImuBufferSize];

  // Last body-frame acceleration (for apogee detection lockout checks)
  mutable eskf_scalar last_body_accel_x_ = 0;

  // Checkpoint timing for RailShadow (pre-liftoff)
  uint64_t last_rail_checkpoint_us_ = 0;

  // Ground reference for barometer (set at liftoff from RailShadow)
  eskf::GroundReference ground_reference_;
  float ground_isa_altitude_ = 0; ///< ISA altitude of launch pad (m MSL), for FlightShadow AGL

  bool was_aero_blind_ = false;
  bool baro_reacquire_needed_ = false;
  static constexpr uint64_t kFlightShadowImuOutageFallbackUs = 120000;
  static constexpr uint64_t kSustainedImuOutageHandoffUs = 1000000;
  uint64_t last_flight_shadow_predict_us_ = 0;
  uint64_t last_valid_inflight_imu_us_ = 0;
  bool sustained_imu_outage_handoff_todo_logged_ = false;
  uint32_t catchup_yield_count_ = 0;
  uint16_t sideslip_decimation_counter_ = 0;  ///< Counts IMU steps for sideslip decimation

  // Single-sensor stale/frozen health tracking (GNSS and magnetometer).
  eskf::SensorHealthState mag_health_state_ = eskf::SensorHealthState::ACTIVE;
  uint16_t mag_hard_fault_counter_ = 0;
  uint16_t mag_healthy_counter_ = 0;
  uint16_t mag_stale_counter_ = 0;
  bool mag_has_prev_sample_ = false;
  uint32_t mag_prev_x_ = 0;
  uint32_t mag_prev_y_ = 0;
  uint32_t mag_prev_z_ = 0;

  eskf::SensorHealthState gnss_health_state_ = eskf::SensorHealthState::ACTIVE;
  uint16_t gnss_hard_fault_counter_ = 0;
  uint16_t gnss_healthy_counter_ = 0;
  uint16_t gnss_stale_counter_ = 0;
  bool gnss_has_prev_sample_ = false;
  sensors::gnss::GnssSample gnss_prev_sample_{};
  bool gnss_fix_logged_init_ = false;
  bool last_logged_gnss_fix_usable_ = false;
  uint64_t last_gnss_fix_drop_log_us_ = 0;

  // Logged health-state edge tracking.
  bool imu_health_logged_init_[ESKF_MAX_IMUS] = {false};
  eskf::SensorHealthState last_logged_imu_health_[ESKF_MAX_IMUS] = {};
  bool baro_health_logged_init_[ESKF_MAX_BAROS] = {false};
  eskf::BaroHealthState last_logged_baro_health_[ESKF_MAX_BAROS] = {};
  bool last_logged_imu_salvage_ = false;
  bool last_logged_baro_salvage_ = false;

  LiftoffSnapshot last_liftoff_snapshot_{};
  bool has_liftoff_snapshot_ = false;

  // Pre-liftoff accelerometer turn-on bias estimator.
  // Estimates only the observable component along gravity from norm mismatch.
  eskf_scalar turn_on_accel_bias_[3] = {0, 0, 0};
  uint32_t turn_on_accel_bias_samples_ = 0;
  bool turn_on_accel_bias_valid_ = false;

  // IMU batch synchronization state
  // Buffers pending batches from each source until pairing is possible
  static constexpr size_t kMaxImuSources = ESKF_MAX_IMUS;
  static constexpr uint32_t kImuSyncToleranceUs = 500;   // Max timestamp diff for pairing (~half sample period)
  static constexpr uint32_t kImuBatchTimeoutUs = 5000;   // Flush unpaired batch after this delay

  struct PendingImuBatch {
    std::vector<ImuSample> samples;  // Deep copy of batch samples
    uint64_t t0_us = 0;
    uint32_t dt_us = 0;
    uint8_t source = 0;
    bool valid = false;
  };
  PendingImuBatch pending_imu_[kMaxImuSources];

  // Barometer synchronization state for native replay multi-baro ingestion.
  static constexpr size_t kMaxBaroSources = ESKF_MAX_BAROS;
  static constexpr uint32_t kBaroSyncToleranceUs = 3000;
  static constexpr uint32_t kBaroBatchTimeoutUs = 50000;

  struct PendingBaroSample {
    eskf_sensor_t pressure_pa = 0;
    eskf_sensor_t temperature_k = 0;
    eskf::SensorStatus status = eskf::SensorStatus::HARD_FAIL;
    uint64_t timestamp_us = 0;
    bool valid = false;
  };
  PendingBaroSample pending_baro_[kMaxBaroSources];
  /// Per-source wall-clock timestamp of last received baro sample.
  /// Used by processBaroObservation to detect which sources are alive
  /// and skip dead sources in the multi-baro batch readiness check.
  uint64_t baro_source_last_seen_us_[kMaxBaroSources] = {};
  /// A baro source is considered "alive" if it reported within this window.
  static constexpr uint64_t kBaroSourceAliveTimeoutUs = 2000000; // 2 seconds
  size_t active_imu_sources_ = 2;
  size_t active_baro_sources_ = 1;

  // Private helpers
  void initializeFilter();
  void setupVirtualImu();
  void setupVirtualBaro();
  void setupVirtualCompass();
  void enterDescentMode(uint64_t timestamp_us);
  static void virtualImuCgCallback(void* user_data,
                                   uint64_t timestamp_us,
                                   eskf_scalar cg_out[3]);
  void interpolateCgAtTimestamp(uint64_t timestamp_us,
                                eskf_scalar cg_out[3]) const;
  void convertGpsToNed(const sensors::gnss::GnssSample& sample,
                       eskf_scalar pos_ned[3],
                       eskf_scalar vel_ned[3]) const;
  void updateCachedOutput() const;

  // IMU synchronization helpers
  void processSyncedImuBatches(const PendingImuBatch& batch0, const PendingImuBatch& batch1);
  void processSyncedImuGroup(const PendingImuBatch* const* group, size_t group_count);
  void processBufferedImuBatch(const PendingImuBatch& batch);
  void flushPendingImuIfStale(uint64_t current_t0_us);
  void updateInFlightImuOutageState(const eskf::VirtualImuOutput& vout);
  void processBaroObservation(uint8_t source, eskf_sensor_t pressure_pa,
                              eskf_sensor_t temperature_k,
                              uint64_t timestamp_us, float dt_s,
                              bool trigger_before_complete);
  void flushPendingBaroIfStale(uint64_t current_ts_us);
  bool shouldStartEskfBaroFusion(uint64_t timestamp_us);
  void performBaroReacquisition(eskf_scalar altitude_isa_m, uint64_t timestamp_us);
  void resetTurnOnAccelBiasEstimator();
  void resetAuxSensorStaleEstimators();
  static bool updateSingleSensorHealthFromHardObservation(
      bool hard_observation, eskf::SensorHealthState &state,
      uint16_t &hard_fault_counter, uint16_t &healthy_counter,
      uint16_t hard_fault_suspect_samples,
      uint16_t hard_fault_persistence_samples,
      uint16_t recovery_cooldown_samples,
      uint16_t recovery_confirm_samples);
  void updateTurnOnAccelBiasEstimate(const eskf_scalar nav_accel[3],
                                     const eskf_scalar nav_gyro[3],
                                     uint64_t sample_timestamp_us);

  // Shadow filter logging helpers
  void logRailShadowIfDue(uint64_t timestamp_us);
  void logFlightShadowIfDue(uint64_t timestamp_us);
  void logImuPipelineIfDue(const eskf::VirtualImuOutput &vout,
                           eskf_scalar dt_s);
  void logImuHealthTransitions(const eskf::VirtualImuOutput &vout);
  void logBaroHealthTransitions(const eskf::BaroOutput &bout);
};

}  // namespace app

#endif  // APP_TARGET_NATIVE || APP_TARGET_STM32
