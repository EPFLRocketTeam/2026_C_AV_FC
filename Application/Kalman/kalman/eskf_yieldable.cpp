// ESKF Yieldable Wrapper Implementation
// Unified timestamp-controlled replay for GPS rewind.
//
// Key design: All pushX() methods only buffer data. Processing
// happens exclusively in catchUp().

#include "eskf_yieldable.hpp"
#include "eskf_logger.hpp"
#include "eskf_math.hpp"
#include <cstring>
#include <algorithm>
#include <cstdio>

#if APP_TARGET_STM32
extern "C" {
#include "Application/app_timebase.h"
}
#elif APP_TARGET_NATIVE
#include <chrono>
#endif

namespace eskf {

// ============================================================
// Platform-Specific Time
// ============================================================

#if APP_TARGET_NATIVE
namespace {
  std::chrono::steady_clock::time_point g_time_origin;
  bool g_time_origin_set = false;
}

void resetTimeOrigin() {
  g_time_origin = std::chrono::steady_clock::now();
  g_time_origin_set = true;
}
#endif

uint32_t EskfYieldable::nowMicros() const {
#if APP_TARGET_STM32
  // Keep uint32 API parity with existing yieldable contracts; wrap is expected.
  return static_cast<uint32_t>(app_timebase_now_us());
#elif APP_TARGET_NATIVE
  if (test_now_micros_fn_) {
    return test_now_micros_fn_(test_now_micros_ctx_);
  }
  using namespace std::chrono;
  if (!g_time_origin_set) {
    g_time_origin = steady_clock::now();
    g_time_origin_set = true;
  }
  return static_cast<uint32_t>(
      duration_cast<microseconds>(steady_clock::now() - g_time_origin).count());
#else
  return 0;
#endif
}

// ============================================================
// Initialization
// ============================================================

void EskfYieldable::init(const TuningConfig& tuning_cfg) {
  tuning_cfg_ = tuning_cfg;
  core_.init(tuning_cfg);
  cfg_.gpsDelayUs = tuning_cfg.gps_delay_us;
}

void EskfYieldable::initialize(const State& initial_state,
                                const InitialCovariance& P0,
                                const ProcessNoise& Q) {
  core_.initialize(initial_state, P0, Q);
  
  // Reset all buffers
  imu_push_seq_ = 0;
  imu_read_seq_ = 0;
  
  baro_head_ = 0;
  baro_count_ = 0;
  baro_read_idx_ = 0;
  
  event_head_ = 0;
  event_count_ = 0;
  event_read_idx_ = 0;
  
  // Reset checkpoint state
  kalman_timestamp_us_ = initial_state.timestamp_us;
  has_checkpoint_ = false;
  checkpoint_head_ = 0;
  checkpoint_count_ = 0;
  imu_since_checkpoint_ = 0;
  
  // (M3) Initialize oldest_checkpoint_ immediately as fallback
  oldest_checkpoint_ = captureRewindCheckpoint();
  has_checkpoint_ = true;
  
  // Reset pending baro state
  has_pending_baro_ = false;
  
  // Reset tracking state
  in_rewind_ = false;
  pending_rewind_gap_dt_s_ = 0;
  pending_rewind_gap_dt_us_ = 0;
  rewind_imu_replayed_ = 0;
  rewind_baro_replayed_ = 0;
  rewind_events_replayed_ = 0;
  rewind_trigger_timestamp_us_ = 0;
  rewind_origin_timestamp_us_ = 0;
  rewind_checkpoint_timestamp_us_ = 0;
  last_event_timestamp_ = 0;
  liftoff_snap_pending_ = false;
  consecutive_gps_rejects_ = 0;
  has_fused_first_pos_ = false;
  last_gps_position_accept_us_ = 0;
  last_accel_magnitude_g_ = 1.0;
  // Before liftoff, do not enqueue GPS/mag aiding packets. Liftoff transition
  // sets the real rejection window end.
  rejection_end_us_ = UINT64_MAX;
  
  last_catchup_duration_us_ = 0;
  last_events_processed_ = 0;
  
  // Don't reset stats_ here - caller can use resetStats() if desired
}

// ============================================================
// Sensor Input - Push to Ring Buffers (NO PROCESSING)
// ============================================================

void EskfYieldable::pushImu(const ImuFrame& imu, eskf_scalar dt) {
#if ESKF_USE_GPS_REWIND
  // (M1) Store in ring buffer (BUFFER ONLY - no processing here)
  const size_t slot = imu_push_seq_ % ESKF_IMU_BUFFER_SIZE;
  imu_buffer_[slot].imu = imu;
  imu_buffer_[slot].dt = dt;
  imu_push_seq_++;

  // Detect overwrite of unread entries.
  // If push_seq - read_seq > BUFFER_SIZE, the oldest unread entry was
  // overwritten.  Advance read_seq to the oldest surviving entry.
  const uint64_t pending = imu_push_seq_ - imu_read_seq_;
  if (pending > ESKF_IMU_BUFFER_SIZE) {
    const uint64_t lost = pending - ESKF_IMU_BUFFER_SIZE;
    stats_.imu_drops += static_cast<uint32_t>(lost);
    imu_read_seq_ = imu_push_seq_ - ESKF_IMU_BUFFER_SIZE;

    // Rate-limit overflow logging to 1 Hz to avoid log spam
    constexpr uint64_t kOverflowLogIntervalUs = 1000000;
    if (imu.timestamp_us - last_imu_overflow_log_us_ >= kOverflowLogIntervalUs) {
      last_imu_overflow_log_us_ = imu.timestamp_us;
      getEskfLogger().logEvent(EskfEventType::ImuBufferOverflow,
                               imu.timestamp_us,
                               static_cast<float>(stats_.imu_drops));
    }

    // (M1) Data lost while caught up - update oldest checkpoint
    oldest_checkpoint_ = captureRewindCheckpoint();
  }
  
  // (I2) Track high-water mark
  {
    const size_t count = (imu_push_seq_ > ESKF_IMU_BUFFER_SIZE)
                             ? ESKF_IMU_BUFFER_SIZE
                             : static_cast<size_t>(imu_push_seq_);
    if (count > stats_.imu_max_usage) {
      stats_.imu_max_usage = count;
    }
  }
  
  // Periodic checkpoints are created while replaying IMU entries in catchUp(),
  // where state and timestamp are guaranteed consistent.
#else
  // Without GPS rewind, process immediately
  core_.predict(imu, dt);
  kalman_timestamp_us_ = imu.timestamp_us;
#endif
}

size_t EskfYieldable::reserveBaro(uint64_t trigger_timestamp_us) {
#if ESKF_USE_GPS_REWIND
  size_t slot = baro_head_;
  
  baro_buffer_[slot].timestamp_us = trigger_timestamp_us;
  baro_buffer_[slot].ready = false;
  baro_buffer_[slot].dropped = false;
  baro_buffer_[slot].alt_m = 0;
  baro_buffer_[slot].R = 1.0;
  
  // Innovation Transport (Section 3.4.A): Snapshot predicted altitude at trigger time
  // This is used to calculate innovation: y = Meas_new - predicted_alt_at_trigger
  // The innovation is then applied to the CURRENT state, not the trigger-time state.
  const State& s = core_.state();
  baro_buffer_[slot].predicted_alt_at_trigger = -s.p[2] + s.b_baro;  // NED: alt = -p_D + bias
  
  baro_head_ = (baro_head_ + 1) % ESKF_BARO_BUFFER_SIZE;
  if (baro_count_ < ESKF_BARO_BUFFER_SIZE) {
    baro_count_++;
  } else {
    if (hibernating_ || in_rewind_) {
      // Pre-liftoff hibernation and rewind both overwrite already-consumed
      // entries by design; don't count or log as overflow.
    } else if (baro_read_idx_ > 0) {
      // Overwriting an already-processed entry - normal ring buffer operation.
      // Keep read index relative to the new oldest entry.
      baro_read_idx_--;
    } else {
      stats_.baro_drops++;  // (I2) Track overflow
      // Rate-limit overflow logging to 1 Hz
      constexpr uint64_t kOverflowLogIntervalUs = 1000000;
      if (trigger_timestamp_us - last_baro_overflow_log_us_ >=
          kOverflowLogIntervalUs) {
        last_baro_overflow_log_us_ = trigger_timestamp_us;
        getEskfLogger().logEvent(EskfEventType::BaroBufferOverflow,
                                 trigger_timestamp_us,
                                 static_cast<float>(stats_.baro_drops));
      }
    }
  }
  
  // (I2) Track high-water mark
  if (baro_count_ > stats_.baro_max_usage) {
    stats_.baro_max_usage = baro_count_;
  }
  
  return slot;
#else
  (void)trigger_timestamp_us;
  return 0;
#endif
}

void EskfYieldable::setBaroMeasurement(size_t slot, eskf_scalar alt_m, 
                                        eskf_scalar R) {
#if ESKF_USE_GPS_REWIND
  if (slot < ESKF_BARO_BUFFER_SIZE) {
    baro_buffer_[slot].alt_m = alt_m;
    baro_buffer_[slot].R = R;
    baro_buffer_[slot].ready = true;
    baro_buffer_[slot].dropped = false;
  }
#else
  (void)slot;
  core_.correctBaroAltitude(alt_m, R);
#endif
}

void EskfYieldable::triggerBaro(uint64_t trigger_timestamp_us) {
  // Simpler API: capture snapshot and store slot internally
  pending_baro_slot_ = reserveBaro(trigger_timestamp_us);
  has_pending_baro_ = true;
}

void EskfYieldable::completeBaro(eskf_scalar alt_m) {
  if (!has_pending_baro_) {
    // No trigger was issued - can't use Innovation Transport
    return;
  }

  const uint64_t trigger_ts = baro_buffer_[pending_baro_slot_].timestamp_us;
  if (tuning_cfg_.baro_snapshot_max_age_us > 0 &&
      kalman_timestamp_us_ > trigger_ts) {
    const uint64_t age_us = kalman_timestamp_us_ - trigger_ts;
    if (age_us > tuning_cfg_.baro_snapshot_max_age_us) {
      has_pending_baro_ = false;
      baro_buffer_[pending_baro_slot_].dropped = true;
      stats_.baro_drops++;
      getEskfLogger().logEvent(EskfEventType::BaroSnapshotDropped,
                               trigger_ts,
                               static_cast<float>(age_us));
      return;
    }
  }
  
  // (m1) Compute auto-variance and mark slot as ready for catchUp processing
  // This ensures baro goes through the unified buffered flow
  const State& s = core_.state();
  eskf_scalar v_sq = s.v[0] * s.v[0] + s.v[1] * s.v[1] + s.v[2] * s.v[2];
  eskf_scalar speed = std::sqrt(v_sq);
  eskf_scalar sigma = tuning_cfg_.baro_sigma_base + tuning_cfg_.baro_k_aero * v_sq;
  if (speed > tuning_cfg_.baro_transonic_low && speed < tuning_cfg_.baro_transonic_high) {
    sigma += tuning_cfg_.baro_transonic_penalty;
  }
  eskf_scalar R = sigma * sigma;
  
  // Mark the slot as ready with measurement - catchUp() will process it
  setBaroMeasurement(pending_baro_slot_, alt_m, R);
  has_pending_baro_ = false;
}

void EskfYieldable::pushMagHeading(eskf_scalar heading_rad, eskf_scalar R,
                                    uint64_t timestamp_us) {
#if ESKF_USE_GPS_REWIND
  EventEntry e;
  e.type = EventType::MagHeading;
  e.timestamp_us = timestamp_us;
  e.data.mag.heading = heading_rad;
  e.data.mag.R = R;
  pushEvent(e);
#else
  (void)timestamp_us;
  (void)core_.processHeadingUpdate(heading_rad,
                                   R,
                                   EskfEventType::MagCorrection,
                                   false);
#endif
}

uint64_t EskfYieldable::gpsEventTimestampFromPps(uint64_t pps_timestamp_us) const {
  if (cfg_.gpsDelayUs < 0) {
    const uint64_t offset = static_cast<uint64_t>(-cfg_.gpsDelayUs);
    return (pps_timestamp_us > offset) ? (pps_timestamp_us - offset) : 0;
  }
  return pps_timestamp_us + static_cast<uint64_t>(cfg_.gpsDelayUs);
}

void EskfYieldable::logGpsDroppedByRejectionWindow(uint64_t gps_timestamp_us) {
  // Pre-liftoff hibernation intentionally drops all aiding packets without
  // flooding reject logs. Once liftoff transitions out of hibernation, grace-
  // window drops are logged as explicit GPS rejects.
  if (hibernating_) {
    return;
  }

  GpsRejectionInfo info{};
  info.reason = GpsRejectReason::RejectionWindow;
  info.consecutive = consecutive_gps_rejects_;
  getEskfLogger().logGpsRejection(EskfEventType::GpsRejectedByWindow,
                                  gps_timestamp_us, info);
}

void EskfYieldable::logGpsDroppedByHistoryHorizon(
    uint64_t gps_timestamp_us,
    uint64_t oldest_imu_timestamp_us) {
  GpsRejectionInfo info{};
  info.reason = GpsRejectReason::HistoryUnavailable;
  info.consecutive = consecutive_gps_rejects_;
  if (oldest_imu_timestamp_us > gps_timestamp_us) {
    // Store the lateness in milliseconds for offline diagnostics.
    const uint64_t lag_us = oldest_imu_timestamp_us - gps_timestamp_us;
    info.threshold = static_cast<float>(lag_us) * 1e-3f;
  }
  getEskfLogger().logGpsRejection(EskfEventType::GpsRejectedByWindow,
                                  gps_timestamp_us, info);
}

void EskfYieldable::logGpsDroppedByCheckpointHorizon(
    uint64_t gps_timestamp_us,
    uint64_t required_min_timestamp_us) {
  GpsRejectionInfo info{};
  info.reason = GpsRejectReason::RewindCheckpointUnavailable;
  info.consecutive = consecutive_gps_rejects_;
  if (required_min_timestamp_us > gps_timestamp_us) {
    const uint64_t lag_us = required_min_timestamp_us - gps_timestamp_us;
    info.threshold = static_cast<float>(lag_us) * 1e-3f;
  }
  getEskfLogger().logGpsRejection(EskfEventType::GpsRejectedByWindow,
                                  gps_timestamp_us, info);
}

bool EskfYieldable::isGpsTimestampOlderThanRetainedImuHistory(
    uint64_t gps_timestamp_us,
    uint64_t *oldest_imu_timestamp_us) const {
  if (imu_push_seq_ == 0) {
    if (oldest_imu_timestamp_us) {
      *oldest_imu_timestamp_us = 0;
    }
    return false;
  }

  const size_t count = imuBufferCount();
  const size_t oldest_idx = (imu_push_seq_ - count) % ESKF_IMU_BUFFER_SIZE;
  const uint64_t oldest_imu_ts = imu_buffer_[oldest_idx].imu.timestamp_us;
  if (oldest_imu_timestamp_us) {
    *oldest_imu_timestamp_us = oldest_imu_ts;
  }
  return gps_timestamp_us < oldest_imu_ts;
}

bool EskfYieldable::isGpsTimestampOutsideRewindCheckpointHorizon(
    uint64_t gps_timestamp_us,
    uint64_t oldest_imu_timestamp_us,
    uint64_t *required_min_timestamp_us) const {
  if (checkpoint_count_ == 0) {
    if (required_min_timestamp_us) {
      *required_min_timestamp_us = 0;
    }
    return false;
  }

  const size_t oldest_cp_idx =
      oldestIndex<ESKF_CHECKPOINT_BUFFER_SIZE>(checkpoint_count_, checkpoint_head_);
  bool found_checkpoint = false;
  uint64_t latest_checkpoint_ts = 0;
  uint64_t oldest_checkpoint_ts =
      checkpoint_buffer_[oldest_cp_idx].timestamp_us;

  for (size_t i = 0; i < checkpoint_count_; ++i) {
    const size_t idx =
        (oldest_cp_idx + i) % ESKF_CHECKPOINT_BUFFER_SIZE;
    const uint64_t cp_ts = checkpoint_buffer_[idx].timestamp_us;
    if (cp_ts <= gps_timestamp_us &&
        (!found_checkpoint || cp_ts > latest_checkpoint_ts)) {
      latest_checkpoint_ts = cp_ts;
      found_checkpoint = true;
    }
  }

  if (!found_checkpoint) {
    if (required_min_timestamp_us) {
      *required_min_timestamp_us = oldest_checkpoint_ts;
    }
    return false;
  }

  if (latest_checkpoint_ts < oldest_imu_timestamp_us) {
    if (required_min_timestamp_us) {
      *required_min_timestamp_us = oldest_imu_timestamp_us;
    }
    return true;
  }

  if (required_min_timestamp_us) {
    *required_min_timestamp_us = latest_checkpoint_ts;
  }
  return false;
}

void EskfYieldable::pushGpsPosition(uint64_t pps_timestamp_us,
                                     const eskf_scalar pos_ned[3],
                                     const eskf_scalar R_pos[3]) {
#if ESKF_USE_GPS_REWIND
  // Retire legacy asymmetry: route position-only updates through packet path
  // so late packets use the same rewind/timing contract as velocity packets.
  static constexpr eskf_scalar kZeroLeverArm[3] = {0, 0, 0};
  pushGpsPacket(pps_timestamp_us, pos_ned, R_pos, nullptr, nullptr,
                kZeroLeverArm);
#else
  (void)pps_timestamp_us;
  core_.correctGpsPosition(pos_ned, R_pos);
#endif
}

void EskfYieldable::pushGpsVelocity(uint64_t pps_timestamp_us,
                                     const eskf_scalar vel_ned[3],
                                     const eskf_scalar R_vel[3],
                                     const eskf_scalar lever_arm_body[3]) {
#if ESKF_USE_GPS_REWIND
  const uint64_t vel_timestamp = gpsEventTimestampFromPps(pps_timestamp_us);
  const bool requires_rewind = (vel_timestamp < kalman_timestamp_us_);

  // If GPS is within the post-liftoff rejection window, drop it before
  // enqueue so it cannot trigger rewind side effects.
  if (vel_timestamp < rejection_end_us_) {
    logGpsDroppedByRejectionWindow(vel_timestamp);
    return;
  }

  if (requires_rewind) {
    uint64_t oldest_imu_ts = 0;
    if (isGpsTimestampOlderThanRetainedImuHistory(vel_timestamp,
                                                  &oldest_imu_ts)) {
      logGpsDroppedByHistoryHorizon(vel_timestamp, oldest_imu_ts);
      return;
    }
    uint64_t required_min_ts = 0;
    if (isGpsTimestampOutsideRewindCheckpointHorizon(vel_timestamp,
                                                     oldest_imu_ts,
                                                     &required_min_ts)) {
      logGpsDroppedByCheckpointHorizon(vel_timestamp, required_min_ts);
      return;
    }
  }
  
  EventEntry e;
  e.type = EventType::GpsVelocity;
  e.timestamp_us = vel_timestamp;
  for (int i = 0; i < 3; ++i) {
    e.data.gps_vel.vel[i] = vel_ned[i];
    e.data.gps_vel.R[i] = R_vel[i];
    e.data.gps_vel.lever_arm[i] = lever_arm_body[i];
  }
  pushEvent(e);
  
  // Trigger rewind to velocity timestamp if filter has passed it
  if (requires_rewind) {
    rewindTo(vel_timestamp);
  }
#else
  (void)pps_timestamp_us;
  core_.correctGpsVelocity(vel_ned, R_vel, lever_arm_body);
#endif
}

void EskfYieldable::pushSideslip(eskf_scalar R_lateral, uint64_t timestamp_us) {
#if ESKF_USE_GPS_REWIND && ESKF_ENABLE_SIDESLIP
  EventEntry e;
  e.type = EventType::Sideslip;
  e.timestamp_us = timestamp_us;
  e.data.sideslip.R_lateral = R_lateral;
  pushEvent(e);
#else
  (void)timestamp_us;
#if ESKF_ENABLE_SIDESLIP
  core_.correctSideslip(R_lateral);
#else
  (void)R_lateral;
#endif
#endif
}

void EskfYieldable::pushGpsPacket(uint64_t pps_timestamp_us,
                                   const eskf_scalar* pos_ned, const eskf_scalar* R_pos,
                                   const eskf_scalar* vel_ned, const eskf_scalar* R_vel,
                                   const eskf_scalar lever_arm_body[3]) {
#if ESKF_USE_GPS_REWIND
  // Use raw PPS as contract input; delay is applied exactly once here.
  const uint64_t gps_timestamp = gpsEventTimestampFromPps(pps_timestamp_us);
  const bool requires_rewind = (gps_timestamp < kalman_timestamp_us_);

  // If GPS is within the post-liftoff rejection window, drop it before
  // enqueue so it cannot trigger rewind side effects.
  if (gps_timestamp < rejection_end_us_) {
    logGpsDroppedByRejectionWindow(gps_timestamp);
    return;
  }

  if (requires_rewind) {
    uint64_t oldest_imu_ts = 0;
    if (isGpsTimestampOlderThanRetainedImuHistory(gps_timestamp,
                                                  &oldest_imu_ts)) {
      logGpsDroppedByHistoryHorizon(gps_timestamp, oldest_imu_ts);
      return;
    }
    uint64_t required_min_ts = 0;
    if (isGpsTimestampOutsideRewindCheckpointHorizon(gps_timestamp,
                                                     oldest_imu_ts,
                                                     &required_min_ts)) {
      logGpsDroppedByCheckpointHorizon(gps_timestamp, required_min_ts);
      return;
    }
  }
  
  EventEntry e;
  e.type = EventType::GpsPacket;
  e.timestamp_us = gps_timestamp;
  
  // Copy position data if provided
  e.data.gps_packet.has_position = (pos_ned != nullptr);
  if (pos_ned) {
    for (int i = 0; i < 3; ++i) {
      e.data.gps_packet.pos[i] = pos_ned[i];
      e.data.gps_packet.R_pos[i] = R_pos ? R_pos[i] : 10.0;  // Default 10m² if not provided
    }
  }
  
  // Copy velocity data if provided
  e.data.gps_packet.has_velocity = (vel_ned != nullptr);
  if (vel_ned) {
    for (int i = 0; i < 3; ++i) {
      e.data.gps_packet.vel[i] = vel_ned[i];
      e.data.gps_packet.R_vel[i] = R_vel ? R_vel[i] : 1.0;  // Default 1 (m/s)² if not provided
    }
  }
  
  // Copy lever arm
  for (int i = 0; i < 3; ++i) {
    e.data.gps_packet.lever_arm[i] = lever_arm_body[i];
  }
  
  pushEvent(e);
  
  // Trigger rewind if filter has already passed this timestamp
  if (requires_rewind) {
    rewindTo(gps_timestamp);
  }
#else
  // Without rewind, process immediately using legacy path
  (void)pps_timestamp_us;
  if (vel_ned) {
    core_.correctGpsVelocity(vel_ned, R_vel, lever_arm_body);
  }
  if (pos_ned) {
    core_.correctGpsPosition(pos_ned, R_pos);
  }
#endif
}

// ============================================================
// Liftoff Transition
// ============================================================

void EskfYieldable::injectLiftoffSnap(const LiftoffInitData& init_data, 
                                       uint64_t rewind_to_ts) {
  uint64_t effective_rewind_to_ts = rewind_to_ts;

  // Clamp liftoff rewind target to retained IMU history horizon.
  // In preflight hibernation the IMU ring continuously laps, so requesting
  // a deeper rewind than retained history is unrecoverable.
  if (imu_push_seq_ > 0) {
    const size_t imu_count = imuBufferCount();
    const size_t oldest = (imu_push_seq_ - imu_count) % ESKF_IMU_BUFFER_SIZE;
    const uint64_t oldest_ts = imu_buffer_[oldest].imu.timestamp_us;
    if (effective_rewind_to_ts < oldest_ts) {
      effective_rewind_to_ts = oldest_ts;
    }
  }

  // Wake up from hibernation - ESKF is now active
  hibernating_ = false;
  
  // Set rejection window end time (Phase 4)
  rejection_end_us_ = init_data.liftoff_us + tuning_cfg_.liftoff_rejection_us;
  
#if ESKF_USE_GPS_REWIND
  EventEntry e;
  e.type = EventType::LiftoffSnap;
  e.timestamp_us = effective_rewind_to_ts;
  
  // Copy full initialization data
  for (int i = 0; i < 4; ++i) {
    e.data.liftoff.q[i] = init_data.q[i];
  }
  for (int i = 0; i < 3; ++i) {
    e.data.liftoff.b_gyro[i] = init_data.b_gyro[i];
    e.data.liftoff.b_acc[i] = init_data.b_acc[i];
  }
  e.data.liftoff.heading_variance = init_data.heading_variance;
  e.data.liftoff.heading_initialized = init_data.heading_initialized;
  e.data.liftoff.ground_altitude_m = init_data.ground_altitude_m;
  e.data.liftoff.ground_reference_valid = init_data.ground_reference_valid;
  
  liftoff_snap_pending_ = true;
  pushEvent(e);
  
  // Rewind to before liftoff. Liftoff uses a state/covariance snap at
  // rewind_to_ts, so replay should start there directly instead of bridging
  // from an initialization-era checkpoint.
  rewindTo(effective_rewind_to_ts, true);
#else
  // Without rewind, apply full initialization immediately
  State s = core_.state();
  
  // Reset position and velocity to zero
  for (int i = 0; i < 3; ++i) {
    s.p[i] = 0;
    s.v[i] = 0;
    s.b_acc[i] = init_data.b_acc[i];
    s.b_gyro[i] = init_data.b_gyro[i];
  }
  for (int i = 0; i < 4; ++i) {
    s.q[i] = init_data.q[i];
  }
  // Initialize baro bias to ground altitude for proper AGL measurement
  s.b_baro = init_data.ground_reference_valid ? init_data.ground_altitude_m : 0;
  
  core_.setState(s);
  core_.setHeadingInitialized(init_data.heading_initialized);
  (void)rewind_to_ts;
#endif
}

// ============================================================
// Event Processing - catchUp()
// ============================================================

bool EskfYieldable::catchUp(uint64_t target_timestamp_us, uint32_t budget_us) {
  if (budget_us == 0) {
    budget_us = cfg_.catchupBudgetUs;
  }
  
  const uint32_t start_us = nowMicros();
  uint32_t events_processed = 0;
  const bool started_in_rewind = in_rewind_;

#if KALMAN_DEBUG_PRINT
  // Diagnose when kalman_timestamp_us_ is ahead of target (while-loop skipped)
  if (kalman_timestamp_us_ > target_timestamp_us) {
    static uint32_t ahead_counter = 0;
    if (++ahead_counter >= 500) {
      ahead_counter = 0;
      printf("[CATCHUP] AHEAD: kalTs=%u:%u > targetTs=%u:%u  diff=%uus  "
             "rewind=%d hiber=%d\r\n",
             (unsigned)(kalman_timestamp_us_ >> 32), (unsigned)kalman_timestamp_us_,
             (unsigned)(target_timestamp_us >> 32), (unsigned)target_timestamp_us,
             (unsigned)(uint32_t)(kalman_timestamp_us_ - target_timestamp_us),
             (int)in_rewind_, (int)hibernating_);
    }
  }
#endif
  
  while (kalman_timestamp_us_ <= target_timestamp_us) {
    discardStalePendingBaro();
    
    // Find next event (earliest timestamp across all buffers)
    uint64_t next_imu_ts = peekNextImuTimestamp();
    uint64_t next_baro_ts = peekNextBaroTimestamp();
    uint64_t next_event_ts = peekNextEventTimestamp();
    
    // Find minimum (UINT64_MAX means buffer exhausted)
    uint64_t earliest = next_imu_ts;
    if (next_baro_ts < earliest) earliest = next_baro_ts;
    if (next_event_ts < earliest) earliest = next_event_ts;
    
    // No more data to process
    if (earliest == UINT64_MAX) {
#if KALMAN_DEBUG_PRINT
      static uint32_t catchup_diag_counter = 0;
      if (++catchup_diag_counter >= 500) {
        catchup_diag_counter = 0;
        printf("[CATCHUP] exhausted: kalTs=%u:%u targetTs=%u:%u "
               "imuPending=%u imuPushSeq=%u imuReadSeq=%u\r\n",
               (unsigned)(kalman_timestamp_us_ >> 32), (unsigned)kalman_timestamp_us_,
               (unsigned)(target_timestamp_us >> 32), (unsigned)target_timestamp_us,
               (unsigned)(imu_push_seq_ - imu_read_seq_),
               (unsigned)imu_push_seq_, (unsigned)imu_read_seq_);
      }
#endif
      break;
    }
    
    // Don't process events beyond target
    if (earliest > target_timestamp_us) {
#if KALMAN_DEBUG_PRINT
      static uint32_t catchup_future_counter = 0;
      if (++catchup_future_counter >= 500) {
        catchup_future_counter = 0;
        printf("[CATCHUP] future: earliest=%u:%u target=%u:%u "
               "kalTs=%u:%u imuPending=%u imuPushSeq=%u\r\n",
               (unsigned)(earliest >> 32), (unsigned)earliest,
               (unsigned)(target_timestamp_us >> 32), (unsigned)target_timestamp_us,
               (unsigned)(kalman_timestamp_us_ >> 32), (unsigned)kalman_timestamp_us_,
               (unsigned)(imu_push_seq_ - imu_read_seq_),
               (unsigned)imu_push_seq_);
      }
#endif
      break;
    }

    // If we already reached target, only drain entries exactly at target.
    if (kalman_timestamp_us_ == target_timestamp_us &&
        earliest != kalman_timestamp_us_) {
      break;
    }

    // ── Guard: skip stale entries from ring-buffer wrap-around ──
    // With sequence-number indexing this should be rare, but keep as safety net
    // in case a rewind or other edge case leaves stale timestamps visible.
    // Threshold: 1 ms — comfortably above any normal jitter but far below
    // the ~500 ms span of the full ring buffer.
    constexpr uint64_t kStaleThresholdUs = 1000;
    if (earliest + kStaleThresholdUs < kalman_timestamp_us_) {
      // Advance the correct read index past this stale entry.
      if (earliest == next_imu_ts)       { imu_read_seq_++; }
      else if (earliest == next_baro_ts) { baro_read_idx_++; }
      else                               { event_read_idx_++; }
      stats_.stale_skips++;
#if KALMAN_DEBUG_PRINT
      static uint32_t stale_log_counter = 0;
      if (++stale_log_counter >= 3000) {
        stale_log_counter = 0;
        printf("[CATCHUP] STALE-SKIP: ts=%u:%u  kalTs=%u:%u  "
               "staleSkips=%u  imuPending=%u\r\n",
               (unsigned)(earliest >> 32), (unsigned)earliest,
               (unsigned)(kalman_timestamp_us_ >> 32),
               (unsigned)kalman_timestamp_us_,
               (unsigned)stats_.stale_skips,
               (unsigned)(imu_push_seq_ - imu_read_seq_));
      }
#endif
      continue;   // re-peek without counting as a processed event
    }

    // Process the earliest event (handle liftoff snap before IMU at same time)
    const bool imu_tied = (earliest == next_imu_ts);
    const bool event_tied = (earliest == next_event_ts);
    if (imu_tied && event_tied &&
        peekNextEventType() == EventType::LiftoffSnap) {
      processNextEvent();
    } else if (earliest == next_imu_ts) {
      processNextImu();
    } else if (earliest == next_baro_ts) {
      processNextBaro();
    } else {
      processNextEvent();
    }

    // Keep timeline monotonic even if a stale out-of-order entry is consumed.
    if (earliest > kalman_timestamp_us_) {
      kalman_timestamp_us_ = earliest;
    }
    events_processed++;
    
#if ESKF_LOG_REWIND_FRAMES
    // Log intermediate states during catchUp for debugging.
    // This captures both pre-rewind states AND replayed states after GPS update.
    // Rate-limited by the logger's internal limiter (e.g., 50Hz).
    logStateIfDue();
    logCovarianceIfDue();
#endif

    // Check time budget after processing each event (post-processing check).
    // The guard is placed here — not at the top of the loop — so that at least
    // one event is always processed before yielding.  A pre-loop check would
    // fire immediately under Valgrind (where startup overhead alone can exceed
    // a tight budget), causing catchUp() to return false on an empty timeline
    // or before processing any events, which violates the contract that the
    // function returns true when there is nothing left to do.
    if (nowMicros() - start_us >= budget_us) {
      last_catchup_duration_us_ = nowMicros() - start_us;
      last_events_processed_ = events_processed;
      stats_.catchup_budget_yields++;
      return false;  // Yield
    }
  }
  
  last_catchup_duration_us_ = nowMicros() - start_us;
  last_events_processed_ = events_processed;

  if (started_in_rewind) {
    RewindInfo rewind_info{};
    rewind_info.target_timestamp_us = rewind_trigger_timestamp_us_;
    rewind_info.rewind_origin_timestamp_us = rewind_origin_timestamp_us_;
    rewind_info.checkpoint_timestamp_us = rewind_checkpoint_timestamp_us_;
    rewind_info.imu_replayed = rewind_imu_replayed_;
    rewind_info.baro_replayed = rewind_baro_replayed_;
    rewind_info.events_replayed = rewind_events_replayed_;
    rewind_info.catchup_duration_us = last_catchup_duration_us_;
    rewind_info.pending_gap_dt_us = pending_rewind_gap_dt_us_;
    rewind_info.had_data_gap = (pending_rewind_gap_dt_us_ > 0);
    getEskfLogger().logRewind(EskfEventType::RewindCompleted,
                              kalman_timestamp_us_, rewind_info);
    getEskfLogger().logEvent(EskfEventType::RewindStateSnapshotPost,
                             kalman_timestamp_us_);
    getEskfLogger().logStateCritical(core_.createStateSnapshot());
    rewind_imu_replayed_ = 0;
    rewind_baro_replayed_ = 0;
    rewind_events_replayed_ = 0;
  }
  
  // (m3) Clear rewind flag when caught up
  in_rewind_ = false;
  
  // Log state and covariance (rate-limited by logger's internal limiters)
  // This ensures periodic logging is tied to the catchUp flow
  logStateIfDue();
  logCovarianceIfDue();
  
  return true;  // Caught up
}

void EskfYieldable::discardStalePendingBaro() {
#if ESKF_USE_GPS_REWIND
  if (tuning_cfg_.baro_snapshot_max_age_us == 0) {
    return;
  }

  while (baro_read_idx_ < baro_count_) {
    const size_t oldest = oldestIndex<ESKF_BARO_BUFFER_SIZE>(baro_count_, baro_head_);
    const size_t idx = (oldest + baro_read_idx_) % ESKF_BARO_BUFFER_SIZE;
    const BaroEntry &entry = baro_buffer_[idx];

    if (entry.ready) {
      return;
    }
    if (entry.dropped) {
      baro_read_idx_++;
      continue;
    }
    if (kalman_timestamp_us_ <= entry.timestamp_us) {
      return;
    }

    const uint64_t age_us = kalman_timestamp_us_ - entry.timestamp_us;
    if (age_us <= tuning_cfg_.baro_snapshot_max_age_us) {
      return;
    }

    if (has_pending_baro_ && pending_baro_slot_ == idx) {
      has_pending_baro_ = false;
    }
    stats_.baro_drops++;
    getEskfLogger().logEvent(EskfEventType::BaroSnapshotDropped,
                             entry.timestamp_us,
                             static_cast<float>(age_us));
    baro_read_idx_++;
  }
#endif
}

void EskfYieldable::saveCheckpointNow() {
  if (hibernating_) {
    return;
  }
  checkpoint_buffer_[checkpoint_head_] = captureRewindCheckpoint();
  checkpoint_head_ = (checkpoint_head_ + 1) % ESKF_CHECKPOINT_BUFFER_SIZE;
  if (checkpoint_count_ < ESKF_CHECKPOINT_BUFFER_SIZE) {
    checkpoint_count_++;
  }
  imu_since_checkpoint_ = 0;
}

// ============================================================
// Rewind Implementation
// ============================================================

void EskfYieldable::rewindTo(uint64_t timestamp_us, bool liftoff_rewind) {
  // (I3) Track rewind statistics
  stats_.rewind_count++;
  if (kalman_timestamp_us_ > timestamp_us) {
    stats_.rewind_total_depth_us += (kalman_timestamp_us_ - timestamp_us);
  }
  
  // Log rewind start
  RewindInfo rewind_info{};
  rewind_info.target_timestamp_us = timestamp_us;
  rewind_info.rewind_origin_timestamp_us = kalman_timestamp_us_;
  rewind_trigger_timestamp_us_ = timestamp_us;
  rewind_origin_timestamp_us_ = kalman_timestamp_us_;
  rewind_checkpoint_timestamp_us_ = 0;
  rewind_imu_replayed_ = 0;
  rewind_baro_replayed_ = 0;
  rewind_events_replayed_ = 0;
  pending_rewind_gap_dt_us_ = 0;
  getEskfLogger().logEvent(EskfEventType::RewindStateSnapshotPre,
                           kalman_timestamp_us_);
  getEskfLogger().logStateCritical(core_.createStateSnapshot());
  getEskfLogger().logRewind(EskfEventType::RewindStarted, 
                            kalman_timestamp_us_, rewind_info);
  
  // Find the best checkpoint to restore from:
  // Search periodic checkpoints for the most recent one BEFORE target timestamp
  bool found_periodic = false;
  size_t best_checkpoint_idx = 0;
  uint64_t best_checkpoint_ts = 0;
  
  for (size_t i = 0; i < checkpoint_count_; ++i) {
    size_t oldest = (checkpoint_count_ < ESKF_CHECKPOINT_BUFFER_SIZE) ? 0 : checkpoint_head_;
    size_t idx = (oldest + i) % ESKF_CHECKPOINT_BUFFER_SIZE;
    uint64_t cp_ts = checkpoint_buffer_[idx].timestamp_us;
    
    if (cp_ts <= timestamp_us && cp_ts > best_checkpoint_ts) {
      best_checkpoint_ts = cp_ts;
      best_checkpoint_idx = idx;
      found_periodic = true;
    }
  }
  
  // Restore from best available checkpoint and determine replay start time.
  // Replay must start from the CHECKPOINT time, not the rewind target, to
  // re-integrate IMU and re-process events (including earlier GPS fixes)
  // between the checkpoint and the GPS event that triggered this rewind.
  uint64_t replay_from = timestamp_us;  // fallback: rewind target
  
  const bool liftoff_direct_replay = liftoff_rewind && checkpoint_count_ == 0;

  if (liftoff_direct_replay) {
    // Pre-liftoff hibernation intentionally skips periodic ESKF checkpoints.
    // For liftoff snap rewinds, replay from the requested target directly.
    // LiftoffSnap will initialize state/covariance at this timestamp.
    replay_from = timestamp_us;
    rewind_info.checkpoint_timestamp_us = replay_from;
  } else if (found_periodic) {
    restoreRewindCheckpoint(checkpoint_buffer_[best_checkpoint_idx]);
    rewind_info.checkpoint_timestamp_us = best_checkpoint_ts;
    replay_from = best_checkpoint_ts;
  } else if (has_checkpoint_) {
    // Fall back to oldest checkpoint
    restoreRewindCheckpoint(oldest_checkpoint_);
    rewind_info.checkpoint_timestamp_us = oldest_checkpoint_.timestamp_us;
    replay_from = oldest_checkpoint_.timestamp_us;
    
    // Only log as warning if we EXPECTED periodic checkpoints to exist.
    // At liftoff transition, checkpoint_count_ == 0 is normal (we skipped
    // checkpoint creation during hibernation). The LiftoffSnap event will
    // properly initialize the ESKF state.
    if (checkpoint_count_ > 0) {
      // We have checkpoints but none cover the target - this is unexpected
      stats_.rewind_no_checkpoint_count++;
      getEskfLogger().logRewind(EskfEventType::RewindNoCheckpoint,
                                kalman_timestamp_us_, rewind_info);
    }
    // Otherwise, this is the normal liftoff transition - no warning needed
  } else {
    // (M3) No checkpoint at all - this shouldn't happen after initialize()
    stats_.rewind_no_checkpoint_count++;
    getEskfLogger().logRewind(EskfEventType::RewindNoCheckpoint,
                              kalman_timestamp_us_, rewind_info);
    // Continue with current (possibly wrong) state
  }
  rewind_checkpoint_timestamp_us_ = rewind_info.checkpoint_timestamp_us;
  
  // Binary search each buffer to find starting read indices from CHECKPOINT time.
  // IMU: when replay starts from a restored checkpoint, use replay_from + 1
  // because that checkpoint already includes the IMU at replay_from
  // (captured after core_.predict()).
  // Liftoff direct replay has no restored checkpoint, so include IMU samples at
  // replay_from to preserve same-timestamp LiftoffSnap->IMU ordering.
  const uint64_t imu_replay_from =
      liftoff_direct_replay ? replay_from : (replay_from + 1);
  // Binary search returns a relative offset from oldest surviving entry.
  // Convert to absolute sequence number.
  {
    const size_t imu_count = imuBufferCount();
    const size_t imu_head = imu_push_seq_ % ESKF_IMU_BUFFER_SIZE;
    const size_t rel = binarySearchBuffer<ImuEntry, ESKF_IMU_BUFFER_SIZE>(
        imu_buffer_, imu_count, imu_head, imu_replay_from);
    const uint64_t oldest_seq = imu_push_seq_ - imu_count;
    imu_read_seq_ = oldest_seq + rel;
  }
  
  baro_read_idx_ = binarySearchBuffer<BaroEntry, ESKF_BARO_BUFFER_SIZE>(
      baro_buffer_, baro_count_, baro_head_, replay_from);

  if (stats_.out_of_order_events > 0) {
    // Out-of-order events violate binary-search ordering assumptions.
    // Fall back to insertion-order scan for a deterministic replay cursor.
    event_read_idx_ = 0;
    const size_t oldest_event =
        oldestIndex<ESKF_EVENT_BUFFER_SIZE>(event_count_, event_head_);
    while (event_read_idx_ < event_count_) {
      const size_t idx =
          (oldest_event + event_read_idx_) % ESKF_EVENT_BUFFER_SIZE;
      if (event_buffer_[idx].timestamp_us >= replay_from) {
        break;
      }
      event_read_idx_++;
    }
  } else {
    event_read_idx_ = binarySearchBuffer<EventEntry, ESKF_EVENT_BUFFER_SIZE>(
        event_buffer_, event_count_, event_head_, replay_from);
  }

  // Default: no pending gap compensation unless a retained-history gap is detected.
  pending_rewind_gap_dt_s_ = 0;
  pending_rewind_gap_dt_us_ = 0;
  
  // (M4) Check for data gap: if oldest buffered sample is newer than replay start
  if (!liftoff_direct_replay && imu_push_seq_ > 0) {
    const size_t imu_count = imuBufferCount();
    const size_t oldest_imu_idx = (imu_push_seq_ - imu_count) % ESKF_IMU_BUFFER_SIZE;
    uint64_t oldest_imu_ts = imu_buffer_[oldest_imu_idx].imu.timestamp_us;
    if (oldest_imu_ts > replay_from) {
      stats_.rewind_data_gap_count++;
      rewind_info.had_data_gap = true;
      getEskfLogger().logRewind(EskfEventType::RewindDataGap,
                                kalman_timestamp_us_, rewind_info);

      // When rewind target is older than retained IMU history, replay starts at
      // oldest retained IMU. Integrate the missing elapsed time once on the first
      // replayed IMU step to avoid time-teleportation with near-zero kinematics.
      const uint64_t gap_us = oldest_imu_ts - replay_from;
      pending_rewind_gap_dt_s_ = static_cast<eskf_scalar>(gap_us) / 1e6;
      pending_rewind_gap_dt_us_ = static_cast<uint32_t>(gap_us > 0xFFFFFFFFull
                        ? 0xFFFFFFFFu
                        : gap_us);

      core_.inflatePositionCovariance(tuning_cfg_.gps_reset_p_pos);
      core_.inflateVelocityCovariance(tuning_cfg_.gps_reset_p_vel);
      getEskfLogger().logEvent(EskfEventType::RewindGapCovInflated,
                               kalman_timestamp_us_,
                               static_cast<float>(tuning_cfg_.gps_reset_p_pos));
    }
  }
  
  // Set Kalman timestamp to checkpoint time so catchUp replays everything
  // from checkpoint through the GPS event and forward to real-time.
  kalman_timestamp_us_ = replay_from;
  in_rewind_ = true;  // (m3) Will be cleared when catchUp reaches real-time
}

template<typename T, size_t N>
size_t EskfYieldable::binarySearchBuffer(const T* buffer, size_t count, 
                                          size_t head,
                                          uint64_t timestamp_us) const {
  if (count == 0) return 0;
  
  // Binary search for first entry with timestamp >= target
  size_t lo = 0, hi = count;
  
  while (lo < hi) {
    size_t mid = lo + (hi - lo) / 2;
    size_t oldest = (count < N) ? 0 : head;
    size_t buf_idx = (oldest + mid) % N;
    
    if (getTimestamp(buffer[buf_idx]) < timestamp_us) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  
  return lo;  // Returns relative offset from oldest entry
}

// ============================================================
// Peek Next Timestamp
// ============================================================

uint64_t EskfYieldable::peekNextImuTimestamp() const {
  if (imu_read_seq_ >= imu_push_seq_) return UINT64_MAX;
  
  // Check for overwrite: entry may have been overwritten since we recorded read_seq
  if (imu_push_seq_ - imu_read_seq_ > ESKF_IMU_BUFFER_SIZE) {
    return UINT64_MAX;  // Will be fixed up in catchUp via drop detection
  }
  
  const size_t slot = imu_read_seq_ % ESKF_IMU_BUFFER_SIZE;
  return imu_buffer_[slot].imu.timestamp_us;
}

uint64_t EskfYieldable::peekNextBaroTimestamp() const {
  if (baro_read_idx_ >= baro_count_) return UINT64_MAX;
  
  size_t oldest = oldestIndex<ESKF_BARO_BUFFER_SIZE>(baro_count_, baro_head_);
  size_t idx = (oldest + baro_read_idx_) % ESKF_BARO_BUFFER_SIZE;
  
  // Skip if measurement not ready yet (async timing)
  if (!baro_buffer_[idx].ready) return UINT64_MAX;
  
  return baro_buffer_[idx].timestamp_us;
}

size_t EskfYieldable::selectNextEventOffset() const {
  if (event_read_idx_ >= event_count_) {
    return event_count_;
  }

  // Fast path: insertion order is chronological.
  if (stats_.out_of_order_events == 0) {
    return event_read_idx_;
  }

  // Out-of-order detected: scan unread window for the earliest timestamp.
  // Keep insertion order for equal timestamps, except prioritize LiftoffSnap
  // among equal-time events to preserve same-timestamp tie contract.
  const size_t oldest =
      oldestIndex<ESKF_EVENT_BUFFER_SIZE>(event_count_, event_head_);
  size_t best_offset = event_read_idx_;
  size_t best_idx = (oldest + best_offset) % ESKF_EVENT_BUFFER_SIZE;
  uint64_t best_ts = event_buffer_[best_idx].timestamp_us;

  for (size_t off = event_read_idx_ + 1; off < event_count_; ++off) {
    const size_t idx = (oldest + off) % ESKF_EVENT_BUFFER_SIZE;
    const uint64_t ts = event_buffer_[idx].timestamp_us;
    if (ts < best_ts) {
      best_ts = ts;
      best_offset = off;
      best_idx = idx;
      continue;
    }

    if (ts == best_ts &&
        event_buffer_[idx].type == EventType::LiftoffSnap &&
        event_buffer_[best_idx].type != EventType::LiftoffSnap) {
      best_offset = off;
      best_idx = idx;
    }
  }

  return best_offset;
}

uint64_t EskfYieldable::peekNextEventTimestamp() const {
  const size_t selected_offset = selectNextEventOffset();
  if (selected_offset >= event_count_) {
    return UINT64_MAX;
  }

  size_t oldest = oldestIndex<ESKF_EVENT_BUFFER_SIZE>(event_count_, event_head_);
  size_t idx = (oldest + selected_offset) % ESKF_EVENT_BUFFER_SIZE;
  return event_buffer_[idx].timestamp_us;
}

EventType EskfYieldable::peekNextEventType() const {
  const size_t selected_offset = selectNextEventOffset();
  if (selected_offset >= event_count_) {
    return EventType::MagHeading;
  }

  size_t oldest = oldestIndex<ESKF_EVENT_BUFFER_SIZE>(event_count_, event_head_);
  size_t idx = (oldest + selected_offset) % ESKF_EVENT_BUFFER_SIZE;
  return event_buffer_[idx].type;
}

// ============================================================
// Process Next Entry
// ============================================================

void EskfYieldable::processNextImu() {
  if (imu_read_seq_ >= imu_push_seq_) return;
  
  // Advance past overwritten entries (drop detection)
  if (imu_push_seq_ - imu_read_seq_ > ESKF_IMU_BUFFER_SIZE) {
    const uint64_t lost = (imu_push_seq_ - imu_read_seq_) - ESKF_IMU_BUFFER_SIZE;
    stats_.imu_drops += static_cast<uint32_t>(lost);
    imu_read_seq_ = imu_push_seq_ - ESKF_IMU_BUFFER_SIZE;
    if (imu_read_seq_ >= imu_push_seq_) return;
  }
  
  const size_t slot = imu_read_seq_ % ESKF_IMU_BUFFER_SIZE;
  const ImuEntry& entry = imu_buffer_[slot];
  const auto replayPredictChunked =
      [&](const ImuFrame &base_imu, eskf_scalar dt_total_s,
          uint64_t final_timestamp_us) {
        if (!(std::isfinite(dt_total_s) && dt_total_s > 0)) {
          return;
        }

        const eskf_scalar max_dt = tuning_cfg_.predict_max_dt_s;
        if (max_dt > 0 && dt_total_s > max_dt) {
          uint64_t synthetic_ts = core_.state().timestamp_us;
          eskf_scalar remaining = dt_total_s;
          while (remaining > 0) {
            const eskf_scalar step_dt = (remaining > max_dt) ? max_dt : remaining;
            remaining -= step_dt;

            ImuFrame imu_step = base_imu;
            if (remaining <= 0) {
              imu_step.timestamp_us = final_timestamp_us;
            } else {
              const uint64_t step_us = static_cast<uint64_t>(step_dt * 1e6);
              synthetic_ts += (step_us > 0) ? step_us : 1;
              if (synthetic_ts > final_timestamp_us) {
                synthetic_ts = final_timestamp_us;
              }
              imu_step.timestamp_us = synthetic_ts;
            }
            core_.predict(imu_step, step_dt);
          }
          return;
        }

        ImuFrame imu_step = base_imu;
        imu_step.timestamp_us = final_timestamp_us;
        core_.predict(imu_step, dt_total_s);
      };

  // Rewind-gap bridge: propagate missing time with neutral IMU (coast-like)
  // instead of stretching the first retained sample across the dead interval.
  if (pending_rewind_gap_dt_s_ > 0) {
    const eskf_scalar gap_dt_s = pending_rewind_gap_dt_s_;
    pending_rewind_gap_dt_s_ = 0;

    // Avoid coning/sculling/trapezoidal coupling across a dropped-history gap.
    core_.resetIntegrationState();

    ImuFrame gap_imu = entry.imu;
    for (int axis = 0; axis < 3; ++axis) {
      gap_imu.accel[axis] = 0;
      gap_imu.gyro[axis] = 0;
    }

    uint64_t gap_end_ts = entry.imu.timestamp_us;
    if (std::isfinite(entry.dt) && entry.dt > 0) {
      const uint64_t entry_dt_us = static_cast<uint64_t>(entry.dt * 1e6);
      if (gap_end_ts > entry_dt_us) {
        gap_end_ts -= entry_dt_us;
      }
    }

    replayPredictChunked(gap_imu, gap_dt_s, gap_end_ts);
  }

  replayPredictChunked(entry.imu, entry.dt, entry.imu.timestamp_us);

  if (in_rewind_) {
    rewind_imu_replayed_++;
  }

  // Periodic checkpoint save at replay time (not ingest time) keeps checkpoint
  // state synchronized with the processed IMU timestamp, even during rewind.
  if (!hibernating_) {
    imu_since_checkpoint_++;
    if (imu_since_checkpoint_ >= ESKF_CHECKPOINT_INTERVAL) {
      checkpoint_buffer_[checkpoint_head_] = captureRewindCheckpoint();
      checkpoint_head_ = (checkpoint_head_ + 1) % ESKF_CHECKPOINT_BUFFER_SIZE;
      if (checkpoint_count_ < ESKF_CHECKPOINT_BUFFER_SIZE) {
        checkpoint_count_++;
      }
      imu_since_checkpoint_ = 0;
    }
  }
  
  // Track acceleration magnitude for High-G GPS rejection
  // This is in g-units: |a| / g_local
  eskf_scalar accel_sq = entry.imu.accel[0] * entry.imu.accel[0] +
                         entry.imu.accel[1] * entry.imu.accel[1] +
                         entry.imu.accel[2] * entry.imu.accel[2];
  last_accel_magnitude_g_ = std::sqrt(accel_sq) / constants::kGravityLocal;

  imu_read_seq_++;
}

void EskfYieldable::processNextBaro() {
  if (baro_read_idx_ >= baro_count_) return;
  
  size_t oldest = oldestIndex<ESKF_BARO_BUFFER_SIZE>(baro_count_, baro_head_);
  size_t idx = (oldest + baro_read_idx_) % ESKF_BARO_BUFFER_SIZE;
  
  const BaroEntry& entry = baro_buffer_[idx];
  
  // Phase 4: Reject aiding sensors during replay and initial liftoff
  if (entry.timestamp_us < rejection_end_us_) {
    baro_read_idx_++;  // Consume but don't process
    return;
  }
  
  // Only process if ready
  if (entry.ready) {
    // Hybrid baro path:
    // - If replay has not yet passed trigger time, apply the normal baro model
    //   against the current replayed state at this epoch.
    // - If trigger time is already behind Kalman timeline (true late completion),
    //   keep innovation-transport behavior using the trigger snapshot.
    const bool late_completion = (kalman_timestamp_us_ > entry.timestamp_us);
    if (late_completion) {
      core_.correctBaroWithSnapshot(entry.alt_m,
                                     entry.predicted_alt_at_trigger,
                                     entry.R);
    } else {
      core_.correctBaroAltitude(entry.alt_m, entry.R);
    }
    if (in_rewind_) {
      rewind_baro_replayed_++;
    }
  }
  
  baro_read_idx_++;
}

void EskfYieldable::processNextEvent() {
  if (event_read_idx_ >= event_count_) return;

  const size_t selected_offset = selectNextEventOffset();
  if (selected_offset >= event_count_) {
    return;
  }

  size_t oldest = oldestIndex<ESKF_EVENT_BUFFER_SIZE>(event_count_, event_head_);
  size_t read_idx = (oldest + event_read_idx_) % ESKF_EVENT_BUFFER_SIZE;
  size_t idx = (oldest + selected_offset) % ESKF_EVENT_BUFFER_SIZE;

  // Preserve ring-read cursor semantics by moving selected event into the
  // current read slot when out-of-order timestamps are present.
  if (idx != read_idx) {
    std::swap(event_buffer_[read_idx], event_buffer_[idx]);
    idx = read_idx;
  }
  
  const EventEntry& entry = event_buffer_[idx];

  if (in_rewind_) {
    rewind_events_replayed_++;
  }
  
  switch (entry.type) {
    case EventType::GpsPosition: {
      // Phase 4: Reject GPS during replay and initial liftoff
      if (entry.timestamp_us < rejection_end_us_) break;
      
      // Apply R inflation (TRUST_FACTOR + High-G)
      eskf_scalar r_mult = tuning_cfg_.gps_trust_factor;
      if (last_accel_magnitude_g_ > tuning_cfg_.gps_high_g_threshold) {
        r_mult *= tuning_cfg_.gps_high_g_r_factor;
      }
      eskf_scalar r_mult_sq = r_mult * r_mult;
      eskf_scalar R_inflated[3] = {
        entry.data.gps_pos.R[0] * r_mult_sq,
        entry.data.gps_pos.R[1] * r_mult_sq,
        entry.data.gps_pos.R[2] * r_mult_sq
      };
      core_.correctGpsPosition(entry.data.gps_pos.pos, R_inflated);
      saveCheckpointNow();
      break;
    }
      
    case EventType::GpsVelocity: {
      // Phase 4: Reject GPS during replay and initial liftoff
      if (entry.timestamp_us < rejection_end_us_) break;
      
      // Apply R inflation (TRUST_FACTOR + High-G)
      eskf_scalar r_mult = tuning_cfg_.gps_trust_factor;
      if (last_accel_magnitude_g_ > tuning_cfg_.gps_high_g_threshold) {
        r_mult *= tuning_cfg_.gps_high_g_r_factor;
      }
      eskf_scalar r_mult_sq = r_mult * r_mult;
      eskf_scalar R_inflated[3] = {
        entry.data.gps_vel.R[0] * r_mult_sq,
        entry.data.gps_vel.R[1] * r_mult_sq,
        entry.data.gps_vel.R[2] * r_mult_sq
      };
      
#if ESKF_USE_LEVER_ARM_AVERAGING
      // Compute averaged lever arm velocity over the GPS internal filter window
    // This mode uses an attitude approximation, so disable it during
    // high-rate tumbling where that approximation is weakest.
      eskf_scalar avg_lever_vel_ned[3];
      uint64_t window_us = static_cast<uint64_t>(cfg_.gpsDelayUs < 0 ? -cfg_.gpsDelayUs : cfg_.gpsDelayUs);
    const bool allow_averaging =
      tuning_cfg_.gps_vel_tumble_gyro_threshold <= 0 ||
      core_.prevGyroMagnitude() <= tuning_cfg_.gps_vel_tumble_gyro_threshold;
      
    bool has_average = false;
    if (allow_averaging) {
    has_average = computeAveragedLeverArmVelocity(
      entry.data.gps_vel.lever_arm,
      entry.timestamp_us,
      window_us,
      avg_lever_vel_ned);
    }
      
      if (has_average) {
        core_.correctGpsVelocityWithAveragedLeverArm(
            entry.data.gps_vel.vel,
            R_inflated,
            entry.data.gps_vel.lever_arm,
            avg_lever_vel_ned);
      } else {
        // Fallback to single-sample method if not enough IMU data
        core_.correctGpsVelocity(entry.data.gps_vel.vel, 
                                 R_inflated,
                                 entry.data.gps_vel.lever_arm);
      }
#else
      // Single-sample lever arm (legacy behavior)
      core_.correctGpsVelocity(entry.data.gps_vel.vel, 
                               R_inflated,
                               entry.data.gps_vel.lever_arm);
#endif
  saveCheckpointNow();
      break;
    }
      
    case EventType::MagHeading:
      // Phase 4: Reject mag during replay and initial liftoff
      if (entry.timestamp_us < rejection_end_us_) break;
        {
          const HeadingUpdateResult heading_result =
              core_.processHeadingUpdate(entry.data.mag.heading,
                                         entry.data.mag.R,
                                         EskfEventType::MagCorrection,
                                         false);
          if (heading_result == HeadingUpdateResult::Snapped ||
              heading_result == HeadingUpdateResult::Resurrected) {
            flushUnreadBaroAfterStateReset(entry.timestamp_us);
          }
        }
      break;
      
    case EventType::LiftoffSnap: {
      liftoff_snap_pending_ = false;
      flushUnreadBaroAfterStateReset(entry.timestamp_us);
      // Phase 3: Complete ESKF state initialization
      getEskfLogger().logEvent(EskfEventType::LiftoffSnapInjected,
                               entry.timestamp_us);
      
      State s;
      s.setIdentity();
      
      // 1. Set quaternion from RailShadow (combined gravity + heading)
      for (int i = 0; i < 4; ++i) {
        s.q[i] = entry.data.liftoff.q[i];
      }
      
      // 2. Set gyro bias from LPF (NOT Mahony integral)
      for (int i = 0; i < 3; ++i) {
        s.b_gyro[i] = entry.data.liftoff.b_gyro[i];
        s.b_acc[i] = entry.data.liftoff.b_acc[i];
      }
      
      // 3. Position/velocity remain zero (setIdentity), accel/gyro biases injected
      // Initialize baro bias to ground altitude for proper AGL measurement
      s.b_baro = entry.data.liftoff.ground_reference_valid 
                 ? entry.data.liftoff.ground_altitude_m 
                 : 0;
      s.timestamp_us = entry.timestamp_us;
      
      core_.setState(s);
      core_.resetIntegrationState();
      
      // 4. Build initial covariance with tight uncertainties
      Covariance cov;
      eskf_scalar diag[kDimError];
      InitialCovariance p0 = InitialCovariance::defaults();
      
      // Use tight position/velocity since we're at origin
      p0.pos = 0.1;        // 0.1m - we're defining this as origin
      p0.vel = 0.01;       // 0.01m/s - stationary
      p0.tilt = 0.01;      // ~0.5 deg - well converged from Mahony
      p0.accel_bias = 0.0; // Start from injected turn-on estimate; keep tight prior
      p0.gyro_bias = 0.001; // 0.001 rad/s - well converged from LPF
      p0.baro_bias = 5.0;  // 5 m — allows filter to quickly absorb residual
                            // b_baro mismatch from sensor warm-up drift
      
      // Use heading variance from RailShadow (or default if not initialized)
      p0.heading = entry.data.liftoff.heading_initialized 
                   ? std::sqrt(entry.data.liftoff.heading_variance)
                   : 0.5;  // ~30 deg fallback
      
      p0.toDiagonal(diag);
      cov.setDiagonal(diag);
      core_.setCovariance(cov);
      
      // 5. Transfer heading initialization state
      core_.setHeadingInitialized(entry.data.liftoff.heading_initialized);
      
      // 6. Freeze bias random walk (Q=0 for biases)
      // Note: This requires a method on core_ - we'll set it via processNoise
      // For now, the bias values are locked in state; covariance growth is minimal
      // with tight initial variance
      
      break;
    }
      
    case EventType::GpsPacket: {
      // Phase 4: Reject GPS during replay and initial liftoff
      if (entry.timestamp_us < rejection_end_us_) break;
      
      const auto& pkt = entry.data.gps_packet;
      bool packet_mutated_state = false;
      
      // === R INFLATION FACTORS ===
      // Compute inflated R values based on:
      // 1. ESKF_GPS_TRUST_FACTOR - GPS receivers are typically optimistic
      // 2. ESKF_GPS_HIGH_G_R_FACTOR - During motor burn, GPS PLL can slip
      
      eskf_scalar r_multiplier = tuning_cfg_.gps_trust_factor;
      
      // High-G inflation: when acceleration exceeds threshold, further inflate R
      if (last_accel_magnitude_g_ > tuning_cfg_.gps_high_g_threshold) {
        r_multiplier *= tuning_cfg_.gps_high_g_r_factor;
      }
      
      // Compute inflated R arrays (variance scales with square of multiplier)
      eskf_scalar r_mult_sq = r_multiplier * r_multiplier;
      eskf_scalar R_vel_inflated[3] = {
        pkt.R_vel[0] * r_mult_sq,
        pkt.R_vel[1] * r_mult_sq,
        pkt.R_vel[2] * r_mult_sq
      };
      eskf_scalar R_pos_inflated[3] = {
        0,
        0,
        0
      };
      if (pkt.has_position) {
        R_pos_inflated[0] = pkt.R_pos[0] * r_mult_sq;
        R_pos_inflated[1] = pkt.R_pos[1] * r_mult_sq;
        R_pos_inflated[2] = pkt.R_pos[2] * r_mult_sq;
      }
      
      // === GPS COG HEADING BOOTSTRAP CHECK ===
      // Optional mode: for aircraft-like motion, bootstrap heading from GPS COG.
      // For tumbling rockets, this mode is disabled and GNSS pos/vel updates are
      // still allowed without forcing COG to body yaw.
      if (tuning_cfg_.enable_gps_cog_heading && !core_.isHeadingAligned()) {
        // Try one-shot heading bootstrap first.
        if (pkt.has_velocity) {
          // Gate 1: Minimum horizontal speed (15 m/s for alignment)
          eskf_scalar v_horiz_sq = pkt.vel[0]*pkt.vel[0] + pkt.vel[1]*pkt.vel[1];
          eskf_scalar v_horiz = std::sqrt(v_horiz_sq);
          eskf_scalar sAcc = std::sqrt(pkt.R_vel[0]) * tuning_cfg_.gps_trust_factor;
          
          if (v_horiz >= tuning_cfg_.heading_align_min_speed && sAcc < tuning_cfg_.heading_align_max_sacc) {
            // attemptHeadingAlignment now passes GPS velocity to forceYaw
            if (core_.attemptHeadingAlignment(pkt.vel, sAcc)) {
              // Heading alignment is an instantaneous state injection.
              // Reset integration history to avoid mixing pre/post-snap
              // samples in coning/sculling/trapezoidal accumulators.
              core_.resetIntegrationState();
              flushUnreadBaroAfterStateReset(entry.timestamp_us);
              packet_mutated_state = true;
            }
          }
        }

        // If still unaligned, skip this packet exactly as before.
        // If bootstrap succeeded, continue same-packet processing so first
        // position reset/velocity fusion can happen deterministically now.
        if (!core_.isHeadingAligned()) {
          if (packet_mutated_state) {
            saveCheckpointNow();
          }
          break;
        }
      }
      
      // Correct GPS NED position with cumulative heading-snap offset.
      // forceYaw rotates the ESKF position when heading snaps occur,
      // but GPS NED values (computed from the pre-rotation origin) remain
      // in the original frame. This constant offset keeps them frame-consistent
      // while preserving velocity direction (unlike a full rotation).
      eskf_scalar corrected_pos[3] = {0, 0, 0};
      if (pkt.has_position) {
        corrected_pos[0] = pkt.pos[0];
        corrected_pos[1] = pkt.pos[1];
        corrected_pos[2] = pkt.pos[2];
        core_.correctGpsNed(corrected_pos);

        // Convert antenna position measurement to equivalent CG position so
        // position resets/updates do not carry static lever-arm offset.
        eskf_scalar lever_arm_ned[3] = {};
        math::quatRotateVector(lever_arm_ned, core_.state().q, pkt.lever_arm);
        for (int i = 0; i < 3; ++i) {
          corrected_pos[i] -= lever_arm_ned[i];
        }
      }

      // Tumble-rate attitude decoupling: when angular rate is high, zero
      // P cross-terms between attitude/gyrbias and velocity/position before
      // GPS corrections. This makes K_att = 0 (since K = P*H'/S and
      // P_att_vel = 0), so velocity corrections can't affect attitude.
      // State-covariance consistent: P_att stays accurate for heading fusion.
      if (tuning_cfg_.gps_vel_tumble_gyro_threshold > 0 &&
          core_.prevGyroMagnitude() > tuning_cfg_.gps_vel_tumble_gyro_threshold) {
        core_.decoupleAttitudeFromVelocity();
      }

      // Step 1: Check for covariance inflation trigger
      if (consecutive_gps_rejects_ >= tuning_cfg_.gps_max_consecutive_rejects) {
        core_.inflatePositionCovariance(tuning_cfg_.gps_reset_p_pos);
        core_.inflateVelocityCovariance(tuning_cfg_.gps_reset_p_vel);
        consecutive_gps_rejects_ = 0;
        stats_.gps_covariance_resets++;
        packet_mutated_state = true;
        
        // Log covariance reset
        GpsRejectionInfo info{};
        info.consecutive = tuning_cfg_.gps_max_consecutive_rejects;
        info.reason = GpsRejectReason::CovarianceResetThreshold;
        getEskfLogger().logGpsRejection(EskfEventType::GpsCovarianceReset,
                                        entry.timestamp_us, info);
      }
      
        // Step 2: Compute velocity innovation Chi-Squared (using inflated R)
        // Soft-accept moderately out-of-gate packets with adaptive R inflation to
        // avoid long rejection plateaus and late hard snaps.
        bool vel_passed = !pkt.has_velocity;
        bool vel_strict_passed = !pkt.has_velocity;
      if (pkt.has_velocity) {
        const eskf_scalar kGpsVelSoftAcceptMultiplier = tuning_cfg_.gps_vel_soft_accept_multiplier;
        const eskf_scalar vel_threshold = tuning_cfg_.gps_vel_chi2_threshold;
        eskf_scalar vel_innovations[3], vel_S[3];
        eskf_scalar vel_chi2 = core_.computeGpsVelocityInnovation(
            pkt.vel, R_vel_inflated, pkt.lever_arm,
            vel_innovations, vel_S);
        const bool vel_pass_strict = vel_chi2 < vel_threshold;
        const bool vel_pass_soft =
          !vel_pass_strict && vel_threshold > 1e-6f &&
          vel_chi2 < (vel_threshold * kGpsVelSoftAcceptMultiplier);
        core_.setGpsVelocityGateDebug(
          vel_chi2, vel_threshold, vel_pass_strict || vel_pass_soft);

        if (vel_pass_strict || vel_pass_soft) {
          vel_passed = true;
          vel_strict_passed = vel_pass_strict;
          consecutive_gps_rejects_ = 0;

          eskf_scalar R_vel_used[3] = {
            R_vel_inflated[0], R_vel_inflated[1], R_vel_inflated[2]};
          if (vel_pass_soft) {
          const eskf_scalar scale = std::max(
              static_cast<eskf_scalar>(1.0f),
              std::min(vel_chi2 / vel_threshold,
                       static_cast<eskf_scalar>(kGpsVelSoftAcceptMultiplier)));
          R_vel_used[0] *= scale;
          R_vel_used[1] *= scale;
          R_vel_used[2] *= scale;
          }
          
          // Step 3: Update velocity (velocity first!) with inflated R
#if ESKF_USE_LEVER_ARM_AVERAGING
          eskf_scalar avg_lever_vel_ned[3];
          uint64_t window_us = static_cast<uint64_t>(cfg_.gpsDelayUs < 0 ? -cfg_.gpsDelayUs : cfg_.gpsDelayUs);
      const bool allow_averaging =
        tuning_cfg_.gps_vel_tumble_gyro_threshold <= 0 ||
        core_.prevGyroMagnitude() <= tuning_cfg_.gps_vel_tumble_gyro_threshold;
          
      bool has_average = false;
      if (allow_averaging) {
      has_average = computeAveragedLeverArmVelocity(
        pkt.lever_arm, entry.timestamp_us, window_us, avg_lever_vel_ned);
      }
          
          if (has_average) {
            core_.correctGpsVelocityWithAveragedLeverArm(
                pkt.vel, R_vel_used, pkt.lever_arm, avg_lever_vel_ned);
          } else {
            core_.correctGpsVelocity(pkt.vel, R_vel_used, pkt.lever_arm);
          }
#else
          core_.correctGpsVelocity(pkt.vel, R_vel_used, pkt.lever_arm);
#endif
          packet_mutated_state = true;
        } else {
          // Velocity failed Chi-Squared - reject entire packet.
          consecutive_gps_rejects_++;
          stats_.gps_vel_rejects++;
          
          // Log velocity rejection
          GpsRejectionInfo info{};
          info.chi2_vel = static_cast<float>(vel_chi2);
          info.threshold = static_cast<float>(tuning_cfg_.gps_vel_chi2_threshold);
          info.consecutive = consecutive_gps_rejects_;
          info.reason = GpsRejectReason::VelocityChi2;
          getEskfLogger().logGpsRejection(EskfEventType::GpsVelRejected,
                                          entry.timestamp_us, info);
          break;
        }
      }

      // === FIRST POSITION RESET (Quadratic Error Fix) ===
      // After heading alignment, the first GPS position is used to "teleport"
      // the position state to clear accumulated quadratic integration error.
      // This reset is now gated on velocity acceptance when velocity exists,
      // preventing bad packets from locking an incorrect first-position state.
      const bool heading_ready_for_first_pos =
          (!tuning_cfg_.enable_gps_cog_heading) || core_.isHeadingAligned();
      const bool velocity_ready_for_first_pos =
          (!pkt.has_velocity) || vel_passed;
      if (!has_fused_first_pos_ && heading_ready_for_first_pos && pkt.has_position &&
          velocity_ready_for_first_pos &&
          last_accel_magnitude_g_ < tuning_cfg_.gps_high_g_threshold) {
        State s = core_.state();
        const eskf_scalar prev_p_d = s.p[2];
        s.p[0] = corrected_pos[0];
        s.p[1] = corrected_pos[1];
        s.p[2] = corrected_pos[2];
        // Preserve baro measurement-model continuity h=-p_d+b_baro across
        // hard position snaps to avoid large immediate baro innovations.
        s.b_baro += (s.p[2] - prev_p_d);
        core_.setState(s);

        // Position is hard-overwritten above, so use a covariance block reset.
        core_.resetPositionCovariance(R_pos_inflated[0]);
        flushUnreadBaroAfterStateReset(entry.timestamp_us);
        has_fused_first_pos_ = true;
        last_gps_position_accept_us_ = entry.timestamp_us;
        packet_mutated_state = true;

        // No velocity in packet: we're done after first-pos reset.
        if (!pkt.has_velocity) {
          saveCheckpointNow();
          break;
        }
      }

      // Reacquire hard reset: after a prolonged gap without accepted GPS
      // position updates, snap position back to current GNSS once velocity
      // gating is healthy. This avoids long position-reject plateaus after
      // deploy-induced outages while keeping replay determinism.
      constexpr uint64_t kGpsPositionHardReacquireGapUs = 1500000ULL;
      const bool long_position_gap =
          has_fused_first_pos_ &&
          last_gps_position_accept_us_ > 0 &&
          entry.timestamp_us >
              (last_gps_position_accept_us_ + kGpsPositionHardReacquireGapUs);
      if (pkt.has_position && vel_passed && long_position_gap &&
          last_accel_magnitude_g_ < tuning_cfg_.gps_high_g_threshold) {
        State s = core_.state();
        const eskf_scalar prev_p_d = s.p[2];
        s.p[0] = corrected_pos[0];
        s.p[1] = corrected_pos[1];
        s.p[2] = corrected_pos[2];
        // Preserve baro predicted altitude continuity across reacquire reset.
        s.b_baro += (s.p[2] - prev_p_d);
        core_.setState(s);
        core_.resetPositionCovariance(R_pos_inflated[0]);
        flushUnreadBaroAfterStateReset(entry.timestamp_us);
        last_gps_position_accept_us_ = entry.timestamp_us;
        packet_mutated_state = true;
      }

      // === OPTIONAL CONTINUOUS GPS COG HEADING FUSION ===
      // Require strictly accepted velocity so marginal soft-accepted packets
      // cannot drive heading updates.
      const bool allow_gps_cog_heading_fusion =
          tuning_cfg_.enable_gps_cog_heading_fusion &&
          tuning_cfg_.gps_heading_bootstrap_mode !=
              GpsHeadingBootstrapMode::VelocityAngleDelta;
      if (allow_gps_cog_heading_fusion && pkt.has_velocity && vel_strict_passed) {
        eskf_scalar v_horiz_sq = pkt.vel[0] * pkt.vel[0] + pkt.vel[1] * pkt.vel[1];
        eskf_scalar v_horiz = std::sqrt(v_horiz_sq);
        eskf_scalar sAcc = std::sqrt(pkt.R_vel[0]) * tuning_cfg_.gps_trust_factor;
        if (v_horiz >= tuning_cfg_.heading_align_min_speed &&
            sAcc < tuning_cfg_.heading_align_max_sacc &&
            last_accel_magnitude_g_ < tuning_cfg_.gps_high_g_threshold) {
          eskf_scalar gps_heading = std::atan2(pkt.vel[1], pkt.vel[0]);
          const HeadingUpdateResult heading_result =
              core_.processHeadingUpdate(gps_heading,
                                         tuning_cfg_.gps_heading_variance,
                                         EskfEventType::GpsHeadingCorrection,
                                         false);
          if (heading_result == HeadingUpdateResult::Snapped ||
              heading_result == HeadingUpdateResult::Resurrected) {
            flushUnreadBaroAfterStateReset(entry.timestamp_us);
          }
          packet_mutated_state = true;
        }
      }

      // Diagnostic: after heading alignment, skip vel/pos corrections
      // (heading fusion above still runs)
      if (tuning_cfg_.disable_gps_after_alignment && core_.isHeadingAligned()) {
        if (packet_mutated_state) {
          saveCheckpointNow();
        }
        break;
      }
      
      // Step 4: Compute position innovation Chi-Squared (after first reset, using inflated R)
      if (pkt.has_position && vel_passed && has_fused_first_pos_) {
        const eskf_scalar kGpsPosSoftAcceptMultiplier = tuning_cfg_.gps_pos_soft_accept_multiplier;
        const eskf_scalar pos_threshold = tuning_cfg_.gps_pos_chi2_threshold;
        eskf_scalar pos_innovations[3], pos_S[3];
        eskf_scalar pos_chi2 = core_.computeGpsPositionInnovation(
            corrected_pos, R_pos_inflated, pos_innovations, pos_S);

        const bool pos_pass_strict = pos_chi2 < pos_threshold;
        const bool pos_pass_soft =
            !pos_pass_strict && pos_threshold > 1e-6f &&
            pos_chi2 < (pos_threshold * kGpsPosSoftAcceptMultiplier);

        if (pos_pass_strict || pos_pass_soft) {
          eskf_scalar R_pos_used[3] = {
              R_pos_inflated[0], R_pos_inflated[1], R_pos_inflated[2]};
          if (pos_pass_soft) {
            const eskf_scalar scale = std::max(
					static_cast<eskf_scalar>(1.0f),
					std::min(pos_chi2 / pos_threshold,
							 static_cast<eskf_scalar>(kGpsPosSoftAcceptMultiplier)));
            R_pos_used[0] *= scale;
            R_pos_used[1] *= scale;
            R_pos_used[2] *= scale;
          }
          core_.correctGpsPosition(corrected_pos, R_pos_used);
          last_gps_position_accept_us_ = entry.timestamp_us;
          packet_mutated_state = true;
        } else {
          stats_.gps_pos_rejects++;
          
          // Log position rejection
          GpsRejectionInfo info{};
          info.chi2_pos = static_cast<float>(pos_chi2);
          info.threshold = static_cast<float>(tuning_cfg_.gps_pos_chi2_threshold);
          info.reason = GpsRejectReason::PositionChi2;
          getEskfLogger().logGpsRejection(EskfEventType::GpsPosRejected,
                                          entry.timestamp_us, info);
        }
      }

      if (packet_mutated_state) {
        saveCheckpointNow();
      }

      break;
    }
      
    case EventType::Sideslip:
#if ESKF_ENABLE_SIDESLIP
      core_.correctSideslip(entry.data.sideslip.R_lateral);
#endif
      break;
  }
  
  event_read_idx_++;
}

// ============================================================
// Push Event to Buffer
// ============================================================

void EskfYieldable::pushEvent(const EventEntry& e) {
  // (m4) Validate monotonic timestamps
  if (event_count_ > 0 && e.timestamp_us < last_event_timestamp_) {
    stats_.out_of_order_events++;
    // Still push the event - just flag it for debugging
  }
  last_event_timestamp_ = e.timestamp_us;

  const bool reserve_for_liftoff = liftoff_snap_pending_ &&
                                   e.type != EventType::LiftoffSnap;
  if (reserve_for_liftoff && event_count_ >= (ESKF_EVENT_BUFFER_SIZE - 1)) {
    stats_.event_drops++;
    constexpr uint64_t kOverflowLogIntervalUs = 1000000;
    if (e.timestamp_us - last_event_overflow_log_us_ >= kOverflowLogIntervalUs) {
      last_event_overflow_log_us_ = e.timestamp_us;
      getEskfLogger().logEvent(EskfEventType::EventBufferOverflow,
                               e.timestamp_us,
                               static_cast<float>(stats_.event_drops));
    }
    return;
  }
  
  event_buffer_[event_head_] = e;
  
  event_head_ = (event_head_ + 1) % ESKF_EVENT_BUFFER_SIZE;
  if (event_count_ < ESKF_EVENT_BUFFER_SIZE) {
    event_count_++;
  } else {
    if (hibernating_ || in_rewind_) {
      // Pre-liftoff hibernation and active rewind can naturally lap old
      // events while replay cursor movement is constrained by mode semantics.
      // Do not count this as data loss.
    } else if (event_read_idx_ > 0) {
      // Overwriting an already-processed event - normal ring-buffer behavior.
      // Keep read index relative to the new oldest entry.
      event_read_idx_--;
    } else {
      // REAL OVERFLOW: event_read_idx_ == 0 means we're overwriting unread
      // events because catchUp isn't keeping up.
      stats_.event_drops++;
      // Rate-limit overflow logging to 1 Hz
      constexpr uint64_t kOverflowLogIntervalUs = 1000000;
      if (e.timestamp_us - last_event_overflow_log_us_ >= kOverflowLogIntervalUs) {
        last_event_overflow_log_us_ = e.timestamp_us;
        getEskfLogger().logEvent(EskfEventType::EventBufferOverflow,
                                 e.timestamp_us,
                                 static_cast<float>(stats_.event_drops));
      }
    }
  }
  
  // (I2) Track high-water mark
  if (event_count_ > stats_.event_max_usage) {
    stats_.event_max_usage = event_count_;
  }
}

void EskfYieldable::flushUnreadBaroAfterStateReset(uint64_t context_ts) {
#if ESKF_USE_GPS_REWIND
  const size_t unread = (baro_count_ > baro_read_idx_)
                            ? (baro_count_ - baro_read_idx_)
                            : 0;
  if (unread == 0 && !has_pending_baro_) {
    return;
  }

  stats_.baro_drops += static_cast<uint32_t>(unread);

  // Clear full unread baro queue to prevent stale trigger-time snapshots from
  // being fused after discrete state/covariance resets.
  baro_head_ = 0;
  baro_count_ = 0;
  baro_read_idx_ = 0;
  has_pending_baro_ = false;
  pending_baro_slot_ = 0;

  getEskfLogger().logEvent(EskfEventType::BaroQueueFlushed,
                           context_ts,
                           static_cast<float>(unread));
#else
  (void)context_ts;
#endif
}

// ============================================================
// Lever Arm Averaging
// ============================================================

bool EskfYieldable::computeAveragedLeverArmVelocity(
    const eskf_scalar lever_arm_body[3],
    uint64_t gps_timestamp_us,
    uint64_t window_us,
    eskf_scalar avg_lever_vel_ned[3]) const {
  
  // Initialize accumulator
  avg_lever_vel_ned[0] = 0;
  avg_lever_vel_ned[1] = 0;
  avg_lever_vel_ned[2] = 0;
  
  if (imu_push_seq_ == 0) return false;
  
  // Find window boundaries
  uint64_t window_start = (gps_timestamp_us > window_us) ? (gps_timestamp_us - window_us) : 0;
  
  // Iterate through IMU buffer to find samples in window
  const size_t count = imuBufferCount();
  size_t oldest_idx = (imu_push_seq_ - count) % ESKF_IMU_BUFFER_SIZE;
  uint32_t sample_count = 0;
  
  // We need attitude at each sample time. Since we can't easily reconstruct
  // attitude at arbitrary times, we use the current attitude as an approximation.
  // For high-spin rockets, the Earth-frame lever arm rotates, so this produces
  // the averaging effect we need even with a constant body-frame lever arm.
  const eskf_scalar* q = core_.state().q;
  
  for (size_t i = 0; i < count; ++i) {
    size_t buf_idx = (oldest_idx + i) % ESKF_IMU_BUFFER_SIZE;
    const ImuEntry& entry = imu_buffer_[buf_idx];
    
    // Check if sample is in the averaging window
    if (entry.imu.timestamp_us >= window_start && entry.imu.timestamp_us <= gps_timestamp_us) {
      // Get gyro (bias-corrected using current bias estimate)
      eskf_scalar gyro[3];
      for (int j = 0; j < 3; ++j) {
        gyro[j] = entry.imu.gyro[j] - core_.state().b_gyro[j];
      }
      
      // Compute lever arm velocity in body frame: ω × r
      eskf_scalar v_arm_body[3];
      v_arm_body[0] = gyro[1] * lever_arm_body[2] - gyro[2] * lever_arm_body[1];
      v_arm_body[1] = gyro[2] * lever_arm_body[0] - gyro[0] * lever_arm_body[2];
      v_arm_body[2] = gyro[0] * lever_arm_body[1] - gyro[1] * lever_arm_body[0];
      
      // Rotate to NED frame using current attitude
      eskf_scalar v_arm_ned[3];
      math::quatRotateVector(v_arm_ned, q, v_arm_body);
      
      // Accumulate
      avg_lever_vel_ned[0] += v_arm_ned[0];
      avg_lever_vel_ned[1] += v_arm_ned[1];
      avg_lever_vel_ned[2] += v_arm_ned[2];
      sample_count++;
    }
    
    // Early exit if past window
    if (entry.imu.timestamp_us > gps_timestamp_us) break;
  }
  
  // Compute average
  if (sample_count >= 3) {  // Minimum samples for meaningful average
    eskf_scalar inv_count = static_cast<eskf_scalar>(1.0) / static_cast<eskf_scalar>(sample_count);
    avg_lever_vel_ned[0] *= inv_count;
    avg_lever_vel_ned[1] *= inv_count;
    avg_lever_vel_ned[2] *= inv_count;
    return true;
  }
  
  // Not enough samples - caller should use fallback
  return false;
}

RewindCheckpoint EskfYieldable::captureRewindCheckpoint() const {
  RewindCheckpoint cp{};
  cp.core = core_.createCheckpoint();
  cp.timestamp_us = cp.core.timestamp_us;
  cp.consecutive_gps_rejects = consecutive_gps_rejects_;
  cp.imu_since_checkpoint = imu_since_checkpoint_;
  cp.has_fused_first_pos = has_fused_first_pos_;
  cp.last_gps_position_accept_us = last_gps_position_accept_us_;
  cp.last_accel_magnitude_g = last_accel_magnitude_g_;
  return cp;
}

void EskfYieldable::restoreRewindCheckpoint(const RewindCheckpoint& cp) {
  core_.restoreCheckpoint(cp.core);
  consecutive_gps_rejects_ = cp.consecutive_gps_rejects;
  imu_since_checkpoint_ = cp.imu_since_checkpoint;
  has_fused_first_pos_ = cp.has_fused_first_pos;
  last_gps_position_accept_us_ = cp.last_gps_position_accept_us;
  last_accel_magnitude_g_ = cp.last_accel_magnitude_g;
}

// ============================================================
// Logging Helpers
// ============================================================

void EskfYieldable::logStateIfDue() {
  // Create and log state snapshot
  // The rate limiting is handled by the logger implementation
  getEskfLogger().logState(core_.createStateSnapshot());
}

void EskfYieldable::logCovarianceIfDue() {
  // Create and log covariance snapshot
  // The rate limiting is handled by the logger implementation
  getEskfLogger().logCovariance(core_.createCovarianceSnapshot());
}

} // namespace eskf
