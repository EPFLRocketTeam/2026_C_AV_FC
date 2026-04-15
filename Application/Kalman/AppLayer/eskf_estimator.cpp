#include "Application/Kalman/AppLayer/eskf_estimator.hpp"

#if APP_TARGET_TEENSY || APP_TARGET_NATIVE

#include "Application/Kalman/AppLayer/hw_calibration_data.hpp"
#include "Application/Kalman/AppLayer/hw_config.hpp"
#include "Application/Kalman/kalman/eskf_logger.hpp"
#include "Application/Kalman/kalman/eskf_math.hpp"
#include "Application/Kalman/kalman/preprocessor/pressure_altitude.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

// GnssSample and MmcSample are already defined in sensor_manager.hpp
// which is included via state_estimator.hpp

namespace app {

namespace {

constexpr uint16_t kSingleSensorHardSuspectSamples = 1;
constexpr uint16_t kSingleSensorHardPersistenceSamples = 2;
constexpr uint16_t kSingleSensorRecoveryCooldownSamples = 3;
constexpr uint16_t kSingleSensorRecoveryConfirmSamples = 3;
constexpr uint16_t kMagStalePersistenceSamples = 64;
constexpr uint16_t kGnssStalePersistenceSamples = 3;

} // namespace

namespace {

// Earth parameters for GPS→NED conversion
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

} // namespace

void EskfEstimator::virtualImuCgCallback(void *user_data,
                                         uint64_t timestamp_us,
                                         eskf_scalar cg_out[3]) {
  auto *self = static_cast<EskfEstimator *>(user_data);
  if (!self) {
    cg_out[0] = 0;
    cg_out[1] = 0;
    cg_out[2] = 0;
    return;
  }
  self->interpolateCgAtTimestamp(timestamp_us, cg_out);
}

void EskfEstimator::interpolateCgAtTimestamp(uint64_t timestamp_us,
                                             eskf_scalar cg_out[3]) const {
  const uint8_t n = calib_cfg_.cg_table_size;
  if (n == 0) {
    cg_out[0] = 0;
    cg_out[1] = 0;
    cg_out[2] = 0;
    return;
  }

  if (n == 1) {
    cg_out[0] = static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[0]);
    cg_out[1] = static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[1]);
    cg_out[2] = static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[2]);
    return;
  }

    const uint64_t rel_ms =
      (in_flight_ && (timestamp_us > liftoff_us_))
        ? ((timestamp_us - liftoff_us_) / 1000ULL)
        : 0ULL;

  // Clamp before first point.
  if (rel_ms <= calib_cfg_.cg_table[0].time_ms) {
    cg_out[0] = static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[0]);
    cg_out[1] = static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[1]);
    cg_out[2] = static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[2]);
    return;
  }

  // Find interpolation segment.
  for (uint8_t i = 1; i < n; ++i) {
    const uint32_t t0 = calib_cfg_.cg_table[i - 1].time_ms;
    const uint32_t t1 = calib_cfg_.cg_table[i].time_ms;
    if (rel_ms <= t1) {
      const eskf_scalar denom = static_cast<eskf_scalar>(
          (t1 > t0) ? (t1 - t0) : 1U);
      const eskf_scalar alpha = static_cast<eskf_scalar>(
          (rel_ms > t0) ? (rel_ms - t0) : 0ULL) /
          denom;
      for (int j = 0; j < 3; ++j) {
        const eskf_scalar p0 =
            static_cast<eskf_scalar>(calib_cfg_.cg_table[i - 1].position[j]);
        const eskf_scalar p1 =
            static_cast<eskf_scalar>(calib_cfg_.cg_table[i].position[j]);
        cg_out[j] = p0 + alpha * (p1 - p0);
      }
      return;
    }
  }

  // Clamp after last point.
  cg_out[0] = static_cast<eskf_scalar>(calib_cfg_.cg_table[n - 1].position[0]);
  cg_out[1] = static_cast<eskf_scalar>(calib_cfg_.cg_table[n - 1].position[1]);
  cg_out[2] = static_cast<eskf_scalar>(calib_cfg_.cg_table[n - 1].position[2]);
}

EskfEstimator::EskfEstimator(hal::ILogSink *log) : log_(log) {
  // Initialize configuration with compile-time defaults
  // This ensures valid scaling/tuning even if configure() is not called
  static_cast<appcfg::ReplayableConfig &>(cfg_) = appcfg::getDefaultConfig();
  calib_cfg_ = appcfg::getDefaultCalibrationConfig();
}

void EskfEstimator::configure(const Config &cfg) { cfg_ = cfg; }

void EskfEstimator::configureReplaySensorCounts(size_t imu_count,
                        size_t baro_count) {
  active_imu_sources_ =
    std::max<size_t>(1, std::min<size_t>(imu_count, ESKF_MAX_IMUS));
  active_baro_sources_ =
    std::min<size_t>(baro_count, ESKF_MAX_BAROS);
}

void EskfEstimator::configureCalibration(const appcfg::CalibrationConfig &cfg) {
  calib_cfg_ = cfg;
  calib_cfg_set_ = true;
}

void EskfEstimator::reset() {
  initialized_ = false;
  in_flight_ = false;
  rail_shadow_initialized_ = false;
  liftoff_ms_ = 0;
  liftoff_us_ = 0;
  was_aero_blind_ = false;
  baro_reacquire_needed_ = false;
  last_flight_shadow_predict_us_ = 0;
  last_valid_inflight_imu_us_ = 0;
  sustained_imu_outage_handoff_todo_logged_ = false;
  catchup_yield_count_ = 0;
  sideslip_decimation_counter_ = 0;
  gps_origin_set_ = false;
  latest_preflight_gps_anchor_ = {};
  has_liftoff_snapshot_ = false;
  last_liftoff_snapshot_ = LiftoffSnapshot{};
  last_logged_imu_salvage_ = false;
  last_logged_baro_salvage_ = false;
  for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
    imu_health_logged_init_[i] = false;
    last_logged_imu_health_[i] = eskf::SensorHealthState::ACTIVE;
  }
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    baro_health_logged_init_[i] = false;
    last_logged_baro_health_[i] = eskf::BaroHealthState::ACTIVE;
  }
  output_dirty_ = true;
  descent_mode_active_ = false;
  descent_waiting_initial_gnss_snap_ = false;
  descent_last_gnss_fuse_us_ = 0;
  latest_descent_gnss_ = LatestGnssForDescent{};
  latest_descent_baro_ = LatestBaroForDescent{};
  descent_filter_.configure(DescentNavFilter::Config{});
  descent_filter_.reset();
  for (size_t i = 0; i < kMaxImuSources; ++i) {
    pending_imu_[i] = PendingImuBatch{};
  }
  for (size_t i = 0; i < kMaxBaroSources; ++i) {
    pending_baro_[i] = PendingBaroSample{};
  }
  resetTurnOnAccelBiasEstimator();
  resetAuxSensorStaleEstimators();
  // Reset shadow filters with default tuning
  eskf::TuningConfig tuning = eskf::getDefaultTuningConfig();
  rail_shadow_.init(tuning);
  flight_shadow_.init(tuning);

  // Setup preprocessors
  setupVirtualImu();
  setupVirtualBaro();
  setupVirtualCompass();

  // Initialize main filter
  initializeFilter();

  initialized_ = true;

  if (log_) {
    log_->println("[ESKF] Estimator reset complete");
  }
}

void EskfEstimator::resetAuxSensorStaleEstimators() {
  mag_health_state_ = eskf::SensorHealthState::ACTIVE;
  mag_hard_fault_counter_ = 0;
  mag_healthy_counter_ = 0;
  mag_stale_counter_ = 0;
  mag_has_prev_sample_ = false;
  mag_prev_x_ = 0;
  mag_prev_y_ = 0;
  mag_prev_z_ = 0;

  gnss_health_state_ = eskf::SensorHealthState::ACTIVE;
  gnss_hard_fault_counter_ = 0;
  gnss_healthy_counter_ = 0;
  gnss_stale_counter_ = 0;
  gnss_has_prev_sample_ = false;
  gnss_prev_sample_ = {};
  gnss_fix_logged_init_ = false;
  last_logged_gnss_fix_usable_ = false;
  last_gnss_fix_drop_log_us_ = 0;
}

bool EskfEstimator::updateSingleSensorHealthFromHardObservation(
    bool hard_observation, eskf::SensorHealthState &state,
    uint16_t &hard_fault_counter, uint16_t &healthy_counter,
    uint16_t hard_fault_suspect_samples,
    uint16_t hard_fault_persistence_samples,
    uint16_t recovery_cooldown_samples, uint16_t recovery_confirm_samples) {
  switch (state) {
  case eskf::SensorHealthState::INHIBITED:
    if (!hard_observation) {
      if (healthy_counter < std::numeric_limits<uint16_t>::max()) {
        healthy_counter++;
      }
      if (healthy_counter >= recovery_cooldown_samples) {
        state = eskf::SensorHealthState::RECOVERING;
        healthy_counter = 0;
      }
    } else {
      healthy_counter = 0;
    }
    break;
  case eskf::SensorHealthState::RECOVERING:
    if (hard_observation) {
      state = eskf::SensorHealthState::INHIBITED;
      healthy_counter = 0;
    } else {
      if (healthy_counter < std::numeric_limits<uint16_t>::max()) {
        healthy_counter++;
      }
      if (healthy_counter >= recovery_confirm_samples) {
        state = eskf::SensorHealthState::ACTIVE;
        healthy_counter = 0;
        hard_fault_counter = 0;
      }
    }
    break;
  case eskf::SensorHealthState::ACTIVE:
  case eskf::SensorHealthState::SUSPECT:
  default:
    if (hard_observation) {
      if (hard_fault_counter < std::numeric_limits<uint16_t>::max()) {
        hard_fault_counter++;
      }
      healthy_counter = 0;
    } else {
      if (hard_fault_counter > 0) {
        hard_fault_counter--;
      }
      if (state == eskf::SensorHealthState::SUSPECT) {
        if (healthy_counter < std::numeric_limits<uint16_t>::max()) {
          healthy_counter++;
        }
        if (healthy_counter >= recovery_confirm_samples) {
          state = eskf::SensorHealthState::ACTIVE;
          healthy_counter = 0;
        }
      } else {
        healthy_counter = 0;
      }
    }

    if (hard_fault_counter >= hard_fault_persistence_samples) {
      state = eskf::SensorHealthState::INHIBITED;
      healthy_counter = 0;
    } else if (hard_fault_counter >= hard_fault_suspect_samples) {
      state = eskf::SensorHealthState::SUSPECT;
    }
    break;
  }

  return state == eskf::SensorHealthState::INHIBITED ||
         state == eskf::SensorHealthState::RECOVERING || hard_observation;
}

bool EskfEstimator::shouldStartEskfBaroFusion(uint64_t timestamp_us) {
  (void)timestamp_us;
  if (!in_flight_) {
    was_aero_blind_ = false;
    baro_reacquire_needed_ = false;
    return false;
  }

  const bool aero_blind_now = flight_shadow_.isAeroBlind();
  if (aero_blind_now) {
    was_aero_blind_ = true;
    return false;
  }

  // Transition out of aero-blind: require one-shot vertical reacquisition
  // before re-enabling normal ESKF baro fusion.
  if (was_aero_blind_) {
    was_aero_blind_ = false;
    baro_reacquire_needed_ = true;
  }

  // Require a one-shot vertical reacquisition before normal baro fusion.
  return !baro_reacquire_needed_;
}

void EskfEstimator::performBaroReacquisition(eskf_scalar altitude_isa_m,
                                             uint64_t timestamp_us) {
  auto &core = filter_.core();

  const eskf::TuningConfig tuning = cfg_.toTuningConfig();
  const eskf::State &s = core.state();
  const eskf_scalar v_sq =
      s.v[0] * s.v[0] + s.v[1] * s.v[1] + s.v[2] * s.v[2];
  const eskf_scalar speed = std::sqrt(v_sq);
  eskf_scalar sigma = tuning.baro_sigma_base + tuning.baro_k_aero * v_sq;
  if (speed > tuning.baro_transonic_low && speed < tuning.baro_transonic_high) {
    sigma += tuning.baro_transonic_penalty;
  }
  const eskf_scalar baro_reacq_var = sigma * sigma;

  core.resetVerticalChannelFromBaro(altitude_isa_m, baro_reacq_var);
  // Reacquisition snaps position; inflate velocity uncertainty so subsequent
  // IMU+baro cycles can re-identify vertical rate without overconfidence.
  core.inflateVelocityCovariance(tuning.gps_reset_p_vel);
  // Bias can drift during aero-blind/transonic thermal transients.
  // Raise baro-bias uncertainty so post-reacquisition updates can re-learn it.
  core.inflateBaroBiasCovariance(baro_reacq_var);

  eskf::State snapped = core.state();
  snapped.timestamp_us = timestamp_us;
  core.setState(snapped);
  core.resetIntegrationState();

  baro_reacquire_needed_ = false;
}

void EskfEstimator::initializeFilter() {
  // Compute local gravity from launch latitude
  // Note: ReplayableConfig already has local_gravity, but we verify it here
  // against the configured latitude if needed. For now we trust
  // cfg_.local_gravity.
  eskf_scalar local_g = static_cast<eskf_scalar>(cfg_.local_gravity);

  // Initial state: identity orientation, zero position/velocity
  eskf::State initial;
  initial.setIdentity();
  initial.timestamp_us = 0;

  // Initial covariance and process noise can be replay-injected.
  eskf::InitialCovariance P0 = cfg_.toInitialCovariance();
  eskf::ProcessNoise Q = cfg_.toProcessNoise();

  // Initialize filter with TuningConfig from ReplayableConfig
  // This ensures identical behavior in replays
  eskf::TuningConfig tuning = cfg_.toTuningConfig();

  filter_.init(tuning);
  filter_.initialize(initial, P0, Q);

  // Configure yieldable timing
  eskf::YieldableConfig ycfg;
  ycfg.catchupBudgetUs = cfg_.catchup_budget_us;
  filter_.configure(ycfg);

  // Set gravity for shadow filters
  rail_shadow_.setLocalGravity(local_g);
  flight_shadow_.setLocalGravity(local_g);

  if (log_) {
    log_->printf("[ESKF] Local gravity: %.5f m/s² (lat=%.1f°)\n",
                 (double)local_g, cfg_.launch_latitude_deg);
  }
}

void EskfEstimator::setupVirtualImu() {
  eskf::VirtualImuConfig cfg;
  eskf::TuningConfig tuning = cfg_.toTuningConfig();

  const size_t requested_imu_count =
      std::max<size_t>(1, std::min<size_t>(active_imu_sources_, ESKF_MAX_IMUS));
  const size_t cfg_imu_count =
      std::max<size_t>(1, std::min<size_t>(static_cast<size_t>(cfg_.imu_count),
                                           ESKF_MAX_IMUS));
  cfg.imu_count = std::max<size_t>(requested_imu_count, cfg_imu_count);

  for (size_t i = 0; i < cfg.imu_count; ++i) {
    cfg.imus[i].enabled = true;
    for (int axis = 0; axis < 3; ++axis) {
      cfg.imus[i].position[axis] = 0;
      for (int col = 0; col < 3; ++col) {
        cfg.imus[i].to_body[axis][col] = (axis == col) ? 1.0 : 0.0;
      }
    }

    if (i < appcfg::kMaxCalibImus) {
      for (int axis = 0; axis < 3; ++axis) {
        cfg.imus[i].position[axis] =
            static_cast<eskf_scalar>(calib_cfg_.imu[i].position[axis]);
      }
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          cfg.imus[i].to_body[r][c] =
              static_cast<eskf_scalar>(calib_cfg_.imu[i].sensor_to_body[r][c]);
        }
      }
    }
  }

  // Voting requires ≥3 sensors for meaningful median; controlled by compile
  // flag
  cfg.voting_enabled = ESKF_ENABLE_SENSOR_VOTING && (cfg.imu_count > 2);

  cfg.gyro_voting_threshold = tuning.imu_gyro_voting_threshold;
  cfg.accel_voting_threshold = tuning.imu_accel_voting_threshold;
  cfg.gyro_hard_fault_threshold = tuning.imu_gyro_hard_fault_threshold;
  cfg.accel_hard_fault_threshold = tuning.imu_accel_hard_fault_threshold;
  cfg.hard_fault_suspect_samples = tuning.imu_hard_fault_suspect_samples;
  cfg.hard_fault_persistence_samples =
      tuning.imu_hard_fault_persistence_samples;
  cfg.recovery_cooldown_samples = tuning.imu_recovery_cooldown_samples;
  cfg.recovery_confirm_samples = tuning.imu_recovery_confirm_samples;
  cfg.accel_saturation_threshold = tuning.imu_accel_saturation_threshold;
  cfg.gyro_saturation_threshold = tuning.imu_gyro_saturation_threshold;
  cfg.saturation_hard_fault_persistence_samples =
      tuning.imu_saturation_hard_fault_persistence_samples;
  cfg.saturation_multi_axis_limit = tuning.imu_saturation_multi_axis_limit;
  cfg.enable_all_soft_reject_salvage =
      tuning.imu_enable_all_soft_reject_salvage;
  cfg.stale_persistence_samples = tuning.imu_stale_persistence_samples;
  cfg.stale_accel_delta_threshold = tuning.imu_stale_accel_delta_threshold;
  cfg.stale_gyro_delta_threshold = tuning.imu_stale_gyro_delta_threshold;
  cfg.enable_preflight_tare = tuning.imu_enable_preflight_tare;
  cfg.tare_window_samples = tuning.imu_tare_window_samples;
  cfg.tare_accel_gravity = tuning.imu_tare_accel_gravity;

  // Runtime tuning flag for ω̇ computation
  cfg.use_central_diff = tuning.use_central_diff;

  // Build calibration data from stored CalibrationConfig
  // Uses static storage since VirtualImu stores pointer, not copy
  static eskf::ImuCalibration imu_cals[ESKF_MAX_IMUS];
  for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
    auto &dst = imu_cals[i];
    dst = eskf::ImuCalibration{};
    dst.accel_transform[0][0] = 1.0;
    dst.accel_transform[1][1] = 1.0;
    dst.accel_transform[2][2] = 1.0;
    dst.gyro_scale[0] = 1.0;
    dst.gyro_scale[1] = 1.0;
    dst.gyro_scale[2] = 1.0;

    if (i >= cfg.imu_count || i >= appcfg::kMaxCalibImus) {
      continue;
    }

    const auto &src = calib_cfg_.imu[i];
    dst.ellipsoid_calibrated = src.ellipsoid_calibrated != 0;
    dst.thermal_calibrated = src.thermal_calibrated != 0;

    for (int j = 0; j < 3; ++j) {
      dst.accel_bias[j] = static_cast<eskf_scalar>(src.accel_bias[j]);
      dst.gyro_bias[j] = static_cast<eskf_scalar>(src.gyro_bias[j]);
      dst.gyro_scale[j] = static_cast<eskf_scalar>(src.gyro_scale[j]);
      for (int k = 0; k < 3; ++k) {
        dst.accel_transform[j][k] =
            static_cast<eskf_scalar>(src.accel_transform[j][k]);
        dst.gyro_g_sensitivity[j][k] =
            static_cast<eskf_scalar>(src.gyro_g_sensitivity[j][k]);
      }
      for (int c = 0; c < 4; ++c) {
        dst.accel_thermal[j].coeff[c] =
            static_cast<eskf_scalar>(src.accel_thermal[j][c]);
        dst.gyro_thermal[j].coeff[c] =
            static_cast<eskf_scalar>(src.gyro_thermal[j][c]);
      }
    }
    dst.reference_temp_k = static_cast<eskf_scalar>(src.reference_temp_k);
    dst.thermal_valid_min_temp_k =
      static_cast<eskf_scalar>(src.thermal_valid_min_temp_k);
    dst.thermal_valid_max_temp_k =
      static_cast<eskf_scalar>(src.thermal_valid_max_temp_k);
  }
  cfg.calibration = imu_cals;

  // Enable dynamic CG interpolation from full cg_table.
  cfg.cg_callback = &EskfEstimator::virtualImuCgCallback;
  cfg.cg_user_data = this;

  // Set static CG from calibration table (entry 0 = initial/static CG)
  if (calib_cfg_.cg_table_size > 0) {
    cfg.static_cg[0] =
        static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[0]);
    cfg.static_cg[1] =
        static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[1]);
    cfg.static_cg[2] =
        static_cast<eskf_scalar>(calib_cfg_.cg_table[0].position[2]);
  } else {
    cfg.static_cg[0] = 0;
    cfg.static_cg[1] = 0;
    cfg.static_cg[2] = 0;
  }

  virtual_imu_.configure(cfg);
  virtual_imu_.reset();
}

void EskfEstimator::setupVirtualBaro() {
  eskf::VirtualBaroConfig cfg;
  eskf::TuningConfig tuning = cfg_.toTuningConfig();

  const size_t requested_baro_count =
      std::min<size_t>(active_baro_sources_, ESKF_MAX_BAROS);
  cfg.baro_count = requested_baro_count > 0
      ? requested_baro_count
      : std::min<size_t>(static_cast<size_t>(ESKF_APP_BARO_COUNT),
                         ESKF_MAX_BAROS);
  for (size_t i = 0; i < cfg.baro_count; ++i) {
  cfg.baros[i].enabled = true;
  }
  cfg.voting_enabled = ESKF_ENABLE_SENSOR_VOTING && (cfg.baro_count > 2);

  cfg.voting_threshold_pa = tuning.baro_voting_threshold_pa;
  cfg.hard_fault_threshold_pa = tuning.baro_hard_fault_threshold_pa;
  cfg.calibration_mismatch_tolerance_pa =
    tuning.baro_calibration_mismatch_tolerance_pa;
  cfg.hard_fault_suspect_samples = tuning.baro_hard_fault_suspect_samples;
  cfg.hard_fault_persistence_samples =
    tuning.baro_hard_fault_persistence_samples;
  cfg.recovery_cooldown_samples = tuning.baro_recovery_cooldown_samples;
  cfg.recovery_confirm_samples = tuning.baro_recovery_confirm_samples;
  cfg.continuity_max_step_pa = tuning.baro_continuity_max_step_pa;
  cfg.enable_all_soft_reject_salvage =
    tuning.baro_enable_all_soft_reject_salvage;
  cfg.stale_persistence_samples = tuning.baro_stale_persistence_samples;
  cfg.stale_pressure_delta_threshold_pa =
      tuning.baro_stale_pressure_delta_threshold_pa;
  cfg.stale_temperature_delta_threshold_k =
      tuning.baro_stale_temperature_delta_threshold_k;

  // Build calibration data from stored CalibrationConfig
  // Uses static storage since VirtualBaro stores pointer, not copy
  static eskf::BaroCalibration baro_cals[ESKF_MAX_BAROS];
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
  baro_cals[i] = eskf::BaroCalibration{};
  baro_cals[i].pressure_scale = 1.0;
  }
  if (cfg.baro_count > 0) {
  baro_cals[0].pressure_bias_pa =
    static_cast<eskf_scalar>(calib_cfg_.baro.pressure_bias_pa);
  baro_cals[0].pressure_scale =
    static_cast<eskf_scalar>(calib_cfg_.baro.pressure_scale);
  baro_cals[0].temperature_bias_k =
    static_cast<eskf_scalar>(calib_cfg_.baro.temperature_bias_k);
  }
  cfg.calibration = baro_cals;

  // Build static pressure compensation from CalibrationConfig
  static eskf::StaticPressureCompensation static_p;
  static_p.cp_coefficient =
      static_cast<eskf_scalar>(calib_cfg_.static_pressure.cp_coefficient);
  static_p.air_density =
      static_cast<eskf_scalar>(calib_cfg_.static_pressure.air_density);
  cfg.static_pressure = static_p;

  virtual_baro_.configure(cfg);
  virtual_baro_.reset();
}

void EskfEstimator::setupVirtualCompass() {
  eskf::VirtualCompassConfig cfg;

  // Build magnetometer calibration from stored CalibrationConfig
  static eskf::MagCalibration mag_cal;
  for (int j = 0; j < 3; ++j) {
    mag_cal.hard_iron_bias[j] =
        static_cast<eskf_scalar>(calib_cfg_.mag.hard_iron_bias[j]);
    for (int k = 0; k < 3; ++k) {
      mag_cal.soft_iron_matrix[j][k] =
          static_cast<eskf_scalar>(calib_cfg_.mag.soft_iron_matrix[j][k]);
    }
  }
  mag_cal.variance_ut_sq =
      static_cast<eskf_scalar>(calib_cfg_.mag.variance_ut_sq);
  cfg.calibration = mag_cal;

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      cfg.sensor_to_body[r][c] =
          static_cast<eskf_scalar>(calib_cfg_.mag.sensor_to_body[r][c]);
    }
  }

  // Site-specific validation params from TuningConfig
  eskf::TuningConfig tuning = cfg_.toTuningConfig();
  cfg.expected_magnitude_ut = tuning.mag_expected_magnitude_ut;
  cfg.magnitude_threshold_ut = tuning.mag_magnitude_threshold_ut;
  // Note: Declination is applied in processMagSample via
  // calculateTiltCompensatedHeading()

  virtual_compass_.configure(cfg);
  virtual_compass_.reset();
}

void EskfEstimator::onLiftoff(uint32_t liftoff_ms) {
  if (in_flight_)
    return;

  descent_mode_active_ = false;
  descent_waiting_initial_gnss_snap_ = false;
  descent_last_gnss_fuse_us_ = 0;
  descent_filter_.reset();
  latest_descent_gnss_ = LatestGnssForDescent{};
  latest_descent_baro_ = LatestBaroForDescent{};

  in_flight_ = true;
  liftoff_ms_ = liftoff_ms;
  liftoff_us_ = static_cast<uint64_t>(liftoff_ms) * 1000ULL;
  last_valid_inflight_imu_us_ = liftoff_us_;
  sustained_imu_outage_handoff_todo_logged_ = false;

  // Calculate rewind timestamp (before liftoff detection)
  uint64_t rewind_to = liftoff_us_ > cfg_.liftoff_rewind_us
                           ? liftoff_us_ - cfg_.liftoff_rewind_us
                           : 0;

  // Prefer a preflight GNSS anchor at liftoff over first in-flight fix anchoring.
  // This avoids locking the origin to a potentially unconverged first in-flight fix.
  if (!gps_origin_set_ && latest_preflight_gps_anchor_.valid) {
    gps_origin_lat_ = latest_preflight_gps_anchor_.lat_rad;
    gps_origin_lon_ = latest_preflight_gps_anchor_.lon_rad;
    gps_origin_alt_ = latest_preflight_gps_anchor_.alt_m;
    gps_origin_set_ = true;
    filter_.core().setLateGpsOrigin(false);

    if (log_) {
      log_->printf("[ESKF] GPS origin anchored from preflight fix: lat=%.6f, "
                   "lon=%.6f, alt=%.1f (ts=%llu)\n",
                   gps_origin_lat_ / kDegToRad, gps_origin_lon_ / kDegToRad,
                   gps_origin_alt_,
                   static_cast<unsigned long long>(
                       latest_preflight_gps_anchor_.timestamp_us));
    }
  }

  // Find the most recent checkpoint before the rewind timestamp
  const eskf::RailShadowCheckpoint *cp =
      rail_shadow_.findCheckpointBefore(rewind_to);
  eskf::RailShadowCheckpoint fallback;

  if (!cp) {
    // No checkpoint available - use current state as fallback
    fallback = rail_shadow_.currentAsCheckpoint(liftoff_us_);
    cp = &fallback;
    if (log_) {
      log_->printf("[ESKF] Warning: No checkpoint before rewind_to=%llu, using "
                   "current state\n",
                   (unsigned long long)rewind_to);
    }
  }

  // Build combined quaternion from checkpoint's state (Issue #1 fix)
  // IMPORTANT: Use checkpoint's quaternion + heading, NOT current state.
  // The checkpoint was captured when the rocket was stationary at rewind_to
  // time. This ensures the ESKF is initialized with a consistent static state.
  eskf_scalar combined_q[4];
  eskf::RailShadowFilter::getCombinedQuaternionFromCheckpoint(*cp, combined_q);

  // Initialize Flight Shadow Filter with combined orientation
  flight_shadow_.reset(combined_q);

  // Get ground reference from oldest complete window
  bool used_ground_ref_fallback = false;
  eskf::GroundReference ground_ref = rail_shadow_.getOldestGroundReference();
  if (!ground_ref.valid) {
    eskf::GroundReference fallback_ref{};
    if (rail_shadow_.estimateGroundReferenceFromActiveWindow(fallback_ref)) {
      ground_ref = fallback_ref;
      used_ground_ref_fallback = true;
      if (log_) {
        log_->println(
            "[ESKF] Ground reference fallback: using active preflight baro window");
      }
      eskf::getEskfLogger().logEvent(eskf::EskfEventType::GroundRefFallbackUsed,
                                     liftoff_us_,
                                     1.0f);
    }
  }
  ground_reference_ = ground_ref;
  // Store ISA altitude of launch pad for FlightShadow AGL computation.
  // Uses ISA model (fixed T0) to avoid baro die-temperature bias.
  ground_isa_altitude_ = ground_ref.valid
                             ? static_cast<float>(eskf::pressureToAltitudeIsa(
                                   ground_ref.pressure_pa))
                             : 0.0f;

  // Set tare offsets on VirtualBaro for sensor-rejection-safe operation
  if (ground_ref.valid) {
    virtual_baro_.setTareOffsets(ground_ref.per_baro_offset_pa);
  }

  // Build LiftoffInitData from checkpoint
  eskf::LiftoffInitData init_data{};
  for (int i = 0; i < 4; ++i)
    init_data.q[i] = combined_q[i];
  for (int i = 0; i < 3; ++i)
    init_data.b_gyro[i] = cp->gyro_bias[i];
  for (int i = 0; i < 3; ++i) {
    init_data.b_acc[i] = turn_on_accel_bias_valid_ ? turn_on_accel_bias_[i] : 0;
  }
  init_data.heading_variance = cp->heading_variance;
  init_data.heading_initialized = cp->heading_initialized;
  init_data.liftoff_us = liftoff_us_;

  // Pass ground reference for b_baro initialization
  init_data.ground_altitude_m =
      ground_ref.valid ? eskf::pressureToAltitudeIsa(ground_ref.pressure_pa)
                       : 0;
  init_data.ground_reference_valid = ground_ref.valid;

  // Persist a full handoff snapshot for app-layer SD logging.
  last_liftoff_snapshot_ = LiftoffSnapshot{};
  for (int i = 0; i < 4; ++i) {
    last_liftoff_snapshot_.q[i] = static_cast<float>(combined_q[i]);
  }
  for (int i = 0; i < 3; ++i) {
    last_liftoff_snapshot_.b_gyro[i] = static_cast<float>(init_data.b_gyro[i]);
    last_liftoff_snapshot_.b_acc[i] = static_cast<float>(init_data.b_acc[i]);
  }
  last_liftoff_snapshot_.heading_variance =
      static_cast<float>(init_data.heading_variance);
  last_liftoff_snapshot_.heading_initialized = init_data.heading_initialized;
  last_liftoff_snapshot_.ground_reference_valid =
      init_data.ground_reference_valid;
  last_liftoff_snapshot_.used_ground_ref_fallback = used_ground_ref_fallback;
  last_liftoff_snapshot_.ground_pressure_pa =
      static_cast<float>(ground_ref.pressure_pa);
  last_liftoff_snapshot_.ground_temperature_k =
      static_cast<float>(ground_ref.temperature_k);
  last_liftoff_snapshot_.ground_isa_altitude_m = ground_isa_altitude_;
  last_liftoff_snapshot_.ground_altitude_m =
      static_cast<float>(init_data.ground_altitude_m);
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    last_liftoff_snapshot_.baro_offsets_pa[i] =
        static_cast<float>(ground_ref.per_baro_offset_pa[i]);
  }
  last_liftoff_snapshot_.timestamp_us = liftoff_us_;
  has_liftoff_snapshot_ = true;

  // Inject into ESKF with full state initialization
  filter_.injectLiftoffSnap(init_data, rewind_to);
  // Enter flight mode so in-flight update policies (e.g. bias handling)
  // are actually activated after liftoff.
  filter_.core().setMode(eskf::FilterMode::Flight);

  output_dirty_ = true;

  if (log_) {
    log_->printf("[ESKF] Liftoff at T=%ums, q=[%.3f, %.3f, %.3f, %.3f], "
                 "gyro_bias=[%.5f, %.5f, %.5f], accel_bias=[%.5f, %.5f, %.5f], "
                 "heading_init=%d, ground_ref=%d "
                 "(alt=%.1fm)\\n",
                 liftoff_ms, (double)combined_q[0], (double)combined_q[1],
                 (double)combined_q[2], (double)combined_q[3],
                 (double)init_data.b_gyro[0], (double)init_data.b_gyro[1],
                 (double)init_data.b_gyro[2], (double)init_data.b_acc[0],
                 (double)init_data.b_acc[1], (double)init_data.b_acc[2],
                 init_data.heading_initialized ? 1 : 0,
                 init_data.ground_reference_valid ? 1 : 0,
                 (double)init_data.ground_altitude_m);
  }
}

void EskfEstimator::onFlightStateChange(flight_computer::State state,
                                        uint8_t reason) {
  (void)reason;

  if (state == flight_computer::State::DESCENT && in_flight_) {
    const uint64_t ts = filter_.state().timestamp_us > 0
                            ? filter_.state().timestamp_us
                            : static_cast<uint64_t>(liftoff_ms_) * 1000ULL;
    enterDescentMode(ts);
  } else if (state == flight_computer::State::LANDED ||
             state == flight_computer::State::INIT ||
             state == flight_computer::State::CALIBRATION ||
             state == flight_computer::State::FILLING ||
             state == flight_computer::State::ARMED ||
             state == flight_computer::State::PRESSURIZATION ||
             state == flight_computer::State::IGNITION ||
             state == flight_computer::State::ABORT_ON_GROUND ||
             state == flight_computer::State::ABORT_IN_FLIGHT) {
    descent_mode_active_ = false;
    descent_waiting_initial_gnss_snap_ = false;
    descent_last_gnss_fuse_us_ = 0;
    descent_filter_.reset();
  }

  output_dirty_ = true;
}

void EskfEstimator::enterDescentMode(uint64_t timestamp_us) {
  if (descent_mode_active_ && descent_filter_.isActive()) {
    return;
  }

  DescentNavFilter::InitData init{};
  init.timestamp_us = timestamp_us;

  const eskf::State &s = filter_.state();
  for (int i = 0; i < 3; ++i) {
    init.position_ned[i] = static_cast<float>(s.p[i]);
    init.velocity_ned[i] = static_cast<float>(s.v[i]);
    init.position_var_ned[i] = 400.0f;
    init.velocity_var_ned[i] = 100.0f;
  }
  init.baro_bias_m = 0.0f;
  init.baro_bias_var = 25.0f;

  const bool gnss_fresh = latest_descent_gnss_.valid &&
                          latest_descent_gnss_.timestamp_us > 0 &&
                          (timestamp_us <= latest_descent_gnss_.timestamp_us +
                                              3000000ULL);
  if (gnss_fresh) {
    init.timestamp_us = std::max(init.timestamp_us, latest_descent_gnss_.timestamp_us);
    for (int i = 0; i < 3; ++i) {
      const float min_pos_var = (i < 2) ? 25.0f : 4.0f;
      const float min_vel_var = (i < 2) ? 64.0f : 9.0f;
      init.position_ned[i] = latest_descent_gnss_.pos_ned[i];
      init.velocity_ned[i] = latest_descent_gnss_.vel_ned[i];
      init.position_var_ned[i] =
          std::max(min_pos_var, latest_descent_gnss_.var_pos_ned[i]);
      init.velocity_var_ned[i] =
          std::max(min_vel_var, latest_descent_gnss_.var_vel_ned[i]);
    }
  }

  if (latest_descent_baro_.valid) {
    init.timestamp_us = std::max(init.timestamp_us, latest_descent_baro_.timestamp_us);
    init.baro_bias_m = latest_descent_baro_.down_position_m - init.position_ned[2];
    init.baro_bias_var = std::max(4.0f, latest_descent_baro_.variance);
  }

  descent_filter_.enter(init);
  descent_filter_.setBaroHoldUntilUs(init.timestamp_us + 700000ULL);
  descent_mode_active_ = true;
  descent_waiting_initial_gnss_snap_ = !gnss_fresh;
  descent_last_gnss_fuse_us_ = gnss_fresh ? init.timestamp_us : 0;

  if (log_) {
    log_->printf(
        "[ESKF] DescentNav active: p=[%.1f, %.1f, %.1f] v=[%.1f, %.1f, %.1f] b_baro=%.2f waiting_gnss_snap=%d\n",
        init.position_ned[0], init.position_ned[1], init.position_ned[2],
        init.velocity_ned[0], init.velocity_ned[1], init.velocity_ned[2],
        init.baro_bias_m, descent_waiting_initial_gnss_snap_ ? 1 : 0);
  }
}

void EskfEstimator::processImuBatch(const ImuBatch &batch) {
  if (!initialized_ || !batch.data || batch.count == 0)
    return;

  // Buffer the batch by source index.
  const size_t src = batch.source < kMaxImuSources ? batch.source : 0;

  // If we already have a pending batch for this source, flush it first to
  // avoid overwriting.
  if (pending_imu_[src].valid) {
    processBufferedImuBatch(pending_imu_[src]);
    pending_imu_[src].valid = false;
  }

  PendingImuBatch &pending = pending_imu_[src];

  // Deep copy samples (original may be released after this call)
  pending.samples.resize(batch.count);
  for (size_t i = 0; i < batch.count; ++i) {
    pending.samples[i] = batch.data[i];
  }
  pending.t0_us = batch.t0_us;
  pending.dt_us = batch.dt_us > 0 ? batch.dt_us : 1000;
  pending.source = static_cast<uint8_t>(src);
  pending.valid = true;

  // Check for stale batches from other sources (flush before pairing)
  flushPendingImuIfStale(batch.t0_us);

  const size_t target_group_size =
      std::max<size_t>(1, std::min<size_t>(active_imu_sources_,
                                           kMaxImuSources));
  if (target_group_size == 1) {
    processBufferedImuBatch(pending_imu_[src]);
    pending_imu_[src].valid = false;
    return;
  }

  size_t valid_count = 0;
  uint64_t anchor_t0 = std::numeric_limits<uint64_t>::max();
  for (size_t i = 0; i < kMaxImuSources; ++i) {
    if (!pending_imu_[i].valid) {
      continue;
    }
    ++valid_count;
    if (pending_imu_[i].t0_us < anchor_t0) {
      anchor_t0 = pending_imu_[i].t0_us;
    }
  }
  if (valid_count < target_group_size) {
    return;
  }

  const PendingImuBatch *group[kMaxImuSources] = {};
  size_t group_count = 0;
  for (size_t i = 0; i < kMaxImuSources; ++i) {
    if (!pending_imu_[i].valid) {
      continue;
    }
    const int64_t dt = static_cast<int64_t>(pending_imu_[i].t0_us) -
                       static_cast<int64_t>(anchor_t0);
    if (std::llabs(dt) <= static_cast<int64_t>(kImuSyncToleranceUs)) {
      group[group_count++] = &pending_imu_[i];
      if (group_count >= target_group_size) {
        break;
      }
    }
  }

  if (group_count >= target_group_size) {
    processSyncedImuGroup(group, target_group_size);
    for (size_t gi = 0; gi < target_group_size; ++gi) {
      const size_t source = group[gi]->source;
      if (source < kMaxImuSources) {
        pending_imu_[source].valid = false;
      }
    }
  }
}

void EskfEstimator::flushPendingImuIfStale(uint64_t current_t0_us) {
  for (size_t i = 0; i < kMaxImuSources; ++i) {
    if (pending_imu_[i].valid) {
      int64_t age = static_cast<int64_t>(current_t0_us) -
                    static_cast<int64_t>(pending_imu_[i].t0_us);
      if (age > static_cast<int64_t>(kImuBatchTimeoutUs)) {
        processBufferedImuBatch(pending_imu_[i]);
        pending_imu_[i].valid = false;
      }
    }
  }
}

void EskfEstimator::processSyncedImuBatches(const PendingImuBatch &batch0,
                                            const PendingImuBatch &batch1) {
  const PendingImuBatch *group[2] = {&batch0, &batch1};
  processSyncedImuGroup(group, 2);
}

void EskfEstimator::updateInFlightImuOutageState(
    const eskf::VirtualImuOutput &vout) {
  if (!in_flight_) {
    return;
  }

  const uint64_t ts = vout.frame.timestamp_us;
  if (vout.valid_imu_count > 0) {
    last_valid_inflight_imu_us_ = ts;
    sustained_imu_outage_handoff_todo_logged_ = false;
    return;
  }

  if (last_valid_inflight_imu_us_ == 0 || ts <= last_valid_inflight_imu_us_) {
    return;
  }

  const uint64_t outage_us = ts - last_valid_inflight_imu_us_;
  if (outage_us < kSustainedImuOutageHandoffUs ||
      sustained_imu_outage_handoff_todo_logged_) {
    return;
  }

  sustained_imu_outage_handoff_todo_logged_ = true;

  // TODO(eskf-fallback): when the descent/non-IMU estimator path is available,
  // trigger automatic handoff here for sustained in-flight all-IMU outage.
  if (log_) {
    log_->printf(
        "[ESKF] Sustained in-flight all-IMU outage (%llums): TODO handoff "
        "to non-IMU descent path (baro+GNSS).\n",
        static_cast<unsigned long long>(outage_us / 1000ULL));
  }
}

void EskfEstimator::processSyncedImuGroup(const PendingImuBatch *const *group,
                                          size_t group_count) {
  if (!group || group_count == 0)
    return;

  size_t count = std::numeric_limits<size_t>::max();
  uint64_t t0_us = std::numeric_limits<uint64_t>::max();
  uint32_t dt_us = std::numeric_limits<uint32_t>::max();
  for (size_t i = 0; i < group_count; ++i) {
    const PendingImuBatch *b = group[i];
    if (!b)
      continue;
    count = std::min(count, b->samples.size());
    t0_us = std::min(t0_us, b->t0_us);
    dt_us = std::min(dt_us, b->dt_us);
  }
  if (count == std::numeric_limits<size_t>::max()) {
    return;
  }

  if (count == 0)
    return;

  if (dt_us == std::numeric_limits<uint32_t>::max()) {
    dt_us = 1000;
  }

  const eskf_scalar dt_s = static_cast<eskf_scalar>(dt_us) / 1e6;

  static constexpr size_t kMaxBatchSize = 64;
  const size_t safe_count = std::min(count, kMaxBatchSize);

  // ICM-45686 FIFO temperature conversion: T_C = temp_data/2.07 + 25
  constexpr eskf_scalar kTempScale = 1.0 / 2.07;
  constexpr eskf_scalar kTempOffsetK = 25.0 + 273.15; // 25°C in Kelvin

  eskf_sensor_t accel_data[ESKF_MAX_IMUS][kMaxBatchSize * 3] = {};
  eskf_sensor_t gyro_data[ESKF_MAX_IMUS][kMaxBatchSize * 3] = {};
  eskf_scalar temp_data[ESKF_MAX_IMUS][kMaxBatchSize] = {};
  bool source_present[ESKF_MAX_IMUS] = {};

#if APP_IMU_LOG_FORMAT == 0
  const eskf_scalar kAccelScale =
      (static_cast<float>(cfg_.imu_accel_fsr_g) * 9.80665f) / 32768.0f;
  const eskf_scalar kGyroScale =
      (static_cast<float>(cfg_.imu_gyro_fsr_dps) * 3.14159265359f / 180.0f) /
      32768.0f;

  for (size_t gi = 0; gi < group_count; ++gi) {
    const PendingImuBatch *b = group[gi];
    if (!b || b->source >= ESKF_MAX_IMUS) {
      continue;
    }
    const size_t src = b->source;
    source_present[src] = true;
    for (size_t i = 0; i < safe_count; ++i) {
      const ImuSample &s = b->samples[i];
      accel_data[src][i * 3 + 0] = static_cast<eskf_scalar>(s.ax) * kAccelScale;
      accel_data[src][i * 3 + 1] = static_cast<eskf_scalar>(s.ay) * kAccelScale;
      accel_data[src][i * 3 + 2] = static_cast<eskf_scalar>(s.az) * kAccelScale;
      gyro_data[src][i * 3 + 0] = static_cast<eskf_scalar>(s.gx) * kGyroScale;
      gyro_data[src][i * 3 + 1] = static_cast<eskf_scalar>(s.gy) * kGyroScale;
      gyro_data[src][i * 3 + 2] = static_cast<eskf_scalar>(s.gz) * kGyroScale;
      temp_data[src][i] =
          kTempOffsetK + static_cast<eskf_scalar>(s.temperature) * kTempScale;
    }
  }
#else
  for (size_t gi = 0; gi < group_count; ++gi) {
    const PendingImuBatch *b = group[gi];
    if (!b || b->source >= ESKF_MAX_IMUS) {
      continue;
    }
    const size_t src = b->source;
    source_present[src] = true;
    for (size_t i = 0; i < safe_count; ++i) {
      const ImuSample &s = b->samples[i];
      accel_data[src][i * 3 + 0] = s.ax;
      accel_data[src][i * 3 + 1] = s.ay;
      accel_data[src][i * 3 + 2] = s.az;
      gyro_data[src][i * 3 + 0] = s.gx;
      gyro_data[src][i * 3 + 1] = s.gy;
      gyro_data[src][i * 3 + 2] = s.gz;
      temp_data[src][i] =
          kTempOffsetK + static_cast<eskf_scalar>(s.temperature) * kTempScale;
    }
  }
#endif

  const eskf_sensor_t *accel_ptrs[ESKF_MAX_IMUS] = {};
  const eskf_sensor_t *gyro_ptrs[ESKF_MAX_IMUS] = {};
  const eskf_scalar *temp_ptrs[ESKF_MAX_IMUS] = {};
  eskf::SensorStatus statuses[ESKF_MAX_IMUS];
  for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
    if (source_present[i]) {
      accel_ptrs[i] = accel_data[i];
      gyro_ptrs[i] = gyro_data[i];
      temp_ptrs[i] = temp_data[i];
      statuses[i] = eskf::SensorStatus::OK;
    } else {
      statuses[i] = eskf::SensorStatus::HARD_FAIL;
    }
  }

  // Process batch
  eskf::VirtualImuRuntimePolicy imu_policy;
  imu_policy.soft_threshold_scale = 1.0;
  imu_policy.boost_phase = false;
  imu_policy.in_flight = in_flight_;
  if (in_flight_ && liftoff_us_ > 0 && t0_us >= liftoff_us_) {
    const uint64_t since_liftoff_us = t0_us - liftoff_us_;
    if (since_liftoff_us <= cfg_.imu_boost_soft_window_us) {
      imu_policy.soft_threshold_scale = cfg_.imu_boost_soft_threshold_scale;
      imu_policy.boost_phase = true;
    }
  }
  virtual_imu_.setRuntimePolicy(imu_policy);

  const eskf_scalar *gyro_bias_body =
      in_flight_ ? filter_.state().b_gyro : rail_shadow_.gyroBias();
  eskf::VirtualImuOutput vout_buffer[kMaxBatchSize];
  size_t out_count = virtual_imu_.process(accel_ptrs, gyro_ptrs, temp_ptrs,
                                          statuses, safe_count, t0_us,
                                          vout_buffer, kMaxBatchSize, dt_us,
                                          gyro_bias_body);

  // Push processed frames to filter and shadow filters
  for (size_t i = 0; i < out_count; ++i) {
    const eskf::VirtualImuOutput &vout = vout_buffer[i];
    updateInFlightImuOutageState(vout);

    if (vout.valid_imu_count > 0) {
      if (!in_flight_) {
        updateTurnOnAccelBiasEstimate(vout.nav_accel, vout.nav_gyro,
                                      vout.frame.timestamp_us);
      }

      // One-shot initialization from first valid, calibrated sample
      if (!rail_shadow_initialized_) {
        // Initialize from pre-lever-arm accel to avoid gyro-bias artifacts.
        rail_shadow_.initializeFromAccel(vout.nav_accel);
        rail_shadow_initialized_ = true;
      }

      filter_.pushImu(vout.frame, dt_s);
      last_body_accel_x_ = vout.frame.accel[0];

      if (!in_flight_) {
        // Use pre-lever-arm accel for rail shadow to avoid bias-induced drift.
        rail_shadow_.update(vout.nav_accel, vout.nav_gyro, dt_s,
                            vout.frame.timestamp_us);
        if (vout.frame.timestamp_us - last_rail_checkpoint_us_ >=
            ESKF_RAIL_CHECKPOINT_INTERVAL_US) {
          rail_shadow_.saveCheckpoint(vout.frame.timestamp_us);
          last_rail_checkpoint_us_ = vout.frame.timestamp_us;
        }
      } else {
        flight_shadow_.predict(vout.frame.accel, vout.frame.gyro, dt_s,
                               vout.frame.timestamp_us);
        last_flight_shadow_predict_us_ = vout.frame.timestamp_us;

        // Zero sideslip constraint: fires decimated from IMU rate.
        // Uses body-frame lateral velocity ≈ 0 to correct yaw.
#if ESKF_ENABLE_SIDESLIP
  // Sideslip needs a trustworthy yaw reference; defer until one-shot
  // heading alignment has completed.
  if (cfg_.enable_sideslip && filter_.core().isHeadingAligned()) {
          if (++sideslip_decimation_counter_ >= cfg_.sideslip_decimation) {
            sideslip_decimation_counter_ = 0;
            const auto &st = filter_.state();
            const eskf_scalar spd_sq =
                st.v[0] * st.v[0] + st.v[1] * st.v[1] + st.v[2] * st.v[2];
            const bool ascent_or_coast = (st.v[2] <= static_cast<eskf_scalar>(0));
            const eskf_scalar gyro_sq =
                vout.frame.gyro[0] * vout.frame.gyro[0] +
                vout.frame.gyro[1] * vout.frame.gyro[1] +
                vout.frame.gyro[2] * vout.frame.gyro[2];
            const eskf_scalar min_spd =
                static_cast<eskf_scalar>(cfg_.sideslip_min_speed);
            const eskf_scalar max_gyro =
                static_cast<eskf_scalar>(cfg_.sideslip_max_gyro);
            if (ascent_or_coast &&
                spd_sq >= min_spd * min_spd &&
                gyro_sq < max_gyro * max_gyro) {
              filter_.pushSideslip(
                  static_cast<eskf_scalar>(cfg_.sideslip_r_lateral),
                  vout.frame.timestamp_us);
            }
          }
        } else if (!filter_.core().isHeadingAligned()) {
          // Keep schedule deterministic: decimation starts only after alignment.
          sideslip_decimation_counter_ = 0;
        }
#endif
      }

      // Keep IMU pipeline visibility in all phases for post-flight
      // attribution of high-dynamics spikes.
      logImuPipelineIfDue(vout, dt_s);
      logImuHealthTransitions(vout);
    } else {
      // Keep pipeline visibility when all IMUs are currently invalid.
      logImuPipelineIfDue(vout, dt_s);
      logImuHealthTransitions(vout);
    }
  }

  output_dirty_ = true;
}

void EskfEstimator::processBufferedImuBatch(const PendingImuBatch &batch) {
  // Process a single-source batch (fallback when pairing fails)
  const size_t count = batch.samples.size();
  if (count == 0)
    return;

  const uint32_t dt_us = batch.dt_us;
  const eskf_scalar dt_s = static_cast<eskf_scalar>(dt_us) / 1e6;

  static constexpr size_t kMaxBatchSize = 64;
  const size_t safe_count = std::min(count, kMaxBatchSize);

  constexpr eskf_scalar kTempScale = 1.0 / 2.07;
  constexpr eskf_scalar kTempOffsetK = 25.0 + 273.15;

  eskf_sensor_t accel_data[kMaxBatchSize * 3];
  eskf_sensor_t gyro_data[kMaxBatchSize * 3];
  eskf_scalar temp_data[kMaxBatchSize];

#if APP_IMU_LOG_FORMAT == 0
  const eskf_scalar kAccelScale =
      (static_cast<float>(cfg_.imu_accel_fsr_g) * 9.80665f) / 32768.0f;
  const eskf_scalar kGyroScale =
      (static_cast<float>(cfg_.imu_gyro_fsr_dps) * 3.14159265359f / 180.0f) /
      32768.0f;

  for (size_t i = 0; i < safe_count; ++i) {
    const ImuSample &s = batch.samples[i];
    accel_data[i * 3 + 0] = static_cast<eskf_scalar>(s.ax) * kAccelScale;
    accel_data[i * 3 + 1] = static_cast<eskf_scalar>(s.ay) * kAccelScale;
    accel_data[i * 3 + 2] = static_cast<eskf_scalar>(s.az) * kAccelScale;
    gyro_data[i * 3 + 0] = static_cast<eskf_scalar>(s.gx) * kGyroScale;
    gyro_data[i * 3 + 1] = static_cast<eskf_scalar>(s.gy) * kGyroScale;
    gyro_data[i * 3 + 2] = static_cast<eskf_scalar>(s.gz) * kGyroScale;
    temp_data[i] =
        kTempOffsetK + static_cast<eskf_scalar>(s.temperature) * kTempScale;
  }
#else
  for (size_t i = 0; i < safe_count; ++i) {
    const ImuSample &s = batch.samples[i];
    accel_data[i * 3 + 0] = s.ax;
    accel_data[i * 3 + 1] = s.ay;
    accel_data[i * 3 + 2] = s.az;
    gyro_data[i * 3 + 0] = s.gx;
    gyro_data[i * 3 + 1] = s.gy;
    gyro_data[i * 3 + 2] = s.gz;
    temp_data[i] =
        kTempOffsetK + static_cast<eskf_scalar>(s.temperature) * kTempScale;
  }
#endif

  // Setup data pointers - only the source IMU is valid
  const eskf_sensor_t *accel_ptrs[ESKF_MAX_IMUS] = {nullptr, nullptr, nullptr,
                                                    nullptr};
  const eskf_sensor_t *gyro_ptrs[ESKF_MAX_IMUS] = {nullptr, nullptr, nullptr,
                                                   nullptr};
  const eskf_scalar *temp_ptrs[ESKF_MAX_IMUS] = {nullptr, nullptr, nullptr,
                                                 nullptr};

  size_t imu_idx = batch.source < ESKF_MAX_IMUS ? batch.source : 0;
  accel_ptrs[imu_idx] = accel_data;
  gyro_ptrs[imu_idx] = gyro_data;
  temp_ptrs[imu_idx] = temp_data;

  eskf::SensorStatus statuses[ESKF_MAX_IMUS];
  for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
    statuses[i] =
        (i == imu_idx) ? eskf::SensorStatus::OK : eskf::SensorStatus::HARD_FAIL;
  }

  eskf::VirtualImuRuntimePolicy imu_policy;
  imu_policy.soft_threshold_scale = 1.0;
  imu_policy.boost_phase = false;
  imu_policy.in_flight = in_flight_;
  if (in_flight_ && liftoff_us_ > 0 && batch.t0_us >= liftoff_us_) {
    const uint64_t since_liftoff_us = batch.t0_us - liftoff_us_;
    if (since_liftoff_us <= cfg_.imu_boost_soft_window_us) {
      imu_policy.soft_threshold_scale = cfg_.imu_boost_soft_threshold_scale;
      imu_policy.boost_phase = true;
    }
  }
  virtual_imu_.setRuntimePolicy(imu_policy);

  const eskf_scalar *gyro_bias_body =
      in_flight_ ? filter_.state().b_gyro : rail_shadow_.gyroBias();
  eskf::VirtualImuOutput vout_buffer[kMaxBatchSize];
  size_t out_count = virtual_imu_.process(accel_ptrs, gyro_ptrs, temp_ptrs,
                                          statuses, safe_count, batch.t0_us,
                                          vout_buffer, kMaxBatchSize, dt_us,
                                          gyro_bias_body);

  for (size_t i = 0; i < out_count; ++i) {
    const eskf::VirtualImuOutput &vout = vout_buffer[i];
    updateInFlightImuOutageState(vout);

    if (vout.valid_imu_count > 0) {
      if (!in_flight_) {
        updateTurnOnAccelBiasEstimate(vout.nav_accel, vout.nav_gyro,
                                      vout.frame.timestamp_us);
      }

      // One-shot initialization from first valid, calibrated sample
      if (!rail_shadow_initialized_) {
        // Initialize from pre-lever-arm accel to avoid gyro-bias artifacts.
        rail_shadow_.initializeFromAccel(vout.nav_accel);
        rail_shadow_initialized_ = true;
      }

      filter_.pushImu(vout.frame, dt_s);
      last_body_accel_x_ = vout.frame.accel[0];

      if (!in_flight_) {
        // Use pre-lever-arm accel for rail shadow to avoid bias-induced drift.
        rail_shadow_.update(vout.nav_accel, vout.nav_gyro, dt_s,
                            vout.frame.timestamp_us);
        if (vout.frame.timestamp_us - last_rail_checkpoint_us_ >=
            cfg_.rail_checkpoint_interval_us) {
          rail_shadow_.saveCheckpoint(vout.frame.timestamp_us);
          last_rail_checkpoint_us_ = vout.frame.timestamp_us;
        }
      } else {
        flight_shadow_.predict(vout.frame.accel, vout.frame.gyro, dt_s,
                               vout.frame.timestamp_us);
        last_flight_shadow_predict_us_ = vout.frame.timestamp_us;

        // Zero sideslip constraint: fires decimated from IMU rate.
        // Uses body-frame lateral velocity ≈ 0 to correct yaw.
#if ESKF_ENABLE_SIDESLIP
  // Sideslip needs a trustworthy yaw reference; defer until one-shot
  // heading alignment has completed.
  if (cfg_.enable_sideslip && filter_.core().isHeadingAligned()) {
          if (++sideslip_decimation_counter_ >= cfg_.sideslip_decimation) {
            sideslip_decimation_counter_ = 0;
            const auto &st = filter_.state();
            const eskf_scalar spd_sq =
                st.v[0] * st.v[0] + st.v[1] * st.v[1] + st.v[2] * st.v[2];
            const bool ascent_or_coast = (st.v[2] <= static_cast<eskf_scalar>(0));
            const eskf_scalar gyro_sq =
                vout.frame.gyro[0] * vout.frame.gyro[0] +
                vout.frame.gyro[1] * vout.frame.gyro[1] +
                vout.frame.gyro[2] * vout.frame.gyro[2];
            const eskf_scalar min_spd =
                static_cast<eskf_scalar>(cfg_.sideslip_min_speed);
            const eskf_scalar max_gyro =
                static_cast<eskf_scalar>(cfg_.sideslip_max_gyro);
            if (ascent_or_coast &&
                spd_sq >= min_spd * min_spd &&
                gyro_sq < max_gyro * max_gyro) {
              filter_.pushSideslip(
                  static_cast<eskf_scalar>(cfg_.sideslip_r_lateral),
                  vout.frame.timestamp_us);
            }
          }
        } else if (!filter_.core().isHeadingAligned()) {
          // Keep schedule deterministic: decimation starts only after alignment.
          sideslip_decimation_counter_ = 0;
        }
#endif
      }

      // Keep IMU pipeline visibility in all phases for post-flight
      // attribution of high-dynamics spikes.
      logImuPipelineIfDue(vout, dt_s);
      logImuHealthTransitions(vout);
    } else {
      // Keep pipeline visibility when all IMUs are currently invalid.
      logImuPipelineIfDue(vout, dt_s);
      logImuHealthTransitions(vout);
    }
  }

  output_dirty_ = true;
}

void EskfEstimator::flushPendingBaroIfStale(uint64_t current_ts_us) {
  for (size_t i = 0; i < kMaxBaroSources; ++i) {
    if (!pending_baro_[i].valid) {
      continue;
    }
    const int64_t age = static_cast<int64_t>(current_ts_us) -
                        static_cast<int64_t>(pending_baro_[i].timestamp_us);
    if (age > static_cast<int64_t>(kBaroBatchTimeoutUs)) {
      pending_baro_[i].valid = false;
    }
  }
}

void EskfEstimator::processBaroObservation(uint8_t source,
                                           eskf_sensor_t pressure_pa,
                                           eskf_sensor_t temperature_k,
                                           uint64_t timestamp_us,
                                           float dt_s,
                                           bool trigger_before_complete) {
  const size_t baro_count_cfg = std::max<size_t>(
      size_t(1), std::min<size_t>(active_baro_sources_, ESKF_MAX_BAROS));

  auto fuse_output = [&](const eskf::BaroOutput &bout, uint64_t ts) {
    logBaroHealthTransitions(bout);

    if (!in_flight_ && bout.valid_count > 0) {
      // Pre-flight ground-reference accumulation consumes the fused virtual
      // barometer output (single scalar pressure/temperature sample).
      // Pass baro_count=1 to avoid treating adjacent memory as extra sensors
      // when multi-baro replay is enabled.
      bool fused_valid_flags[ESKF_MAX_BAROS] = {};
      fused_valid_flags[0] = true;
      rail_shadow_.updateBaro(&bout.pressure_pa, &bout.temperature_k,
                              fused_valid_flags, 1, ts);
    }

    if (bout.valid_count > 0) {
      const float altitude_isa_m =
          static_cast<float>(eskf::pressureToAltitudeIsa(bout.pressure_pa));
      const float altitude_agl_m = altitude_isa_m - ground_isa_altitude_;

      if (gps_origin_set_) {
        latest_descent_baro_.valid = true;
        latest_descent_baro_.timestamp_us = ts;
        latest_descent_baro_.down_position_m = gps_origin_alt_ - altitude_isa_m;
        latest_descent_baro_.variance = std::max(0.25f, static_cast<float>(bout.variance));
        if (descent_mode_active_) {
          descent_filter_.fuseBaroDown(ts,
                                       latest_descent_baro_.down_position_m,
                                       latest_descent_baro_.variance);
        }
      }

      if (in_flight_) {
        if (flight_shadow_.isAeroBlind() && last_flight_shadow_predict_us_ > 0 &&
            ts > last_flight_shadow_predict_us_ &&
            (ts - last_flight_shadow_predict_us_) > kFlightShadowImuOutageFallbackUs) {
          flight_shadow_.forceExitAeroBlindForImuOutage();
          was_aero_blind_ = false;
          baro_reacquire_needed_ = true;
        }

        if (baro_reacquire_needed_) {
          performBaroReacquisition(static_cast<eskf_scalar>(altitude_isa_m), ts);
          flight_shadow_.correctBaro(altitude_agl_m, dt_s);
          return;
        }
        if (shouldStartEskfBaroFusion(ts)) {
          if (trigger_before_complete) {
            filter_.triggerBaro(ts);
          }
          filter_.completeBaro(static_cast<eskf_scalar>(altitude_isa_m));
        }
        flight_shadow_.correctBaro(altitude_agl_m, dt_s);
      }
    }
  };

  if (baro_count_cfg <= 1) {
    eskf::SensorStatus status = eskf::SensorStatus::OK;
    eskf::BaroOutput bout =
        virtual_baro_.process(&pressure_pa, &temperature_k, &status, timestamp_us);
    fuse_output(bout, timestamp_us);
    output_dirty_ = true;
    return;
  }

  const size_t src = std::min<size_t>(source, kMaxBaroSources - 1);
  pending_baro_[src].pressure_pa = pressure_pa;
  pending_baro_[src].temperature_k = temperature_k;
  pending_baro_[src].timestamp_us = timestamp_us;
  pending_baro_[src].valid = true;

  flushPendingBaroIfStale(timestamp_us);

  bool ready = true;
  uint64_t min_ts = std::numeric_limits<uint64_t>::max();
  uint64_t max_ts = 0;
  for (size_t i = 0; i < baro_count_cfg; ++i) {
    if (!pending_baro_[i].valid) {
      ready = false;
      break;
    }
    min_ts = std::min(min_ts, pending_baro_[i].timestamp_us);
    max_ts = std::max(max_ts, pending_baro_[i].timestamp_us);
  }
  if (!ready) {
    return;
  }
  if (max_ts - min_ts > kBaroSyncToleranceUs) {
    return;
  }

  eskf_sensor_t pressure[ESKF_MAX_BAROS] = {};
  eskf_sensor_t temp[ESKF_MAX_BAROS] = {};
  eskf::SensorStatus status[ESKF_MAX_BAROS];
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    status[i] = eskf::SensorStatus::HARD_FAIL;
  }
  for (size_t i = 0; i < baro_count_cfg; ++i) {
    pressure[i] = pending_baro_[i].pressure_pa;
    temp[i] = pending_baro_[i].temperature_k;
    status[i] = eskf::SensorStatus::OK;
    pending_baro_[i].valid = false;
  }

  eskf::BaroOutput bout = virtual_baro_.process(pressure, temp, status, max_ts);
  fuse_output(bout, max_ts);
  output_dirty_ = true;
}

void EskfEstimator::processBaroBatch(const BaroBatch &batch) {
#if !ESKF_USE_BARO
  (void)batch;
  return;
#endif
  if (!initialized_ || !batch.data || batch.count == 0)
    return;

  const uint32_t dt_us = batch.dt_us > 0 ? batch.dt_us : 20000;
  const float dt_s = static_cast<float>(dt_us) / 1e6f;

  for (size_t i = 0; i < batch.count; ++i) {
    const BaroSample &sample = batch.data[i];
    const uint64_t sample_ts = batch.t0_us + static_cast<uint64_t>(i) * dt_us;
#if APP_TARGET_NATIVE
    const uint8_t source = sample.source;
#else
    const uint8_t source = 0;
#endif
    processBaroObservation(source, sample.pressurePa,
                           sample.temperatureC + 273.15f, sample_ts, dt_s,
                           true);
  }
}

void EskfEstimator::onBaroTrigger(uint64_t trigger_timestamp_us) {
#if !ESKF_USE_BARO
  (void)trigger_timestamp_us;
  return;
#endif
  if (!initialized_ || !in_flight_)
    return;
  if (!shouldStartEskfBaroFusion(trigger_timestamp_us))
    return;
  // Save ESKF state snapshot at trigger time for Innovation Transport
  filter_.triggerBaro(trigger_timestamp_us);
}

void EskfEstimator::processBaroSample(const BaroSample &sample) {
#if !ESKF_USE_BARO
  (void)sample;
  return;
#endif
  if (!initialized_)
    return;

#if APP_TARGET_NATIVE
  const uint8_t source = sample.source;
#else
  const uint8_t source = 0;
#endif
  processBaroObservation(source, sample.pressurePa,
                         sample.temperatureC + 273.15f, sample.timestamp_us,
                         0.020f, false);
}

void EskfEstimator::processGpsSample(const sensors::gnss::GnssSample &sample) {
  if (!initialized_)
    return;

  // GNSS stale/frozen detection: repeated identical packet payload over
  // consecutive polls is treated as hard-fault observation with recovery.
  bool stale_packet = false;
  if (gnss_has_prev_sample_) {
    stale_packet =
        (sample.timestamp_us == gnss_prev_sample_.timestamp_us) &&
        (sample.pps_timestamp_us == gnss_prev_sample_.pps_timestamp_us) &&
        (sample.itow_ms == gnss_prev_sample_.itow_ms) &&
        (sample.lat_deg7 == gnss_prev_sample_.lat_deg7) &&
        (sample.lon_deg7 == gnss_prev_sample_.lon_deg7) &&
        (sample.alt_msl_mm == gnss_prev_sample_.alt_msl_mm) &&
        (sample.vel_n_mms == gnss_prev_sample_.vel_n_mms) &&
        (sample.vel_e_mms == gnss_prev_sample_.vel_e_mms) &&
        (sample.vel_d_mms == gnss_prev_sample_.vel_d_mms) &&
        (sample.fix_type == gnss_prev_sample_.fix_type) &&
        (sample.num_sv == gnss_prev_sample_.num_sv) &&
        (sample.valid == gnss_prev_sample_.valid);
  }
  gnss_prev_sample_ = sample;
  gnss_has_prev_sample_ = true;

  if (stale_packet) {
    if (gnss_stale_counter_ < std::numeric_limits<uint16_t>::max()) {
      gnss_stale_counter_++;
    }
  } else {
    gnss_stale_counter_ = 0;
  }
  const bool gnss_hard_stale = gnss_stale_counter_ >= kGnssStalePersistenceSamples;
  const eskf::SensorHealthState gnss_prev_state = gnss_health_state_;
  const bool reject_gnss = updateSingleSensorHealthFromHardObservation(
      gnss_hard_stale, gnss_health_state_, gnss_hard_fault_counter_,
      gnss_healthy_counter_, kSingleSensorHardSuspectSamples,
      kSingleSensorHardPersistenceSamples, kSingleSensorRecoveryCooldownSamples,
      kSingleSensorRecoveryConfirmSamples);
  if (gnss_health_state_ != gnss_prev_state) {
    const float packed = static_cast<float>(
        (0u << 16) |
        (static_cast<uint8_t>(gnss_prev_state) << 8) |
        static_cast<uint8_t>(gnss_health_state_));
    eskf::getEskfLogger().logEvent(eskf::EskfEventType::GnssHealthTransition,
                                   sample.timestamp_us,
                                   packed);
  }
  if (reject_gnss) {
    return;
  }

  const bool fix_usable = sample.valid && sample.fix_type >= 2;
  if (!fix_usable) {
    const bool edge_to_unusable =
        !gnss_fix_logged_init_ || last_logged_gnss_fix_usable_;
    constexpr uint64_t kFixDropLogIntervalUs = 1000000;
    const bool periodic_unusable_log =
        gnss_fix_logged_init_ && !last_logged_gnss_fix_usable_ &&
        sample.timestamp_us >= last_gnss_fix_drop_log_us_ &&
        (sample.timestamp_us - last_gnss_fix_drop_log_us_) >=
            kFixDropLogIntervalUs;

    if (edge_to_unusable || periodic_unusable_log) {
      const uint32_t packed =
          (sample.valid ? 1u : 0u) |
          (static_cast<uint32_t>(sample.fix_type) << 8) |
          (static_cast<uint32_t>(sample.num_sv) << 16);
      eskf::getEskfLogger().logEvent(eskf::EskfEventType::GnssFixDropped,
                                     sample.timestamp_us,
                                     static_cast<float>(packed));
      last_gnss_fix_drop_log_us_ = sample.timestamp_us;
    }

    gnss_fix_logged_init_ = true;
    last_logged_gnss_fix_usable_ = false;
    return;
  }
  gnss_fix_logged_init_ = true;
  last_logged_gnss_fix_usable_ = true;

  // Convert from scaled integers to floating point
  double latitude_deg = sample.lat_deg7 * 1e-7;
  double longitude_deg = sample.lon_deg7 * 1e-7;
  float altitude_m = sample.alt_msl_mm * 0.001f;
  const double lat_rad = latitude_deg * kDegToRad;
  const double lon_rad = longitude_deg * kDegToRad;

  if (!in_flight_) {
    latest_preflight_gps_anchor_.valid = true;
    latest_preflight_gps_anchor_.timestamp_us = sample.timestamp_us;
    latest_preflight_gps_anchor_.lat_rad = lat_rad;
    latest_preflight_gps_anchor_.lon_rad = lon_rad;
    latest_preflight_gps_anchor_.alt_m = altitude_m;
  }

  // Origin policy:
  // - Preflight: cache latest valid fix and defer anchoring to liftoff.
  // - In-flight without preflight anchor: fall back to continuity-derived origin.
  if (!gps_origin_set_) {
    if (!in_flight_) {
      return;
    }

    const eskf::State &s = filter_.state();

    const bool launch_origin_valid =
        std::isfinite(cfg_.launch_latitude_deg) &&
        std::isfinite(cfg_.launch_longitude_deg) &&
        std::abs(cfg_.launch_latitude_deg) <= 90.0f &&
        std::abs(cfg_.launch_longitude_deg) <= 180.0f;

    // Prefer configured launch-site horizontal origin when available so late
    // GNSS-fix runs remain in the same NED frame as normal preflight-anchored
    // flights. Keep altitude continuity against current ESKF down state.
    if (launch_origin_valid) {
      gps_origin_lat_ = static_cast<double>(cfg_.launch_latitude_deg) * kDegToRad;
      gps_origin_lon_ = static_cast<double>(cfg_.launch_longitude_deg) * kDegToRad;
      gps_origin_alt_ = altitude_m + static_cast<float>(s.p[2]);
      gps_origin_set_ = true;
      filter_.core().setLateGpsOrigin(false);

      if (log_) {
        log_->printf("[ESKF] GPS origin initialized from configured launch site: "
                     "lat=%.6f, lon=%.6f, alt=%.1f\n",
                     gps_origin_lat_ / kDegToRad,
                     gps_origin_lon_ / kDegToRad,
                     gps_origin_alt_);
      }
      return; // Don't use first fix for correction to avoid jump
    }

    gps_origin_lat_ = lat_rad - (static_cast<double>(s.p[0]) / kEarthRadiusM);

    double cos_lat = std::cos(gps_origin_lat_);
    if (std::abs(cos_lat) < 1e-6) {
      cos_lat = (cos_lat >= 0.0) ? 1e-6 : -1e-6;
    }
    gps_origin_lon_ =
        lon_rad - (static_cast<double>(s.p[1]) / (kEarthRadiusM * cos_lat));

    // NED convention: p[2] is Down, and altitude = origin_alt - down.
    gps_origin_alt_ = altitude_m + static_cast<float>(s.p[2]);
    gps_origin_set_ = true;
    filter_.core().setLateGpsOrigin(true);

    if (log_) {
      log_->printf("[ESKF] GPS origin initialized for continuity: lat=%.6f, "
                   "lon=%.6f, alt=%.1f (late=%d)\n",
                   gps_origin_lat_ / kDegToRad, gps_origin_lon_ / kDegToRad,
                   gps_origin_alt_, in_flight_ ? 1 : 0);
    }
    return; // Don't use first fix for correction to avoid jump
  }

  // Convert to NED
  eskf_scalar pos_ned[3], vel_ned[3];
  convertGpsToNed(sample, pos_ned, vel_ned);

  // Position variance from accuracy estimates (mm → m) - use h_acc and v_acc
  float h_acc_m = sample.h_acc_mm * 0.001f;
  float v_acc_m = sample.v_acc_mm * 0.001f;
  eskf_scalar R_pos[3] = {static_cast<eskf_scalar>(h_acc_m * h_acc_m),
                          static_cast<eskf_scalar>(h_acc_m * h_acc_m),
                          static_cast<eskf_scalar>(v_acc_m * v_acc_m)};

  // Velocity variance from speed accuracy (mm/s → m/s)
  float s_acc_mps = sample.s_acc_mms * 0.001f;
  eskf_scalar R_vel[3] = {static_cast<eskf_scalar>(s_acc_mps * s_acc_mps),
                          static_cast<eskf_scalar>(s_acc_mps * s_acc_mps),
                          static_cast<eskf_scalar>(s_acc_mps * s_acc_mps)};

  // Use PPS timestamp if available, otherwise use sample time
  uint64_t pps_ts = sample.pps_timestamp_us > 0 ? sample.pps_timestamp_us
                                                : sample.timestamp_us;

  // Compute measurement timestamp using replay delay convention:
  // measurement_ts = pps_ts + gps_delay_us.
  int64_t measurement_ts_i64 = static_cast<int64_t>(pps_ts) +
                               static_cast<int64_t>(cfg_.gps_delay_us);
  if (measurement_ts_i64 < 0) {
    measurement_ts_i64 = 0;
  }
  const uint64_t measurement_ts = static_cast<uint64_t>(measurement_ts_i64);

  // DescentNav is a real-time output filter, so fuse GNSS at ingress time to
  // avoid replay-style PPS delay from lagging descent response after deploy.
  const uint64_t descent_gnss_ts =
      sample.timestamp_us > 0 ? sample.timestamp_us : measurement_ts;

  latest_descent_gnss_.valid = true;
  latest_descent_gnss_.timestamp_us = descent_gnss_ts;
  for (int i = 0; i < 3; ++i) {
    latest_descent_gnss_.pos_ned[i] = static_cast<float>(pos_ned[i]);
    latest_descent_gnss_.vel_ned[i] = static_cast<float>(vel_ned[i]);
    latest_descent_gnss_.var_pos_ned[i] = std::max(0.25f, static_cast<float>(R_pos[i]));
    latest_descent_gnss_.var_vel_ned[i] = std::max(0.04f, static_cast<float>(R_vel[i]));
  }

  if (descent_mode_active_) {
    const bool need_snap = descent_waiting_initial_gnss_snap_;

    if (need_snap) {
      descent_filter_.hardResetToGnss(descent_gnss_ts,
                                      latest_descent_gnss_.pos_ned,
                                      latest_descent_gnss_.var_pos_ned,
                                      latest_descent_gnss_.vel_ned,
                                      latest_descent_gnss_.var_vel_ned,
                                      false);
      descent_waiting_initial_gnss_snap_ = false;
      if (log_) {
        log_->printf("[ESKF] DescentNav GNSS hard snap (initial) at %llu us\n",
                     static_cast<unsigned long long>(descent_gnss_ts));
      }
    }

    descent_filter_.fuseGnssPosition(descent_gnss_ts,
                                     latest_descent_gnss_.pos_ned,
                                     latest_descent_gnss_.var_pos_ned);
    descent_filter_.fuseGnssVelocity(descent_gnss_ts,
                                     latest_descent_gnss_.vel_ned,
                                     latest_descent_gnss_.var_vel_ned);
    descent_last_gnss_fuse_us_ = descent_gnss_ts;
  }

  // GPS lever arm: prefer dynamic datum-based model when provided.
  eskf_scalar lever_arm[3] = {
      static_cast<eskf_scalar>(calib_cfg_.gps.lever_arm[0]),
      static_cast<eskf_scalar>(calib_cfg_.gps.lever_arm[1]),
      static_cast<eskf_scalar>(calib_cfg_.gps.lever_arm[2])};

  if (calib_cfg_.gps.antenna_position_valid != 0) {
    eskf_scalar cg[3];
    interpolateCgAtTimestamp(measurement_ts, cg);
    for (int i = 0; i < 3; ++i) {
      const eskf_scalar antenna_pos = static_cast<eskf_scalar>(
          calib_cfg_.gps.antenna_position_datum[i]);
      lever_arm[i] = antenna_pos - cg[i];
    }
  }

  // Route GNSS updates through packet API so rewind-consistent heading gating,
  // bootstrap alignment, chi2 rejection, and pos/vel correction ordering are
  // applied in one place.
  filter_.pushGpsPacket(pps_ts, pos_ned, R_pos, vel_ned, R_vel, lever_arm);

  output_dirty_ = true;
}

void EskfEstimator::processMagSample(
    const sensors::mmc5983ma::MmcSample &sample) {
#if !ESKF_USE_MAG
  (void)sample;
  return;
#endif
  if (!initialized_)
    return;

  // Magnetometer stale/frozen detection on raw counts.
  bool stale_sample = false;
  if (mag_has_prev_sample_) {
    stale_sample = (sample.x == mag_prev_x_) && (sample.y == mag_prev_y_) &&
                   (sample.z == mag_prev_z_);
  }
  mag_prev_x_ = sample.x;
  mag_prev_y_ = sample.y;
  mag_prev_z_ = sample.z;
  mag_has_prev_sample_ = true;

  if (stale_sample) {
    if (mag_stale_counter_ < std::numeric_limits<uint16_t>::max()) {
      mag_stale_counter_++;
    }
  } else {
    mag_stale_counter_ = 0;
  }
  const bool mag_hard_stale = mag_stale_counter_ >= kMagStalePersistenceSamples;
  const eskf::SensorHealthState mag_prev_state = mag_health_state_;
  const bool reject_mag = updateSingleSensorHealthFromHardObservation(
      mag_hard_stale, mag_health_state_, mag_hard_fault_counter_,
      mag_healthy_counter_, kSingleSensorHardSuspectSamples,
      kSingleSensorHardPersistenceSamples, kSingleSensorRecoveryCooldownSamples,
      kSingleSensorRecoveryConfirmSamples);
  if (mag_health_state_ != mag_prev_state) {
    const float packed = static_cast<float>(
        (0u << 16) |
        (static_cast<uint8_t>(mag_prev_state) << 8) |
        static_cast<uint8_t>(mag_health_state_));
    eskf::getEskfLogger().logEvent(eskf::EskfEventType::MagHealthTransition,
                                   sample.t_us,
                                   packed);
  }
  if (reject_mag) {
    return;
  }

  // Convert raw counts to microtesla (MMC5983MA: 18-bit, ±800 Gauss = ±80000 µT
  // range)
  constexpr float kScale =
      160000.0f / 131072.0f; // µT per count (centered at 131072)
  constexpr float kOffset = 131072.0f;

  eskf_sensor_t field[3] = {static_cast<float>(sample.x - kOffset) * kScale,
                            static_cast<float>(sample.y - kOffset) * kScale,
                            static_cast<float>(sample.z - kOffset) * kScale};

  // Process through VirtualCompass for calibration and validation
  uint64_t ts = sample.t_us > 0 ? sample.t_us : 0;
  eskf::CompassOutput cout = virtual_compass_.process(field, ts);

  // Skip invalid samples (magnitude or dip angle validation failed)
  if (!cout.valid) {
    return;
  }

  // Select attitude source for tilt-compensated heading calculation
  const eskf_scalar *q =
      in_flight_ ? filter_.state().q : rail_shadow_.quaternion();

  // Calculate tilt-compensated heading
  // Declination is applied after Earth-frame projection (not in body frame)
  // Site-specific declination from TuningConfig
  // Use config directly instead of new TuningConfig
  eskf_scalar heading = eskf::math::calculateTiltCompensatedHeading(
      cout.mag_calibrated, q, cfg_.mag_declination_rad);

  eskf_scalar R_heading = 0.01; // ~6 degrees variance

  if (!in_flight_) {
    // Pre-flight: Update RailShadowFilter's heading state (not ESKF)
    rail_shadow_.updateHeading(heading, R_heading);
  } else {
    // In-flight: Check rail clear delay
    uint64_t rail_clear_time = liftoff_us_ + cfg_.rail_clear_delay_us;

    if (ts < rail_clear_time) {
      // During rewind/rail-clear phase: reject mag to avoid rail interference
      return;
    }

    // Past rail clear: queue mag heading through yieldable event path so
    // heading updates remain replay/rewind consistent.
    filter_.pushMagHeading(heading, R_heading, ts);
  }

  output_dirty_ = true;
}

void EskfEstimator::onTick(uint64_t now_us) {
  if (!initialized_)
    return;

  // Pre-liftoff: Do NOT run catchUp() to prevent ESKF state drift
  // Data is still buffered via pushImu() etc., but not processed
  // The RailShadowFilter handles orientation estimation pre-flight
  if (in_flight_) {
#if APP_TARGET_NATIVE
    // Native replay: process ALL buffered events to prevent buffer overflow.
    // Native runs faster than real-time, so we can't rely on time budgets.
    // Loop until catchUp() returns true (caught up to current time).
    while (!filter_.catchUp(now_us, 0)) {
      catchup_yield_count_++;
    }
#else
    // Production (Teensy): use time budget to avoid blocking the loop too long.
    // This may leave events buffered for the next tick.
    if (!filter_.catchUp(now_us, cfg_.catchup_budget_us)) {
      catchup_yield_count_++;
    }
#endif

    // Log Flight Shadow state (rate-limited)
    logFlightShadowIfDue(now_us);
  } else {
    // Pre-flight: Log Rail Shadow state (rate-limited)
    logRailShadowIfDue(now_us);
  }

  if (descent_mode_active_) {
    descent_filter_.predict(now_us);
  }

  output_dirty_ = true;
}

void EskfEstimator::convertGpsToNed(const sensors::gnss::GnssSample &sample,
                                    eskf_scalar pos_ned[3],
                                    eskf_scalar vel_ned[3]) const {
  // Convert from scaled integers
  double lat_rad = sample.lat_deg7 * 1e-7 * kDegToRad;
  double lon_rad = sample.lon_deg7 * 1e-7 * kDegToRad;
  float altitude_m = sample.alt_msl_mm * 0.001f;

  // Local radius of curvature
  double cos_lat = std::cos(gps_origin_lat_);
  double r_lat = kEarthRadiusM; // Simplified
  double r_lon = kEarthRadiusM * cos_lat;

  // Position offset in NED (meters)
  double d_lat = lat_rad - gps_origin_lat_;
  double d_lon = lon_rad - gps_origin_lon_;

  pos_ned[0] = static_cast<eskf_scalar>(d_lat * r_lat);                // North
  pos_ned[1] = static_cast<eskf_scalar>(d_lon * r_lon);                // East
  pos_ned[2] = static_cast<eskf_scalar>(gps_origin_alt_ - altitude_m); // Down

  // Velocity in NED (GPS provides mm/s, convert to m/s)
  vel_ned[0] = static_cast<eskf_scalar>(sample.vel_n_mms * 0.001);
  vel_ned[1] = static_cast<eskf_scalar>(sample.vel_e_mms * 0.001);
  vel_ned[2] = static_cast<eskf_scalar>(sample.vel_d_mms * 0.001);
}

void EskfEstimator::resetTurnOnAccelBiasEstimator() {
  for (int i = 0; i < 3; ++i) {
    turn_on_accel_bias_[i] = 0;
  }
  turn_on_accel_bias_samples_ = 0;
  turn_on_accel_bias_valid_ = false;
}

void EskfEstimator::updateTurnOnAccelBiasEstimate(const eskf_scalar nav_accel[3],
                                                  const eskf_scalar nav_gyro[3],
                                                  uint64_t sample_timestamp_us) {
  constexpr uint32_t kMinSamplesForValidBias = 256;
  constexpr eskf_scalar kMinAccelNorm = static_cast<eskf_scalar>(1e-4);

  const eskf_scalar g_local = static_cast<eskf_scalar>(cfg_.local_gravity);
  if (!(std::isfinite(g_local) && g_local > 0)) {
    return;
  }

  const eskf_scalar accel_norm_sq =
      nav_accel[0] * nav_accel[0] + nav_accel[1] * nav_accel[1] +
      nav_accel[2] * nav_accel[2];
  if (!(std::isfinite(accel_norm_sq) && accel_norm_sq > kMinAccelNorm * kMinAccelNorm)) {
    return;
  }

  const eskf_scalar accel_norm = std::sqrt(accel_norm_sq);
  const eskf_scalar norm_error = accel_norm - g_local;
  const eskf_scalar gate =
      static_cast<eskf_scalar>(std::max(0.05f, cfg_.shadow_rail_gate));
  if (std::fabs(norm_error) > gate) {
    return;
  }

  const eskf_scalar gyro_norm = std::sqrt(
      nav_gyro[0] * nav_gyro[0] + nav_gyro[1] * nav_gyro[1] +
      nav_gyro[2] * nav_gyro[2]);
  const eskf_scalar gyro_gate = static_cast<eskf_scalar>(
      (cfg_.heading_align_max_gyro > 0.0f) ? cfg_.heading_align_max_gyro
                                           : 0.35f);
  if (!(std::isfinite(gyro_norm) && gyro_norm <= gyro_gate)) {
    return;
  }

  eskf_scalar sample_bias[3] = {
      nav_accel[0] * (norm_error / accel_norm),
      nav_accel[1] * (norm_error / accel_norm),
      nav_accel[2] * (norm_error / accel_norm),
  };

  const eskf_scalar sample_bias_norm = std::sqrt(
      sample_bias[0] * sample_bias[0] + sample_bias[1] * sample_bias[1] +
      sample_bias[2] * sample_bias[2]);
  if (sample_bias_norm > gate && sample_bias_norm > kMinAccelNorm) {
    const eskf_scalar scale = gate / sample_bias_norm;
    sample_bias[0] *= scale;
    sample_bias[1] *= scale;
    sample_bias[2] *= scale;
  }

  const eskf_scalar alpha_raw = static_cast<eskf_scalar>(cfg_.gyro_bias_lpf_alpha);
  const eskf_scalar alpha = std::clamp(alpha_raw, static_cast<eskf_scalar>(0),
                                       static_cast<eskf_scalar>(0.99999));
  for (int i = 0; i < 3; ++i) {
    turn_on_accel_bias_[i] = alpha * turn_on_accel_bias_[i] +
                             (static_cast<eskf_scalar>(1) - alpha) * sample_bias[i];
  }

  if (turn_on_accel_bias_samples_ < UINT32_MAX) {
    turn_on_accel_bias_samples_++;
  }
  const bool was_valid = turn_on_accel_bias_valid_;
  turn_on_accel_bias_valid_ =
      (turn_on_accel_bias_samples_ >= kMinSamplesForValidBias);
  if (!was_valid && turn_on_accel_bias_valid_) {
    eskf::getEskfLogger().logEvent(eskf::EskfEventType::TurnOnAccelBiasLatched,
                                   sample_timestamp_us,
                                   static_cast<float>(turn_on_accel_bias_samples_));
  }
}

EstimatorOutput EskfEstimator::output() const {
  if (output_dirty_) {
    updateCachedOutput();
    output_dirty_ = false;
  }
  return cached_output_;
}

void EskfEstimator::updateCachedOutput() const {
  if (descent_mode_active_ && descent_filter_.isActive() &&
      !descent_waiting_initial_gnss_snap_) {
    const DescentNavFilter::State ds = descent_filter_.state();

    cached_output_.altitude_m = -ds.position_ned[2];
    cached_output_.vertical_velocity_mps = -ds.velocity_ned[2];
    cached_output_.altitude_valid = true;
    cached_output_.velocity_valid = true;

    // Descent mode intentionally does not estimate attitude.
    cached_output_.attitude_valid = false;
    cached_output_.quaternion[0] = 1.0f;
    cached_output_.quaternion[1] = 0.0f;
    cached_output_.quaternion[2] = 0.0f;
    cached_output_.quaternion[3] = 0.0f;

    for (int i = 0; i < 3; ++i) {
      cached_output_.position_ned[i] = ds.position_ned[i];
      cached_output_.velocity_ned[i] = ds.velocity_ned[i];
    }

    cached_output_.last_update_ms =
        static_cast<uint32_t>(ds.timestamp_us / 1000ULL);
    return;
  }

  const eskf::State &s = filter_.state();

  // Basic altitude/velocity (for apogee detection)
  cached_output_.altitude_m =
      static_cast<float>(-s.p[2]); // NED Down → altitude up
  cached_output_.vertical_velocity_mps =
      static_cast<float>(-s.v[2]); // NED Down → velocity up
  cached_output_.altitude_valid = true;
  cached_output_.velocity_valid = true;

  // Attitude
  cached_output_.attitude_valid = true;
  for (int i = 0; i < 4; ++i) {
    cached_output_.quaternion[i] = static_cast<float>(s.q[i]);
  }

  // Full 3D state
  for (int i = 0; i < 3; ++i) {
    cached_output_.position_ned[i] = static_cast<float>(s.p[i]);
    cached_output_.velocity_ned[i] = static_cast<float>(s.v[i]);
  }

  cached_output_.last_update_ms =
      static_cast<uint32_t>(s.timestamp_us / 1000ULL);
}

bool EskfEstimator::isApogeeDetected() const {
  if (!in_flight_)
    return false;

  // Use flight shadow filter for apogee detection (more robust)
  // Apogee = vertical velocity crosses from negative (ascending) to positive
  // (descending) In NED: velocity > threshold means confidently descending
  // (avoid noise chatter)
  return flight_shadow_.velocity() > ESKF_SHADOW_APOGEE_HYSTERESIS;
}

eskf_scalar EskfEstimator::eskfVelocityDown() const {
  // ESKF velocity in NED frame, Z component (positive = down/falling)
  return filter_.state().v[2];
}

eskf_scalar EskfEstimator::shadowVelocityDown() const {
  // Flight shadow filter velocity (NED Down, positive = falling)
  return flight_shadow_.velocity();
}

bool EskfEstimator::isEskfDiverged() const {
  if (filter_.core().hasDiverged()) {
    return true;
  }

  // Check for NaN in critical state elements
  const eskf::State &s = filter_.state();

  // Check velocity components for NaN/Inf
  for (int i = 0; i < 3; ++i) {
    if (std::isnan(s.v[i]) || std::isinf(s.v[i]))
      return true;
    if (std::isnan(s.p[i]) || std::isinf(s.p[i]))
      return true;
  }

  // Check quaternion
  for (int i = 0; i < 4; ++i) {
    if (std::isnan(s.q[i]) || std::isinf(s.q[i]))
      return true;
  }

  return false;
}

bool EskfEstimator::isCoastPhase() const {
  if (!in_flight_)
    return false;

  // Coast phase = body-X acceleration is negative (drag > thrust)
  // During motor burn, body_accel_x > 0 (thrust > drag)
  // After MECO, body_accel_x < 0 (drag > thrust)
  return last_body_accel_x_ < 0;
}

eskf_scalar EskfEstimator::bodyAccelX() const { return last_body_accel_x_; }

bool EskfEstimator::getLiftoffSnapshot(LiftoffSnapshot &out) const {
  if (!has_liftoff_snapshot_) {
    return false;
  }
  out = last_liftoff_snapshot_;
  return true;
}

bool EskfEstimator::getDescentState(DescentNavFilter::State &out) const {
  if (!descent_mode_active_ || !descent_filter_.isActive()) {
    return false;
  }
  out = descent_filter_.state();
  return true;
}

bool EskfEstimator::getDescentStats(DescentNavFilter::Stats &out) const {
  if (!descent_mode_active_ || !descent_filter_.isActive()) {
    return false;
  }
  out = descent_filter_.stats();
  return true;
}

void EskfEstimator::logImuHealthTransitions(const eskf::VirtualImuOutput &vout) {
  for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
    const eskf::SensorHealthState current = vout.imu_health[i];
    if (!imu_health_logged_init_[i]) {
      imu_health_logged_init_[i] = true;
      last_logged_imu_health_[i] = current;
      continue;
    }
    if (current != last_logged_imu_health_[i]) {
      const uint32_t packed_u32 =
          (static_cast<uint32_t>(i) << 16) |
          (static_cast<uint32_t>(last_logged_imu_health_[i]) << 8) |
          static_cast<uint32_t>(current);
      eskf::getEskfLogger().logEvent(
          eskf::EskfEventType::ImuHealthTransition,
          vout.frame.timestamp_us,
          static_cast<float>(packed_u32));
      last_logged_imu_health_[i] = current;
    }
  }

  if (vout.continuity_salvage_used != last_logged_imu_salvage_) {
    if (vout.continuity_salvage_used) {
      eskf::getEskfLogger().logEvent(eskf::EskfEventType::ImuContinuitySalvage,
                                     vout.frame.timestamp_us,
                                     1.0f);
    }
    last_logged_imu_salvage_ = vout.continuity_salvage_used;
  }
}

void EskfEstimator::logBaroHealthTransitions(const eskf::BaroOutput &bout) {
  for (size_t i = 0; i < ESKF_MAX_BAROS; ++i) {
    const eskf::BaroHealthState current = bout.baro_health[i];
    if (!baro_health_logged_init_[i]) {
      baro_health_logged_init_[i] = true;
      last_logged_baro_health_[i] = current;
      continue;
    }
    if (current != last_logged_baro_health_[i]) {
      const uint32_t packed_u32 =
          (static_cast<uint32_t>(i) << 16) |
          (static_cast<uint32_t>(last_logged_baro_health_[i]) << 8) |
          static_cast<uint32_t>(current);
      eskf::getEskfLogger().logEvent(
          eskf::EskfEventType::BaroHealthTransition,
          bout.timestamp_us,
          static_cast<float>(packed_u32));
      last_logged_baro_health_[i] = current;
    }
  }

  if (bout.continuity_salvage_used != last_logged_baro_salvage_) {
    if (bout.continuity_salvage_used) {
      eskf::getEskfLogger().logEvent(eskf::EskfEventType::BaroContinuitySalvage,
                                     bout.timestamp_us,
                                     1.0f);
    }
    last_logged_baro_salvage_ = bout.continuity_salvage_used;
  }
}

void EskfEstimator::logRailShadowIfDue(uint64_t timestamp_us) {
  // Build snapshot from current Rail Shadow state
  eskf::RailShadowSnapshot snapshot;

  // Get combined quaternion (gravity + heading) - use intermediate buffer for
  // type conversion
  eskf_scalar q_temp[4];
  rail_shadow_.getCombinedQuaternion(q_temp);
  for (int i = 0; i < 4; ++i) {
    snapshot.q[i] = static_cast<float>(q_temp[i]);
  }

  // Copy other state
  snapshot.heading_rad = 0.0f;
  snapshot.heading_variance = static_cast<float>(rail_shadow_.headingVariance());

  const eskf_scalar *bias = rail_shadow_.gyroBias();
  for (int i = 0; i < 3; ++i) {
    snapshot.gyro_bias[i] = static_cast<float>(bias[i]);
  }

  const eskf::GroundReference &gref = rail_shadow_.getOldestGroundReference();
  snapshot.ground_pressure_pa =
      gref.valid ? static_cast<float>(gref.pressure_pa) : 0.0f;
  snapshot.window_count = rail_shadow_.groundReferenceWindowCount();

  snapshot.timestamp_us = timestamp_us;
  snapshot.flags = 0;
  if (rail_shadow_.isGateOpen())
    snapshot.flags |= 0x01;
  if (rail_shadow_.isHeadingInitialized())
    snapshot.flags |= 0x02;
  if (rail_shadow_.isGroundReferenceValid())
    snapshot.flags |= 0x04;

  // Log via global logger
  eskf::getEskfLogger().logRailShadow(snapshot);
}

void EskfEstimator::logFlightShadowIfDue(uint64_t timestamp_us) {
  // Build snapshot from current Flight Shadow state
  eskf::FlightShadowSnapshot snapshot;

  snapshot.altitude_m = static_cast<float>(flight_shadow_.altitudeUp());
  snapshot.velocity_mps = static_cast<float>(flight_shadow_.velocity());

  const eskf_scalar *q = flight_shadow_.quaternion();
  for (int i = 0; i < 4; ++i) {
    snapshot.q[i] = static_cast<float>(q[i]);
  }

  snapshot.timestamp_us = timestamp_us;
  snapshot.flags = 0;
  if (flight_shadow_.isAeroBlind())
    snapshot.flags |= 0x01;
  if (flight_shadow_.wasAeroBlind())
    snapshot.flags |= 0x02;
  snapshot.aero_blind_enter_accum_s =
      static_cast<float>(flight_shadow_.aeroBlindEnterAccumS());
  snapshot.aero_blind_exit_accum_s =
      static_cast<float>(flight_shadow_.aeroBlindExitAccumS());
  snapshot.last_reengage_snap_delta_m =
      static_cast<float>(flight_shadow_.lastReengageSnapDeltaM());

  // Log via global logger
  eskf::getEskfLogger().logFlightShadow(snapshot);
}

void EskfEstimator::logImuPipelineIfDue(const eskf::VirtualImuOutput &vout,
                                        eskf_scalar dt_s) {
  eskf::ImuPipelineSnapshot snapshot;
  for (int i = 0; i < 3; ++i) {
    snapshot.accel_body[i] = static_cast<float>(vout.nav_accel[i]);
    snapshot.gyro_body[i] = static_cast<float>(vout.nav_gyro[i]);
    snapshot.omega_dot[i] = static_cast<float>(vout.omega_dot[i]);
    snapshot.accel_cg[i] = static_cast<float>(vout.frame.accel[i]);
    snapshot.gyro_cg[i] = static_cast<float>(vout.frame.gyro[i]);
    snapshot.centroid[i] = static_cast<float>(vout.effective_centroid[i]);
    snapshot.cg[i] = static_cast<float>(vout.cg[i]);
    snapshot.lever_arm[i] = static_cast<float>(vout.lever_arm[i]);
    snapshot.omega_dot_unclamped[i] =
      static_cast<float>(vout.omega_dot_unclamped[i]);
    snapshot.tangential_correction[i] =
      static_cast<float>(vout.tangential_correction[i]);
    snapshot.centripetal_correction[i] =
      static_cast<float>(vout.centripetal_correction[i]);
    snapshot.lever_arm_correction_unclamped[i] =
      static_cast<float>(vout.lever_arm_correction_unclamped[i]);
  }
    const eskf_scalar *gyro_bias =
      in_flight_ ? filter_.state().b_gyro : rail_shadow_.gyroBias();
  for (int i = 0; i < 3; ++i) {
    snapshot.nav_gyro_unbiased[i] =
        static_cast<float>(vout.nav_gyro[i] - gyro_bias[i]);
    snapshot.gyro_bias_body[i] = static_cast<float>(gyro_bias[i]);
    snapshot.lever_arm_correction[i] = static_cast<float>(vout.lever_arm_correction[i]);
  }
    snapshot.omega_dot_norm = static_cast<float>(vout.omega_dot_norm);
    snapshot.omega_dot_unclamped_norm =
      static_cast<float>(vout.omega_dot_unclamped_norm);
    snapshot.lever_arm_correction_norm =
      static_cast<float>(vout.lever_arm_correction_norm);
    snapshot.lever_arm_correction_unclamped_norm =
      static_cast<float>(vout.lever_arm_correction_unclamped_norm);
    snapshot.omega_dot_clipped = vout.omega_dot_clipped ? 1 : 0;
    snapshot.lever_arm_correction_clipped =
      vout.lever_arm_correction_clipped ? 1 : 0;

  eskf_scalar q_gravity[4];
  rail_shadow_.getQuaternion(q_gravity);
  eskf_scalar q_combined[4];
  rail_shadow_.getCombinedQuaternion(q_combined);
  for (int i = 0; i < 4; ++i) {
    snapshot.rail_q_gravity[i] = static_cast<float>(q_gravity[i]);
    snapshot.rail_q_combined[i] = static_cast<float>(q_combined[i]);
  }

  const eskf::RailShadowDebug &dbg = rail_shadow_.getDebugState();
  for (int i = 0; i < 3; ++i) {
    snapshot.rail_expected_accel[i] =
        static_cast<float>(dbg.expected_accel_body[i]);
    snapshot.rail_error[i] = static_cast<float>(dbg.error[i]);
    snapshot.rail_integral[i] = static_cast<float>(dbg.integral[i]);
    snapshot.rail_correction[i] = static_cast<float>(dbg.correction[i]);

    snapshot.imu0_accel_body[i] =
        static_cast<float>(vout.imu_accel_body[0][i]);
    snapshot.imu0_gyro_body[i] =
        static_cast<float>(vout.imu_gyro_body[0][i]);
    snapshot.imu1_accel_body[i] =
        static_cast<float>(vout.imu_accel_body[1][i]);
    snapshot.imu1_gyro_body[i] =
        static_cast<float>(vout.imu_gyro_body[1][i]);
  }

  eskf_scalar accel_norm = std::sqrt(vout.nav_accel[0] * vout.nav_accel[0] +
                                     vout.nav_accel[1] * vout.nav_accel[1] +
                                     vout.nav_accel[2] * vout.nav_accel[2]);
  snapshot.accel_norm = static_cast<float>(accel_norm);
  snapshot.dt_s = static_cast<float>(dt_s);
  snapshot.imu0_present =
      (vout.imu_status[0] != eskf::SensorStatus::HARD_FAIL) ? 1 : 0;
  snapshot.imu1_present =
      (vout.imu_status[1] != eskf::SensorStatus::HARD_FAIL) ? 1 : 0;
  snapshot.valid_imu_count = static_cast<uint8_t>(vout.valid_imu_count);
  snapshot.gate_open = (!in_flight_ && rail_shadow_.isGateOpen()) ? 1 : 0;
  for (size_t i = 0; i < ESKF_MAX_IMUS; ++i) {
    snapshot.imu_status[i] = static_cast<uint8_t>(vout.imu_status[i]);
  }
  snapshot.timestamp_us = vout.frame.timestamp_us;

  eskf::getEskfLogger().logImuPipeline(snapshot);
}

} // namespace app

#endif // APP_TARGET_TEENSY || APP_TARGET_NATIVE
