# Kalman User Manual

Audience: firmware team members who need to operate or debug the Kalman/ESKF
pipeline quickly without knowing the filter maths.

Full details live in `docs/eskf_runtime_logic_reference.md`. Calibration and
tuning ownership lives in `docs/calibration_source_of_truth.md`.

## 1. Open These Files First

- `Application/Kalman/kalman_process.cpp`: runtime loop, sensor draining,
  GOATStore outputs, health snapshot.
- `Application/Kalman/AppLayer/eskf_estimator.cpp`: liftoff, sensor fusion,
  shadow filters, GNSS/baro handling.
- `Application/Kalman/kalman_health.hpp`: app-visible health fields.
- `Application/Kalman/kalman/eskf_logger.hpp`: diagnostic event names.
- `Application/Kalman/AppLayer/replayable_config.hpp`: replayable tuning schema.
- `Application/Kalman/kalman/eskf_tuning_defaults.hpp`: compile-time tuning
  defaults.
- `Application/Kalman/AppLayer/calibration_config.hpp`: calibration schema.
- `Application/Kalman/AppLayer/hw_calibration_data.hpp`: current flight
  calibration defaults.

Start in the app layer. Only go into `Application/Kalman/kalman/` after checking
wiring, timestamps, health counters, and config.

## 2. Normal Runtime Flow

`kalman_loop()` runs once per super-loop iteration:

1. Mirror the FSM state into `EskfEstimator::onFlightStateChange`.
2. Drain IMU rings, ignoring sources marked unhealthy by
   `app_imu_sensor_healthy`.
3. Feed IMU chunks to `processImuBatch`.
4. Consume pending liftoff through `kalman_take_pending_liftoff`.
5. Drain barometer and GNSS data through `ingestAidingFromStore`.
6. Call `EskfEstimator::onTick(now_us)`.
7. Publish navigation to `GOATStore.navigationDataStore`.
8. Publish `apogee_detected` or `catastrophic_failure` when needed.
9. Publish `KalmanHealthSnapshot`.

Important: sensor input is buffered first. The ESKF only processes buffered data
inside `catchUp()`, called from `onTick()`.

## 3. Sensor Rules

IMU:

- Input must already be SI units: accel `m/s^2`, gyro `rad/s`.
- Timestamps must use the shared monotonic microsecond vehicle timebase.
- `VirtualImu` calibrates, rotates to body frame, fuses sources, applies
  CG/lever-arm correction, and rejects unhealthy sensors.
- With 3 or more valid IMUs, voting can identify a bad source. With 1 or 2 IMUs,
  disagreement is only a diagnostic.

Barometer:

- BMP390 conversions use trigger/complete timing.
- `kalman_note_baro_trigger(trigger_us)` records the conversion trigger.
- `processBaroSample` completes the measurement later.
- Wrong trigger timestamps can look like bad altitude fusion.

GNSS:

- Every fix is forwarded to `processGpsSample`, including invalid fixes, so
  stale/fix-drop diagnostics can run.
- GNSS timing uses `measurement_timestamp = pps_timestamp_us + gps_delay_us`.
- Negative `gps_delay_us` means the measurement happened before the packet/PPS
  timestamp.

Magnetometer:

- The library supports it, but current production `kalman_loop()` does not feed
  magnetometer samples. Do not debug current heading as a mag issue unless app
  wiring changed.

## 4. Flight Phases

Preflight:

- ESKF is mostly hibernating.
- Rail Shadow estimates attitude, heading, gyro bias, and ground baro reference.

Liftoff:

- `kalman_on_liftoff(liftoff_ms)` latches liftoff once.
- `EskfEstimator::onLiftoff` injects a `LiftoffSnap` from Rail Shadow state.
- A short rejection window blocks unstable immediate post-liftoff aiding.

In flight:

- IMU predict drives the state.
- Baro, GNSS, heading, and optional sideslip constraints correct it.
- Late GNSS can trigger rewind/replay if its corrected timestamp is behind
  current Kalman time.

Descent:

- A descent-only baro/GNSS filter can provide vertical navigation while the main
  ESKF remains available for attitude and diagnostics.

## 5. First Health Check

Read `KalmanHealthStore::instance().get()`.

- `diverged`: ESKF is unusable; runtime sets `event.catastrophic_failure`.
- `altitude_valid`, `velocity_valid`: app-facing output validity.
- `imu_samples_consumed`: should increase while IMUs produce.
- `baro_updates`: should increase when baro samples arrive.
- `gps_updates`: increases for valid GNSS fixes.
- `imu_ring_hwm[3]`: producer ring high-water marks.
- `last_kalman_loop_us`, `max_kalman_loop_us`: Kalman loop runtime.
- `last_main_loop_iteration_us`, `max_main_loop_iteration_us`: super-loop
  runtime.
- `yieldable_imu_drops`: IMU samples lost inside ESKF rewind history.

Fast interpretation:

- Counters not increasing: producer, wiring, timestamp, or FSM-state problem.
- Ring high-water marks rising: loop budget or sensor-rate problem.
- `yieldable_imu_drops > 0`: GPS rewind history is being overwritten.
- `diverged == true`: inspect the first `NisDivergenceWarning` or invalid-state
  evidence, not only the final failure.

## 6. Log Events To Search

Core lifecycle:

- `FilterReset`, `FilterInitialized`, `ModeChanged`
- `LiftoffSnapInjected`, `RejectionWindowStart`, `RejectionWindowEnd`

Corrections:

- `GpsPosCorrection`, `GpsVelCorrection`, `BaroCorrection`
- `GpsHeadingCorrection`, `SideslipCorrection`

Rejections and health:

- `GpsVelRejected`, `GpsPosRejected`, `GpsRejectedByWindow`
- `GpsCovarianceReset`, `GnssFixDropped`
- `NisDivergenceWarning`, `FilterDiverged`
- `ImuHealthTransition`, `BaroHealthTransition`, `GnssHealthTransition`
- `ImuContinuitySalvage`, `BaroContinuitySalvage`

Timing, buffers, rewind:

- `PredictDtClamped`
- `ImuBufferOverflow`, `BaroBufferOverflow`, `EventBufferOverflow`
- `BaroSnapshotDropped`
- `RewindStarted`, `RewindCompleted`, `RewindNoCheckpoint`, `RewindDataGap`
- `RewindGapCovInflated`, `RewindStateSnapshotPre`,
  `RewindStateSnapshotPost`

Baro and shadow filters:

- `BaroTransonicBlind`, `BaroInnovationClamped`
- `GroundRefCalibrated`, `GroundRefFallbackUsed`
- `AeroBlindEntered`, `AeroBlindExited`, `RailShadowHeadingConverged`

## 7. Debug Playbook

### Output Is Not Updating

Check `kalman_loop()` is being called, `imu_samples_consumed` increases, IMU
source health is true, timestamps are monotonic microseconds, and the FSM did
not reset to `INIT`.

Try verifying `imuData1/2/3` producers, checking loop timings, and reducing
blocking work before changing any Kalman tuning.

### Rings Or Drops Increase

Check `imu_ring_hwm[3]`, `yieldable_imu_drops`, `ImuBufferOverflow`,
`RewindDataGap`, and catch-up yield counts.

Try increasing `ESKF_APP_CATCHUP_BUDGET_US` or replay `catchup_budget_us`,
checking `APP_IMU_PRIMARY_ODR_HZ`, and reducing super-loop load. If
`RewindDataGap` appears, treat rewind output as degraded.

### ESKF Diverges

Check `FilterDiverged`, the first `NisDivergenceWarning`, GPS/baro rejection
bursts, `PredictDtClamped`, and any dropped IMU history.

Likely causes are bad calibration, wrong axis transform, bad timestamps,
incorrect `gps_delay_us`, GNSS lever-arm/sign error, or overconfident noise.

Try checking IMU units, `sensor_to_body`, body-frame gravity sign, and
`gps_delay_us` sign first. Then consider increasing process noise
(`process_noise_accel_noise`, `process_noise_gyro_noise`,
`process_noise_accel_bias_walk`, `process_noise_gyro_bias_walk`) or initial
covariance (`initial_cov_pos`, `initial_cov_vel`, `initial_cov_tilt`,
`initial_cov_heading`, `initial_cov_accel_bias`, `initial_cov_gyro_bias`).
Do not widen gates before checking calibration and timing.

### GNSS Is Rejected Too Often

Check `GpsVelRejected`, `GpsPosRejected`, `GpsRejectedByWindow`,
`GpsCovarianceReset`, `GnssFixDropped`, GNSS fix quality, accuracy fields,
timestamp provenance, `RewindStarted`, and `RewindDataGap`.

Try tuning `gps_delay_us`, checking antenna lever arm and CG table, increasing
`gps_trust_factor` if accuracies are too optimistic, and reviewing
`gps_high_g_threshold`, `gps_high_g_r_factor`, `gps_vel_tumble_gyro_threshold`,
and `gps_vel_tumble_r_factor`. Adjust chi2 gates only after timing and lever arm
are credible.

Useful isolation flags: `disable_gps_after_alignment` and
`disable_gps_vel_lever_arm_attitude_jacobian`.

### Heading Is Wrong Or Never Aligns

Check `HeadingAligned`, `HeadingSnapped`, `HeadingInitialized`,
`GpsHeadingCorrection`, GNSS speed, speed accuracy, and rail heading config.

Try checking `heading_align_min_speed`, `heading_align_max_sacc`,
`heading_align_max_gyro`, `launch_rail_heading`, `trust_hardcoded_heading`,
`enable_gps_cog_heading`, `gps_heading_bootstrap_mode`, and
`enable_gps_cog_heading_fusion`.

### Baro Causes Jumps

Check `BaroCorrection`, `BaroInnovationClamped`, `BaroTransonicBlind`,
`BaroSnapshotDropped`, `BaroHealthTransition`, `GroundRefCalibrated`,
`GroundRefFallbackUsed`, and aero-blind events.

Try checking the trigger timestamp, ground reference, per-sensor
`pressure_bias_pa`, `pressure_scale`, `temperature_bias_k`,
`baro_sigma_base`, `baro_k_aero`, `baro_transonic_low`,
`baro_transonic_high`, `baro_transonic_penalty`, `baro_innovation_clamp`,
`baro_voting_threshold_pa`, `baro_hard_fault_threshold_pa`, and
`baro_calibration_mismatch_tolerance_pa`.

Keep `static_pressure.cp_coefficient = 0` for maiden and early validation
flights unless dynamic data proves a non-zero value.

### One IMU Or Baro Goes Unhealthy

Check `ImuHealthTransition`, `BaroHealthTransition`, salvage events, saturation,
stale/frozen samples, and per-sensor raw versus calibrated traces.

Try checking IMU `accel_bias`, `accel_transform`, `gyro_bias`, `gyro_scale`,
`sensor_to_body`, baro bias/scale, and temperature bias. For high-dynamics false
positives, review `imu_boost_soft_threshold_scale`, `imu_boost_soft_window_us`,
`imu_accel_saturation_threshold`, `imu_gyro_saturation_threshold`, and
`imu_saturation_multi_axis_limit`.

### Apogee Or Catastrophic Flags Look Wrong

Check `GOATStore.eventStore.apogee_detected`,
`GOATStore.eventStore.catastrophic_failure`, `KalmanHealthSnapshot.diverged`,
`EstimatorOutput.eskf_valid`, `EstimatorOutput.shadow_valid`, and lifecycle calls
in `kalman_lifecycle.cpp`.

Try confirming `kalman_on_state_change` and `kalman_on_liftoff` are called at
the intended FSM transitions. If only ESKF is bad, compare with Flight Shadow
validity and vertical velocity.

## 8. Config Cheat Sheet

Calibration fields live in `CalibrationConfig` and flight defaults come from
`hw_calibration_data.hpp`. Most urgent fields are:

- IMU: `accel_bias`, `accel_transform`, `gyro_bias`, `gyro_scale`,
  `gyro_g_sensitivity`, `sensor_to_body`, `position`.
- Baro: `pressure_bias_pa`, `pressure_scale`, `temperature_bias_k`.
- GNSS/geometry: `antenna_position_datum`, `antenna_position_valid`,
  `lever_arm`, `cg_table`.
- Static pressure: `static_pressure.cp_coefficient`,
  `static_pressure.air_density`.

Tuning fields live in `ReplayableConfig` and defaults come from
`eskf_tuning_defaults.hpp`. Main categories are:

- IMU voting, fault, saturation, tare, and boost thresholds.
- Baro dynamic variance, transonic penalty, innovation clamp, and multi-baro
  hardening.
- GNSS trust, high-g inflation, chi2 gates, delay, rejection/reset policy.
- Heading bootstrap and GPS COG heading fusion.
- Shadow filter gains and aero-blind behavior.
- Buffer sizes, checkpoint interval, liftoff rejection timing.
- Process noise and initial covariance.

Flight firmware currently uses committed C++ defaults/calibration data. Native
replay can override replayable config values.

## 9. Rush Debug Order

1. Confirm `kalman_loop()` runs and `KalmanHealthSnapshot` updates.
2. Confirm IMU, baro, and GNSS counters increase as expected.
3. Check all timestamps use the same monotonic microsecond timebase.
4. Check health, rejection, buffer, and rewind events before tuning.
5. Check frame conventions: body X forward, Y right, Z down; navigation NED;
   stationary level accel should be about `+9.81 m/s^2` on body Z.
6. Check GNSS timing and lever arm before loosening GPS gates.
7. Check baro trigger timestamp and ground reference before changing baro noise.
8. If the filter is overconfident, increase process noise or initial covariance
   before widening rejection gates.
9. Reproduce in native replay when possible and only commit values backed by
   logged evidence.

## 10. Do Not

- Do not hide unexplained sensor disagreement by widening gates.
- Do not use non-zero `cp_coefficient` on early flights without evidence.
- Do not trust GPS rewind after `RewindDataGap` as if full history existed.
- Do not debug heading as a magnetometer issue unless mag samples are actually
  wired into production `kalman_loop()`.
- Do not retune many categories from one flight. Prioritize timing,
  calibration, dynamic noise inflation, and gross gate mistakes.
