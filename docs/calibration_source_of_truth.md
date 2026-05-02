# Calibration And Kalman Tuning Source Of Truth

## 1. Purpose And Precedence

This document is the single source of truth for calibration and Kalman tuning in
the `2026_C_AV_FC` project.

It covers:

- Sensor calibration for IMUs, barometers, GNSS geometry/timing, and future
  magnetometer integration.
- Kalman tuning for noise, covariance, gates, phase-dependent behavior, and
  multi-sensor hardening.
- Data collection constraints and the expected offline workflow.

If another document conflicts with this one, this document wins for the
flight-computer calibration and tuning process.

Code-reference snapshot note:

- File paths are convenience references. If files move during refactors, the
  ownership and data-contract statements here remain normative until this
  document is updated.

## 2. Scope, Acronyms, And Terms

## 2.1 Acronyms And Terms

- ESKF: Error-State Kalman Filter.
- NIS: Normalized Innovation Squared.
- ODR: Output Data Rate.
- OSR: Over-Sampling Rate.
- IIR: Infinite Impulse Response.
- Cp: Static pressure coefficient used in aerodynamic pressure compensation.
- PPS: Pulse-Per-Second GNSS timing signal.
- COG: Course over ground.

In scope:

- How to collect calibration and tuning datasets.
- Which values are mandatory for safe operation.
- Which values are optional refinements.
- Where each value is consumed in the current code.
- Required offline workflow from logged data to calibrated/tuned config.

Out of scope:

- Implementing new tools in this document.
- Changing firmware behavior in this document.
- Freezing final SD-card file formats or replay decoders before the logging
  format is complete.

## 3. Current Code Truth: Parameter Ownership

## 3.1 Sensor calibration parameters

Persistent schema:

- `appcfg::CalibrationConfig` in
  `Application/Kalman/AppLayer/calibration_config.hpp`.

Current compile-time source for flight firmware defaults:

- `Application/Kalman/AppLayer/hw_calibration_data.hpp`.

Runtime consumers:

- IMU calibration pipeline in
  `Application/Kalman/kalman/preprocessor/virtual_imu.cpp`.
- Baro calibration and static pressure compensation through the virtual baro
  path and `Application/Kalman/AppLayer/eskf_estimator.cpp`.
- Magnetometer calibration through `VirtualCompass` when a magnetometer stream
  is available.
- GNSS lever arm in ESKF updates through
  `Application/Kalman/AppLayer/eskf_estimator.cpp` and
  `Application/Kalman/kalman/eskf_core.cpp`.

Important hardware note:

- The Kalman library supports up to `ESKF_MAX_IMUS = 4` IMUs and
  `ESKF_MAX_BAROS = 4` barometers.
- The barometer set is BMP390.
- The current vehicle has no flight magnetometer requirement. The
  magnetometer sections remain in this document because the Kalman library and
  future hardware paths support magnetometer heading.

## 3.2 Kalman tuning parameters

Persistent/replayable schema:

- `appcfg::ReplayableConfig` in
  `Application/Kalman/AppLayer/replayable_config.hpp`.

Runtime tuning object:

- `eskf::TuningConfig` in
  `Application/Kalman/kalman/eskf_tuning_config.hpp`.

Default values:

- `Application/Kalman/kalman/eskf_tuning_defaults.hpp`.
- `Application/Kalman/kalman/eskf_types.hpp` for process-noise and initial
  covariance defaults.

Runtime consumers:

- ESKF core behavior in `Application/Kalman/kalman/eskf_core.cpp`.
- Rewind/gating/packet rejection logic in
  `Application/Kalman/kalman/eskf_yieldable.cpp`.
- Rail/flight shadow filters in `Application/Kalman/kalman/shadow_filter.cpp`
  when present in the current tree.

## 3.3 Replay-configurable Kalman process/covariance parameters

`ReplayableConfig` includes replay-configurable process-noise and
initial-covariance fields:

- `process_noise_accel_noise`
- `process_noise_gyro_noise`
- `process_noise_accel_bias_walk`
- `process_noise_gyro_bias_walk`
- `process_noise_baro_bias_walk`
- `initial_cov_*`

Compile-time defaults still originate from the ESKF default configuration. Any
offline replay or simulation path that overrides these fields must keep a clear
trace to the values committed for flight firmware.

## 4. Coordinate Conventions And Frames

Use these conventions consistently across all calibration scripts,
measurements, and reports:

- Body frame: X forward along the nose/thrust axis, Y right, Z down.
- Navigation frame: NED.
- Sensor frame: native axis convention from each sensor datasheet/driver.
- Geometry datum D: fixed physical reference on the airframe, such as an
  avionics bulkhead face. All sensor positions and CG-table positions are
  expressed in body frame relative to datum D.
- Runtime lever-arm definition:
  `lever_arm_sensor(t) = sensor_position_datum - cg_position_datum(t)`.
- Gravity sign sanity check: for a stationary, level vehicle in this
  convention, accelerometer specific force is approximately `+g` on body Z
  (about `+9.81 m/s^2`).
- Angles are radians inside filter/tuning. Human reports may include degrees.

Implementation status:

- Virtual IMU consumes a CG table through runtime interpolation when configured.
- GNSS prefers datum-based dynamic lever arm when
  `gps.antenna_position_datum` is valid, using
  `antenna_position_datum - cg(t)`.
- `CalibrationConfig.gps.lever_arm[3]` remains the legacy/fallback field and
  should still be populated.

## 5. Hard Acquisition Constraints

These operational constraints define valid calibration and tuning data.

## 5.1 IMU constraints

From the app configuration and IMU module:

- Kalman core maximum: up to 4 simultaneous IMUs (`ESKF_MAX_IMUS = 4`).
- Active app wiring may instantiate fewer sensors on a given hardware revision.
  Calibrate every populated IMU and record which indices were active.
- ODR: 6400 Hz primary IMU path.
- Full-scale ranges in the current app defaults: accel 32 g, gyro 4000 dps.
- IMU samples are expected in SI units before entering the Kalman app layer
  (`m/s^2`, `rad/s`, Kelvin when temperature is available).
- FSYNC and timestamp alignment are part of the acquisition contract when
  enabled on hardware.

Current fusion behavior in firmware:

- Each IMU sample is calibrated in sensor frame, rotated to body frame, then
  fused by the virtual IMU.
- With three or more valid IMUs, voting/outlier logic can reject a disagreeing
  sensor.
- With one or two valid IMUs, voting is inherently weaker. Treat pairwise
  disagreement as a health diagnostic, not a majority decision.

Acquisition requirement:

- Calibration datasets must be logged at the flight ODR and range unless the
  campaign explicitly studies setting changes.
- Preserve raw or driver-reported temperature whenever thermal correction is
  being characterized.
- Preserve per-sensor source index, timestamps, status flags, saturation flags,
  and drop/alignment counters where available.

## 5.2 Barometer constraints

From the app layer and BMP390 driver configuration:

- Barometer family: BMP390.
- Kalman/app maximum: up to 4 simultaneous barometers.
- Current module triggers initialized BMP390 sensors in forced/command style and
  assigns samples the shared trigger timestamp.
- Current default OSR/IIR tuple in the app module: pressure x4, temperature x1,
  IIR off.
- BMP390 build defaults target pressure OSR x4, temperature OSR x1, IIR off,
  and a conservative command-rate limit.

Acquisition requirement:

- Use the production-like forced/command trigger path for calibration and tuning
  campaigns unless the campaign explicitly characterizes an alternate mode.
- Preserve one timestamp per conversion epoch and per-sensor status/health
  flags.
- For multi-baro datasets, preserve all four sensor identities even if one is
  unhealthy or absent.

Rationale:

- Simultaneous forced-mode triggering gives a clear timestamp model for
  multi-baro fusion and reduces ambiguity versus batched FIFO behavior.

## 5.3 Magnetometer constraints for future use

The current hardware plan does not depend on a magnetometer. However, the
Kalman library supports magnetometer heading through `VirtualCompass`, and the
configuration schema still has magnetometer calibration and site fields.

If a magnetometer is added later:

- Calibrate it in the final assembled and powered avionics configuration.
- Record hard-iron bias, soft-iron matrix, variance, and sensor-to-body
  transform.
- Record site magnetic declination, expected field magnitude, field-magnitude
  threshold, and dip-angle expectations.
- Do not enable flight-critical magnetometer heading unless the installed
  magnetic environment is validated.

Fallback for current operation:

- Use GNSS COG heading alignment and configured rail/launcher heading fallback.
- Keep magnetometer-specific values documented for traceability, but do not let
  them imply a hardware dependency on the current vehicle.

## 5.4 GNSS constraints

The estimator expects GNSS measurements to carry a vehicle-time timestamp and
validity/accuracy metadata.

Acquisition requirement:

- Preserve GNSS fix validity, position, velocity, speed/course quality, accuracy
  estimates, and timestamp provenance.
- If PPS is available, timestamp it with a dedicated hardware path and record
  whether a packet used PPS-aligned time or an arrival-time fallback.
- If PPS is not yet integrated or validated, mark timestamp provenance clearly
  in the dataset metadata.

## 6. Required Physical Measurements

These measurements are mandatory before final flight calibration lock.

## 6.1 Sensor and antenna geometry

Measure in body frame, relative to fixed datum D:

- IMU0 position from datum D.
- IMU1 position from datum D.
- IMU2 position from datum D, if populated.
- IMU3 position from datum D, if populated.
- Barometer static-port/effective-pressure location metadata, if useful for
  interpreting aero pressure errors.
- GNSS antenna phase center from datum D.
- Future magnetometer position and mounting transform, if a magnetometer is
  added.

Calibration output requirement:

- Always produce absolute datum-referenced positions for all populated sensors.
- Always produce `lever_arm_sensor(t)` derived from datum positions and CG table.

Current mapping to firmware fields:

- `CalibrationConfig.imu[i].position[3]`: store IMU datum-referenced position
  for each supported/calibrated IMU index.
- `CalibrationConfig.gps.antenna_position_datum[3]`: preferred
  datum-referenced GNSS antenna phase-center position.
- `CalibrationConfig.gps.antenna_position_valid`: set to 1 when the
  datum-referenced antenna position is valid.
- `CalibrationConfig.gps.lever_arm[3]`: legacy fallback field expecting a
  CG-referenced lever arm; populate with lever arm at `t=0` for compatibility.

Implementation warning:

- If current schema capacity is lower than the hardware population on a given
  branch, extend the schema before final calibration lock rather than silently
  dropping calibrated sensors.

## 6.2 CG table

If propellant burn significantly shifts CG, build a piecewise table:

- `CalibrationConfig.cg_table_size`
- `CalibrationConfig.cg_table[]` with `{time_ms, position[3]}` in the same datum
  D used for sensor geometry.

Fallback:

- Single fixed CG entry at `t = 0`.

Runtime consumption:

- Virtual IMU and datum-based GNSS lever-arm paths use interpolated CG data when
  available.
- If only one CG entry exists, it behaves as a fixed-CG model.

## 6.3 Launch site priors and heading bootstrap inputs

Per site, obtain and freeze in the release package:

- Approximate launch latitude/longitude.
- Approximate launch pad altitude above mean sea level (MSL), at least as
  metadata.
- Local gravity used by the filter.
- Initial rail/launcher heading, true heading in radians from North.
- Magnetic declination, field magnitude, field threshold, and dip angle for
  future magnetometer use.

Map to current fields when available:

- `ReplayableConfig.launch_latitude_deg`
- `ReplayableConfig.launch_longitude_deg`
- `ReplayableConfig.local_gravity`
- `ReplayableConfig.launch_rail_heading`
- `ReplayableConfig.mag_declination_rad`
- `ReplayableConfig.mag_expected_magnitude_ut`
- `ReplayableConfig.mag_magnitude_threshold_ut`
- `ReplayableConfig.expected_dip_angle_rad`
- `ReplayableConfig.dip_angle_threshold_rad`

No dedicated runtime field may exist for launch-site altitude. Keep it in
campaign metadata and release reports.

How the flight computer uses each parameter:

- `launch_latitude_deg`: site descriptor and source for gravity defaults.
- `launch_longitude_deg`: stored/configured for site traceability and future
  geophysical use.
- Launch altitude: currently metadata for traceability, geodetic processing, and
  future model updates.
- `local_gravity`: used directly by propagation and shadow filters.
- `launch_rail_heading`: fallback yaw before heading is initialized from GNSS
  COG or a future magnetometer path.
- Magnetometer site parameters: reserved for `VirtualCompass` validation and
  heading correction when a magnetometer is present.

Precision requirements:

- Launch latitude: recommended within `+/-0.1 deg`, acceptable within
  `+/-0.5 deg` if `local_gravity` is explicitly set from a trusted calculation.
- Launch longitude: recommended within `+/-0.2 deg` for metadata quality,
  acceptable within `+/-1.0 deg`.
- Launch altitude metadata: recommended within `+/-20 m`, acceptable within
  `+/-100 m`.
- Initial rail heading: recommended within `+/-5 deg`, acceptable within
  `+/-10 deg` if used only as temporary bootstrap. Use `+/-2 deg` when fallback
  heading may persist longer.
- Magnetic declination for future mag use: recommended within `+/-0.5 deg`.

Consistency rule:

- When launch site changes, update all linked site priors together in the same
  release bundle.

## 6.4 Sensor-to-body mounting transforms

For each sensor, define a fixed orthonormal transform from sensor frame to body
frame.

Required outputs:

- `R_imu_sensor_to_body[i]` for each populated IMU.
- `R_mag_sensor_to_body` if a magnetometer is added.

Rationale:

- Current firmware identifies accel ellipsoid and gyro bias/scale parameters in
  sensor frame, then rotates calibrated vectors to body frame.
- Rotating raw measurements to body frame before sensor-frame calibration would
  invalidate fitted `T` and `b` unless calibration is re-identified in that
  rotated frame.

Runtime consumption:

- Virtual IMU populates each IMU `sensor_to_body` transform from
  `CalibrationConfig.imu[i].sensor_to_body`.
- Virtual Compass uses `CalibrationConfig.mag.sensor_to_body` if magnetometer
  samples are processed.

## 7. Dataset Campaign Matrix And Parallelism

All campaigns must be logged as raw flight-computer data first. No in-firmware
fitting is allowed.

## 7.1 Campaigns

- Campaign A: IMU thermal drift.
- Campaign B: IMU static multi-orientation, for accel ellipsoid and static gyro
  bias.
- Campaign C: IMU gyro calibration trajectories using static -> move -> static
  sequences, with optional known-angle validation runs.
- Campaign D: Future magnetometer hard/soft iron figure-8 in final flight
  configuration.
- Campaign E: GNSS timing and lag rocking/motion test.
- Campaign F: Flight-like dynamic run for Kalman tuning and gates.
- Campaign G: Long-duration static log for Allan variance.
- Campaign H: Vibration and aero-acoustic characterization run.
- Campaign I: Static pressure reference campaign for BMP390 bias/scale.
- Campaign J: Integrated end-to-end flight test on a smaller or lower-risk
  rocket with production-like firmware and logging.

## 7.2 Parallelization from shared datasets

The workflow must allow one decoded dataset to feed multiple steps in parallel:

- Campaign B feeds accel ellipsoid and static gyro bias estimates.
- Campaign F feeds baro dynamic-R tuning, GPS gates, heading gates, and
  sideslip tuning.
- Campaign E feeds GPS delay verification and lever-arm sanity checks.
- Campaign G feeds Allan-deviation fitting for accel/gyro white noise and bias
  random walks.
- Campaign H feeds vibration-noise inflation factors and anti-aliasing/tuning
  stress limits.
- Campaign I feeds baro static bias/scale estimation.
- Campaign J feeds final constrained tuning updates and acceptance-gate
  verification under real flight dynamics.

Minimum requirement:

- One canonical decoded dataset folder per run, reusable by multiple offline
  analyzers.

## 7.3 Campaign-to-step mapping

- Step 1 uses Campaign A.
- Step 2 uses Campaign B.
- Step 3 uses Campaign C and Campaign B static segments.
- Step 4 uses Campaign F and optionally Campaign J.
- Step 5 uses Campaign D only if a magnetometer is installed.
- Step 6 uses Campaign I for static terms and Campaign F or J for optional Cp.
- Step 7 uses Campaign E and validates again on Campaign J.
- Step 8 uses Campaign G.
- Step 9 uses Campaign F, H, and J.
- Step 10 uses Campaign H and J.

## 8. Sensor Calibration Procedure

Criticality levels:

- C0: Mandatory for safe/reliable estimation.
- C1: Strongly recommended for robust performance.
- C2: Optional refinement.
- Future: Required only if the corresponding hardware is added/enabled.

## Step 1: IMU thermal calibration (C1)

Objective:

- Estimate temperature-dependent accel and gyro bias drift for each populated
  IMU.

Data collection:

- Cold soak then warm ramp.
- Keep unit static.
- Duration target: 30 to 90 min with broad temperature span.
- Log raw/SI IMU data and temperature continuously.

Model:

- For each axis, fit a cubic polynomial in `dT = T - T_ref`:
  `bias_offset(dT) = a*dT^3 + b*dT^2 + c*dT`.
- The constant term is intentionally zero. Static `accel_bias` and `gyro_bias`
  capture the baseline at `T_ref`.

Output fields:

- `imu[i].reference_temp_k`
- `imu[i].accel_thermal[3][4]`
- `imu[i].gyro_thermal[3][4]`
- `imu[i].thermal_valid_min_temp_k`
- `imu[i].thermal_valid_max_temp_k`
- `imu[i].thermal_calibrated = 1`

Fallback:

- If thermal campaign is unavailable, set thermal terms to zero and keep
  `thermal_calibrated = 0`. This is acceptable only for short,
  narrow-temperature test windows.

## Step 2: Accelerometer ellipsoid fit with triangular constraint (C0)

Objective:

- Estimate accel bias and 3x3 transform robustly and unambiguously for each
  populated IMU.

Data collection:

- At least 20 static orientations, including cardinal and mixed attitudes.
- Hold each pose 1 to 3 s.
- Use the same ODR/range as the flight configuration.

Model and constraint:

- Firmware calibration equation:
  `a_cal = T * (a_raw - b - b_thermal)`.
- Enforce triangular structure on `T` during solve to remove rotational
  ambiguity.
- Recommended parameterization: lower-triangular `T` with positive diagonal
  terms.

Acceptance criteria:

- Norm error after calibration should cluster around local `g`.
- Residuals should not show orientation-dependent bias patterns.

Output fields:

- `imu[i].accel_bias[3]`
- `imu[i].accel_transform[3][3]`
- `imu[i].ellipsoid_calibrated = 1`

Fallback:

- Six-face scale+bias calibration is a temporary fallback only. Do not skip bias
  estimation.

## Step 3: Gyro static bias and scale (C0)

Objective:

- Estimate gyro calibration parameters using accelerometer-constrained
  orientation consistency.

Primary method:

- Build many `static -> move -> static` sequences with broad orientation
  coverage.
- Use calibrated accelerometer data from Step 2 as the gravity-reference
  observation at static endpoints.
- Integrate gyro during move segments and optimize gyro parameters to minimize
  endpoint mismatch.

Data collection:

- At least 30 to 50 static/move/static segments with varied axis excitation.
- Static holds should be long enough for robust averaging and static detection.
- Move segments should avoid sustained linear acceleration when possible.
- Optional known-angle rotations can be added as external validation.

Field-expedient scale check:

- Perform multiple full 360-degree rotations on each axis.
- Integrate gyro rate over each rotation window and compare to `2*pi rad`.

Output fields:

- `imu[i].gyro_bias[3]`
- `imu[i].gyro_scale[3]`

Optional offline artifact:

- `T_gyro_full[3][3]` and fit residual report, even if runtime consumes only
  diagonal `gyro_scale[3]`.

Fallback:

- If trajectory quality is insufficient, use known-angle rotations plus static
  bias estimation.
- If neither dynamic nor known-angle data is reliable, lock scale to 1.0,
  calibrate bias only, and inflate gyro process-noise terms.

Runtime gap:

- Runtime schema supports `gyro_bias[3]` and diagonal `gyro_scale[3]`, not a
  full gyro 3x3 transform. Record significant cross-axis coupling in the release
  report.

## Step 4: Inter-IMU consistency and alignment sanity (C1)

Objective:

- Ensure all populated IMUs agree after per-IMU calibration.

Data collection:

- Smooth multi-axis motion with all populated IMUs active.

Checks:

- Compare calibrated body-frame accel/gyro traces by source index.
- Check for persistent axis sign, permutation, timestamp, or mounting-transform
  mistakes.
- Verify voting thresholds do not reject healthy sensors during normal motion.

Fallback:

- Run single-IMU or reduced-IMU debug campaigns until the mismatch is explained.
  Do not mask an unexplained sensor disagreement by tuning gates wider.

## Step 5: Magnetometer hard/soft iron (Future)

Objective:

- Calibrate magnetometer heading only if a magnetometer is installed and intended
  for use.

Data collection:

- Outdoor, away from steel/rebar and strong current sources.
- Full assembled avionics, powered on.
- Telemetry/RF links transmitting at flight-equivalent power.
- Figure-8 and 3D orientation coverage, target about 2000 samples.

Output fields:

- `mag.hard_iron_bias[3]`
- `mag.soft_iron_matrix[3][3]`
- `mag.variance_ut_sq`
- `mag.sensor_to_body[3][3]`

Site-specific tuning:

- `mag_declination_rad`
- `mag_expected_magnitude_ut`
- `mag_magnitude_threshold_ut`
- `expected_dip_angle_rad`
- `dip_angle_threshold_rad`

Fallback:

- For the current no-magnetometer vehicle, keep magnetometer heading disabled and
  rely on GNSS COG heading gates plus configured rail heading.

## Step 6: Barometer static bias/scale and optional Cp (C1 for bias/scale, C2 for Cp)

Objective:

- Correct BMP390 static pressure and temperature offsets.
- Optionally model static-port aerodynamic error.

Data collection:

- Campaign I static reference pressure/temperature comparison for each populated
  BMP390.
- Flight-like dynamic data for Cp only.

Fallback for limited lab equipment:

- If no precision pressure chamber is available, use a co-located high-quality
  reference barometer/weather station and long-duration static logging.
- If no reliable dynamic campaign is available, keep `cp_coefficient = 0`.

Output fields:

- `baro[i].pressure_bias_pa`
- `baro[i].pressure_scale`
- `baro[i].temperature_bias_k`
- `static_pressure.cp_coefficient`
- `static_pressure.air_density`

Rule:

- Keep `cp_coefficient = 0` for maiden and early validation flights unless
  strong evidence supports non-zero correction.

## Step 7: GNSS timing and lever-arm calibration (C0)

Objective:

- Validate GNSS timestamp behavior and lever-arm compensation quality.

Data collection:

- Dynamic rocking/motion run with the production timestamp path.
- Sufficient rotational and translational excitation.
- PPS-aligned data if PPS is present and validated.

Current runtime truth:

- Replay/tuning delay parameter is `gps_delay_us`.
- Sign convention:
  `measurement_timestamp = pps_timestamp_us + gps_delay_us`.
- Negative `gps_delay_us` means the physical GNSS measurement occurred earlier
  than the PPS-aligned packet timestamp and is shifted backward in time.

Output fields:

- `gps.antenna_position_datum[3]`
- `gps.antenna_position_valid`
- `gps.lever_arm[3]`
- `gps_delay_us` and gate parameters in `ReplayableConfig`

Fallback:

- If lag estimate is uncertain, keep conservative delay and increase measurement
  noise inflation instead of applying aggressive timing shifts.

## Step 8: Long static Allan-variance campaign (C0)

Objective:

- Derive stochastic IMU noise terms from data.

Data collection:

- Power on the vehicle and leave it completely untouched.
- Use a thermally stable environment.
- Duration target: 2 to 12 hours continuous logging.
- Use flight IMU ODR/range and timestamp path.

Analysis outputs:

- Allan-deviation curves per accel and gyro axis.
- Fitted white-noise and bias-instability/random-walk terms.
- Recommended baseline process-noise values.

Runtime consumption:

- Allan-derived values feed `ReplayableConfig` process-noise fields for offline
  replay/simulation and compile-time defaults for flight firmware.

## Step 9: Gyroscope g-sensitivity characterization (C2)

Objective:

- Quantify acceleration-dependent gyro bias shifts during high longitudinal
  acceleration.

Data collection options:

- Preferred: centrifuge or controlled high-g fixture.
- Acceptable: instrumented propulsion ground test with synchronized IMU/accel
  logging.
- Simplified fallback: infer residual signature from Campaign J ascent logs.

Outputs:

- `gyro_g_sensitivity_matrix` with confidence bounds.
- Recommendation whether the term can be neglected for this vehicle class.

Runtime consumption:

- `CalibrationConfig.imu[i].gyro_g_sensitivity[3][3]` is consumed by the Virtual
  IMU preprocessor as `omega_corr = omega_meas - G * a_cal`.
- If unavailable, keep the matrix at zero and account for residual risk in
  process-noise margins.

## Step 10: Vibration and aero-acoustic noise characterization (C0 for final tuning release)

Objective:

- Bridge the gap between static Allan-derived noise and the in-flight vibration
  environment.

Data collection options:

- Preferred: shaker profile representative of ascent vibration spectrum.
- Acceptable: static motor-fire instrumentation run.
- Simplified fallback: controlled bench vibration source plus Campaign J flight
  residual analysis.

Outputs:

- Frequency-band noise envelope and aliasing risk summary.
- Multipliers/inflation factors applied to static Allan-derived baselines.
- Updated tuning recommendations for robust high-dynamics operation.

Minimum policy when high-fidelity vibration equipment is unavailable:

- Apply conservative inflation before flight, at least `2x` on white-noise terms
  and `2x` on bias-walk terms, then tighten after Campaign J evidence.

## 9. Kalman Tuning Procedure

Tune with decoded logs and offline/native replay when available before updating
flight defaults.

## 9.1 Baseline categories and authoritative value source

Authoritative numeric defaults are in code, not in this document:

- `Application/Kalman/kalman/eskf_tuning_defaults.hpp`
- `Application/Kalman/kalman/eskf_types.hpp`
- `Application/Kalman/AppLayer/replayable_config.hpp`

The required baseline-tuning categories are:

- Baro dynamic variance model and innovation clamp.
- Multi-baro voting thresholds, continuity clamps, and recovery policy.
- GNSS trust factors, high-dynamics inflation, and chi2 gates.
- GNSS reject counters and covariance reset thresholds.
- Heading alignment and continuous GNSS COG heading-fusion gates/variance.
- Future magnetometer heading gates if magnetometer hardware is added.
- Liftoff rewind/rejection windows and rail-clear delay.
- Tumble decoupling thresholds.
- Rail/flight shadow gains and aero-blind thresholds.
- Process-noise and bias-random-walk terms.
- Initial covariance terms.
- Multi-IMU voting, hard-fault, saturation, salvage, and recovery thresholds.

Process-noise baseline rule:

- Use Campaign G Allan-deviation results to set initial `accel_noise`,
  `gyro_noise`, `accel_bias_walk`, and `gyro_bias_walk`.
- Keep a direct trace from Allan-fit report to committed defaults used for
  flight and offline replay.

Dynamic-environment correction policy:

- Static Allan terms are baseline only.
- Derive inflation from Campaign H when available.
- If Campaign H is unavailable, apply conservative inflation and refine with
  Campaign J residual analysis.

## 9.2 Tuning loop

For each campaign replay or offline reconstruction:

1. Decode the log to a canonical per-run dataset.
2. For Campaign G, execute Allan analysis and produce process-noise
   recommendations.
3. For Campaign H and J, compute vibration-driven inflation and residual
   consistency metrics.
4. Build candidate tuning and calibration artifacts.
5. Run the available offline replay/simulation path with candidates.
6. Evaluate GPS reject rates, NIS behavior, divergence warnings, baro correction
   behavior, heading alignment timing, and sensor-voting behavior.
7. Accept, revise, and repeat.

## 9.3 Acceptance gates before flight profile update

Minimum pass criteria:

- NIS consistency: for each enabled aiding channel, NIS stays below the
  configured chi2 gate for at least 95 percent of accepted updates outside
  explicitly marked transonic or aero-blind windows.
- Divergence: zero transitions into diverged mode in nominal replay scenarios;
  at most one transient divergence event in stress scenarios, with recovery in
  less than 1.0 s.
- GNSS rejection bursts: no continuous rejection burst longer than 2.0 s during
  ascent/coast, and no more than three bursts longer than 0.5 s per
  flight-equivalent run.
- Heading alignment by GNSS COG: heading error converges within `+/-10 deg`
  within 3.0 s after alignment gates become true, and remains within
  `+/-15 deg` during non-tumbling segments.
- Baro reacquisition transient: no altitude correction jump larger than 15 m and
  no induced vertical-velocity spike larger than 25 m/s lasting more than 0.3 s.
- Multi-IMU health: no unexplained persistent disagreement between healthy IMUs
  after calibration.
- Multi-baro health: no unexplained persistent pressure split between healthy
  BMP390s after static correction.

## 9.4 Integrated flight test protocol

Mandatory data to log from an integrated flight:

- Full-rate raw/SI IMU streams for every populated IMU.
- BMP390 pressure/temperature samples for every populated barometer.
- GNSS raw/fix streams and timestamp provenance.
- Future magnetometer stream only if hardware is installed.
- PPS timestamps if present.
- Liftoff event timestamp and deployment/event markers.
- ESKF state/event streams, shadow-filter outputs, apogee-consensus inputs, and
  rejection/divergence diagnostics.
- Startup snapshots of compile-time calibration and tuning defaults when
  available.

Required post-flight processing:

- Decode to canonical dataset and run available replay/reconstruction with the
  same firmware configuration.
- Estimate dynamic-noise inflation from ascent and high-vibration segments.
- Re-estimate `gps_delay_us` consistency and GNSS rejection behavior.
- Validate baro dynamic-R and transonic penalty behavior against measured
  trajectory smoothness.
- Generate a constrained update proposal using only parameters supported by
  direct evidence.

Safety policy for single-flight overfitting:

- Do not retune all parameters from one flight.
- Prioritize timing, dynamic noise inflation, and gross gate misconfiguration.
- Keep pre-flight campaign datasets as hold-out checks.

## 10. Program/Tooling Specification

This section defines required programs and data flow.

## 10.1 Program A: Firmware logging only

Responsibilities:

- Acquire sensors at production-like settings.
- Log raw/SI streams and critical configuration snapshots.
- Do not run calibration fitting onboard.

Required logged stream coverage:

- IMU samples for every populated IMU.
- BMP390 barometer samples for every populated barometer.
- GNSS fix/raw streams and timestamp provenance.
- Future magnetometer samples only if hardware is installed.
- ESKF state/event streams for tuning diagnostics.
- Calibration/tuning startup snapshots where supported.

## 10.2 Program B: Decoder and canonical dataset builder

Responsibilities:

- Convert the flight-computer log format to a canonical per-run dataset
  usable by calibration solvers and replay/simulation.

Decoder interface contract:

- Numeric streams used by downstream tools should prefer SI units for physical
  values: `m/s^2`, `rad/s`, `Pa`, `K`, `m`, `m/s`, and seconds or microseconds
  as declared.
- Raw or device-native fields may be retained when explicitly named and
  documented.
- Program C must not infer units from value ranges.
- Preserve one unified monotonic vehicle time base in microseconds.
- Preserve original timestamp fields and validity flags for dropped, reordered,
  stale, or fallback-timestamped packets.
- Clock roll-over or wrap handling must be explicit and deterministic in
  decoder metadata.

Critical implementation note:

- The decoder must not perform heuristic cross-sensor time warping to make
  streams line up. Timing correction belongs in acquisition timestamps and
  estimator delay models.

## 10.3 Program C: Calibration solvers

Responsibilities:

- Compute sensor calibration outputs from canonical datasets.
- Emit machine-readable calibration artifacts matching the current
  `CalibrationConfig` schema or a documented successor schema.

Required solver modules:

- IMU thermal fit solver.
- IMU constrained ellipsoid solver.
- Gyro bias/scale solver.
- Allan-variance solver for IMU stochastic terms.
- Baro static bias/scale and optional Cp estimator.
- GNSS lag/lever-arm estimator.
- Future magnetometer ellipsoid solver if magnetometer hardware is installed.

Required artifacts:

- Calibration artifact for hardware calibration fields.
- Noise-model artifact for Allan-derived stochastic recommendations, with fit
  quality metrics and dataset IDs.
- Human-readable report with residuals and acceptance status.

## 10.4 Program D: Replay and tuning optimizer

Responsibilities:

- Run the available native/offline replay or reconstruction path with candidate
  tuning and calibration artifacts.
- Score runs against acceptance metrics.
- Emit tuned config candidates and score report.

Current policy:

- Replay artifacts are qualification tools unless and until flight firmware has
  a validated loadable-config path.
- Flight firmware remains controlled by committed C++ defaults and calibration
  data.

## 10.5 Program E: Report generator

Responsibilities:

- Produce one calibration/tuning report per release candidate:
  - dataset IDs used
  - fitted parameter values
  - tuning parameter values
  - replay/reconstruction metrics
  - pass/fail gates
  - known limitations and firmware gaps

## 10.6 Parallel execution requirement

Given a canonical dataset folder, Program C modules and Program D analysis jobs
must run independently and in parallel where input dependencies allow.

## 11. Integration Targets

For flight firmware build:

- Sensor calibration values in
  `Application/Kalman/AppLayer/hw_calibration_data.hpp` or its successor.
- Tuning defaults in `Application/Kalman/kalman/eskf_tuning_defaults.hpp`,
  `Application/Kalman/kalman/eskf_types.hpp`, and relevant app-layer config
  defaults.

For offline validation:

- Calibration artifact matching `CalibrationConfig` or successor schema.
- Tuning artifact matching `ReplayableConfig` or successor schema.
- Canonical decoded dataset with explicit units, timestamps, source indices, and
  validity metadata.

## 12. Firmware Backlog And Known Gaps

Known gaps to resolve before final calibration lock:

- Confirm final active IMU count and ensure app wiring, calibration schema, and
  Kalman maximums agree for up to 4 IMUs.
- Confirm final BMP390 wiring, bus ownership, command rate, OSR/IIR tuple, and
  timestamp model on flight hardware.
- Finalize SD-card log formats and decoder contract.
- Finalize replay/native reconstruction entrypoint and config override policy.
- Decide whether flight-loadable config support is required or whether committed
  C++ defaults remain the only flight authority.
- Keep magnetometer support documented but disabled unless a validated
  magnetometer is actually installed.
- Extend runtime/schema beyond diagonal `gyro_scale[3]` if cross-axis gyro
  coupling is non-negligible.

Calibration deliverable policy until those gaps are closed:

- Offline calibration must already produce datum geometry, CG table, mounting
  transforms, and Allan-derived stochastic terms.
- Offline calibration must also produce vibration-inflation recommendations and
  g-sensitivity characterization status.
- When runtime cannot yet ingest a field, the release report must include the
  exact projection/approximation used and the associated residual risk.

## 13. Superseded Guidance

This document supersedes conflicting calibration/tuning instructions for the project in older planning documents.

Those documents may still contain useful background theory, but this document is
the authoritative process and data-contract reference for calibration and Kalman
tuning.
