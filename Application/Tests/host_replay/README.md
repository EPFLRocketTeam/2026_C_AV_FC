# Host replay harness for the Kalman runtime

Runs the firmware Kalman runtime on a PC against recorded sensor data, without a board.

**What is real:** `Application/Kalman/**` (including `kalman_process.cpp`, `kalman_lifecycle.cpp` and the whole estimator) and `Application/Data/**`, compiled unchanged from the source tree.

**What is replaced:**
- `inc/` shadows the HAL, `main.h`, the IMU/baro module headers and the InvIMU driver header.
- `stubs.cpp` provides a simulated microsecond clock, the application ring buffers, IMU health and default `FlightParams`.
- `run.cpp` feeds the recorded samples into the rings on 500 µs main-loop ticks, calls `kalman_loop()`, then runs a scripted FSM. The FSM makes the same lifecycle calls as `AvState::update()` (`kalman_on_state_change()`, and `kalman_on_liftoff()` on BURN).

The folder is under `Application/Tests`, which the CubeIDE project excludes from the firmware build.

## Build

```sh
./build.sh                       # this checkout -> build/harness
./build.sh <tree> <out_dir>      # another source tree, e.g. an older commit
```

To compare firmware versions, export a commit with `git archive <rev>`. Also extract the `FLIGHT_PARAMS` (with its `2026_C_AV_CONFMAN` submodule) and `PRC_INTRANET` submodules at the commits pinned by that revision, then pass the tree to `build.sh`. For trees without `kalman_request_reset()`, use `EXTRA=-UHARNESS_HAS_RESET`.

## Inputs

The decoded SD CSVs `IMUSample.csv` (`ts_us,sensor_index,accel_xyz,gyro_xyz,temperature,timestamp_us`) and `BaroSample.csv` (`ts_us,sensor_index,pressure_pa,temperature_c,timestamp_us`), cut to a window around liftoff to keep runs fast, e.g.:

```sh
awk -F, -v a=<liftoff_us-240e6> -v b=<liftoff_us+20e6> 'NR>1 && $1>=a && $1<=b' IMUSample.csv > imu.csv
```

At least ~2 minutes of stationary data before liftoff lets the preflight estimators converge.

## Run

```sh
build/harness imu=imu.csv baro=baro.csv out=run.csv [key=value ...]
```

| Key | Meaning |
|---|---|
| `mode=flown` | INIT → BURN at `t_burn` (what the test flight did) |
| `mode=ignition` | INIT → IGNITION at `t_ign`. Then BURN on cable loss (`t_cable`) or `vertical_acc_hold == DID_HOLD`, and ABORT_ON_GROUND on `DID_NOT_HOLD` without cable (FSM spec, no valve bypass) |
| `burn_to_ascent_ms` | BURN → ASCENT delay (default 2000). ASCENT → DESCENT on `apogee_detected` |
| `descent_max_ms` | DESCENT → LANDED once `touchdown_detected` and this long in DESCENT (default 60000, as `Descent.MaxDurationMs`) |
| `detector=0` | never enable the IMU liftoff detector |
| `reset_at`, `reset_at2` | call `kalman_request_reset()` at these times |
| `fake_gps=1` | inject bogus GNSS fixes (5 km away, 50 m/s) at 5 Hz |
| `drop_imu=<i>@<us>` | mark IMU `i` unhealthy from that time |
| `verbose=1` | keep the firmware `app_printf` output |

All times are absolute microseconds on the log timebase. The output CSV holds, at 100 Hz, the FSM state, navigation output, the ESKF and Flight Shadow altitude and vertical speed (positive up), and the event flags. The log holds the FSM transitions and the firmware prints (`[LIFTOFF]`, `[ACC-HOLD]`, `[APOGEE]`, `[KAL]`).

`scenarios.sh imu.csv baro.csv [harness]` runs the standard set below in parallel; `summary.py runs <names...>` prints apogee and peak values.

## Synthetic flight from an engine log

`make_engine_flight.py` turns an engine `Chamber.csv` log (chamber pressure in bar) into IMU/baro CSVs in the same format. It models a vertical 1-D flight:
- **Thrust:** proportional to chamber pressure, peak `--peak-thrust` (default 7.5 kN). Mass 130 kg, decreasing with an Isp of 230 s.
- **Hold-down:** released at `--release-us`.
- **Drag:** Cd 0.5 on a 0.2 m diameter.
- **Recovery:** a parachute at 7 m/s, then landing and lying on the ground.

The sensors use the firmware mounting, with per-IMU biases, noise and engine vibration. The vehicle constants are rough assumptions, so use it to exercise the liftoff/apogee/touchdown logic with a real thrust profile, not to predict the trajectory. Attitude changes under the parachute and at landing are not modelled consistently (accelerometer only), so the ESKF output after apogee is not meaningful in that data.

```sh
python3 make_engine_flight.py Chamber.csv <out_dir> --release-us <Burn state + 50 ms>
build/harness imu=<out_dir>/imu.csv baro=<out_dir>/baro.csv mode=ignition t_ign=<IgnitionPrechill time> ...
```

## Limitations

- Computation takes no simulated time, so the estimator catch-up budget and IMU drops from loop overruns are not exercised. Sensor-rate problems must be tested on the board.
- The FSM is scripted, not `av_state.cpp`. Burn cut-off is a fixed delay.
- No SD logging; telemetry and CAN are absent.

## Results on the ERT test flight

Window from 240 s before to 20 s after liftoff. Motion onset is about 1350.280 s; the barometers give the apogee as 143.5 m AGL at about 1356.42 s. For the IGNITION scenarios, IGNITION is entered so that motion starts 50 ms after the end of ramp-up (Firehorn timing), and the accel-hold window covers onset + 0.15 s to + 0.95 s.

| Scenario | Before the liftoff-epoch fixes | After |
|---|---|---|
| As flown (BURN 66 ms after onset) | ESKF max 42.9 m; apogee from the Shadow fallback at 1355.89 s | ESKF max 142.2 m; apogee 1356.34 s |
| BURN 100 / 200 / 400 ms after onset, no IMU detector | ESKF max 0.5 m; apogee at 1358.17 / 1358.19 / 1354.98 s | ESKF max 142.2–142.3 m; apogee 1356.34 s |
| IGNITION, accel hold, no cable | ABORT_ON_GROUND 1.9 s before motion (old 3 s + 100 ms window) | IMU epoch at 1350.319 s; DID_HOLD (median 53.4 m/s²) → BURN at 1351.241 s; apogee 1356.34 s |
| Same, accel-hold window moved into thrust | ABORT_ON_GROUND during liftoff (wrong axis) | — |
| IGNITION, cable 30 ms after onset | — | apogee 1356.34 s |
| IGNITION, accel hold, no IMU detector (BURN ≈ 0.96 s late) | — | degraded: ESKF max 31.5 m; apogee from the Shadow fallback at 1356.13 s |
| `kalman_request_reset()` 20 s before liftoff | — | accepted; apogee 1356.34 s |
| `kalman_request_reset()` during IGNITION | — | refused |
| Bogus GNSS injected | — | output identical (GNSS not fused) |
| Real parachute descent (log ends at landing impact) | — | no false touchdown |

## Results on the VSFT 1 engine log (synthetic flight)

IGNITION is entered at the engine's IgnitionPrechill (2832.272 s); hold-down release is at Burn + 50 ms (2837.260 s). Model: burnout at 2839.15 s (66 m/s), apogee at 2846.140 s (305.8 m), landing at 2888.65 s.

| Scenario | Before the liftoff-epoch fixes | After |
|---|---|---|
| Accel hold, no cable | ABORT_ON_GROUND at 2835.37 s (igniter phase) | Kalman epoch at 2837.264 s; DID_HOLD (median 52.4 m/s²) → BURN at 2838.216 s; ESKF max 306.1 m @ 2846.140 s; apogee 2846.146 s; touchdown 2893.82 s → LANDED 2906.15 s |
| Cable 30 ms after release | same abort (before the cable) | same as above, BURN at 2837.290 s |
| Accel hold, IMU detector off | — | ESKF degraded (max 123 m); apogee via the Shadow fallback at 2846.142 s |
