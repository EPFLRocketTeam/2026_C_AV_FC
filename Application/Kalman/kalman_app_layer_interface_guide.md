# Kalman App-Layer Interface Guide (ERT)

Date: 2026-04-14
Audience: Firehorn II firmware team (non-Kalman and Kalman contributors)

## 1. Purpose

This document explains how the Kalman pipeline is currently integrated with the ERT app layer:
- how sensor data reaches Kalman
- how Kalman outputs are published back to app stores
- how apogee input/decision wiring works
- how this task behaves versus other RTOS tasks
- what remains to be completed by the team

This is implementation-grounded and reflects the current code state.

## 2. High-Level Runtime Topology

Subsystem label: RTOS task topology
- Producer side: defaultTask executes mainLoop, which ticks IMU drivers and pushes IMU frames into shared ring buffers.
- Consumer side: kalmanTask waits on thread flags, then runs kalman_loop.
- Synchronization: IMU ring buffers are protected by per-buffer mutexes on write and read.

Compared to other RTOS processes
- This Kalman flow is hybrid wake-driven: IMU production, lifecycle events, and a 20 ms periodic RTOS timer all wake kalmanTask.
- It is not a fixed-period vTaskDelay loop.
- It drains all available IMU data in one wake cycle, then performs aiding ingestion and onTick processing.
- It is stateful across calls (internal runtime singleton), unlike short stateless handlers.

## 3. Data Circulation: Sensors -> Kalman -> Stores

Subsystem label: IMU path
1. InvIMU drivers produce IMUData frames in mainLoop.
2. ImuModule appends IMUData frames to ring buffers (imuData1/2/3) under mutex.
3. kalmanTask drains ring buffers under mutex and converts each IMUData to estimator ImuSample/ImuBatch format.
4. AppLayer EskfEstimator receives processImuBatch calls.
5. Estimator internal pipeline performs VirtualImu preprocessing + ESKF ingestion + catchUp in onTick.

Subsystem label: Baro aiding path
1. kalman_loop reads current GOATStore snapshot.
2. navigationData.baro is converted to BaroSample.
3. Estimator onBaroTrigger and processBaroSample are called (trigger/complete style preserved).
4. Simple dedup guards prevent excessive repeated same-value fusion each loop.

Subsystem label: GNSS aiding path
1. kalman_loop reads gps_state from GOATStore snapshot.
2. If validity/fix checks pass, it maps fields into GnssSample.
3. Estimator processGpsSample is called.
4. Dedup guards prevent repeated unchanged fixes at high loop rates.

Subsystem label: Lifecycle events
- AvState transition logic calls:
  - kalman_on_state_change(state)
  - kalman_on_liftoff(ts)
- Lifecycle hooks also set the Kalman task wake flag so state/liftoff events are consumed even if IMU is temporarily silent.
- kalman_loop drains IMU and aiding first, then consumes pending liftoff, then runs onTick.
- State INIT transition performs a hard runtime reset (`estimator.reset()` + `apogee_hub.reset()`) so each mission starts from a clean estimator lifecycle.
- Lifecycle INIT entry also clears sticky EventStore flags (`catastrophic_failure`, `apogee_detected`) before the next mission sequence.
- kalman_loop consumes lifecycle updates atomically and applies:
  - estimator.onFlightStateChange(...)
  - estimator.onLiftoff(...)

## 4. Data Circulation: Kalman -> App Layer

Subsystem label: Navigation output mapping
Each kalman onTick cycle publishes estimator output into GOATStore.navigationDataStore:
- position_kalman <- estimator position_ned
- speed <- estimator velocity_ned
- altitude <- estimator altitude_m
- attitude <- quaternion converted to roll/pitch/yaw
- course <- yaw heading from quaternion (explicitly not ground-track)
- accel <- latest measured body acceleration from IMU ingest path

Subsystem label: Apogee interface contract
- Runtime builds ApogeeInput from estimator output and estimator-derived coast classification (`EskfEstimator::isCoastPhase()`, parity with ktp-soft).
- Runtime evaluates ApogeeHub (primary: consensus detector).
- When triggered, event.apogee_detected is set in GOATStore.eventStore.
- EventStore apogee/catastrophic flags are synchronized through a dedicated mutex because FSM reads and Kalman writes occur on different tasks.
- navigationData publish/read now uses a dedicated mutex as well to avoid torn cross-task snapshots.

Subsystem label: Health telemetry contract
- Runtime updates KalmanHealthStore each cycle with:
  - diverged flag
  - altitude_valid flag
  - velocity_valid flag
  - imu_samples_consumed counter
  - baro_updates counter
  - gps_updates counter
- Runtime also sets event.catastrophic_failure if ESKF divergence is detected.
- Flight FSM now consumes catastrophic_failure in IGNITION/BURN/ASCENT/DESCENT and transitions to ABORT_IN_FLIGHT.
- Preflight states intentionally do not consume catastrophic_failure; the ignition gate enforces the abort if the flag is still present.
- Wake counters (`wake_imu/lifecycle/timer/backlog`) are cumulative since the last actual transition to INIT.

## 5. Logging and Observability

Subsystem label: Internal Kalman logging
- EskfEstimator supports log sink injection (hal::ILogSink), currently runtime uses default/null sink path.
- Rewind/catchUp diagnostics remain available through estimator/filter diagnostics APIs.

Subsystem label: App-visible observability
- Navigation fields are continuously published in GOATStore.navigationDataStore.
- Apogee decision latch is published to GOATStore.eventStore.apogee_detected.
- Health snapshot available from KalmanHealthStore singleton.

## 6. Interface Contracts (Current)

Subsystem label: Input contracts
- IMU input contract:
  - source-tagged batches
  - monotonic timestamps (us)
  - conversion path from SI IMUData to estimator raw-like format
- Timebase contract:
  - Shared app monotonic clock drives IMU timestamps, GNSS timestamps, av_timestamp, apogee timing, and EskfYieldable nowMicros.
  - DWT-backed microsecond timing is used on STM32 when available, with HAL tick fallback.
  - `use_dwt_timestamps=false` in IMU config is debug-only fallback and bypasses the unified timebase path.
- Baro input contract:
  - pressure Pa + temperature C + timestamp us
  - trigger/complete sequence at runtime level
- GNSS input contract:
  - valid fix gate, basic position/accuracy mapping from GpsBasicFixData
  - derived timestamp us/itow_ms from runtime now

Subsystem label: Output contracts
- NavigationDataStore fields are the primary app-facing Kalman outputs.
- EventStore.apogee_detected is the apogee boolean contract exposed to higher-level logic.
- KalmanHealthStore is the technical health contract for telemetry/debug.

## 7. What Is Left for Team Members (Action Checklist)

Subsystem label: Producer ownership outside Kalman team
- [ ] Baro producer ownership: ensure GOATStore.navigationDataStore.baro is fed with valid, timestamp-consistent data semantics at runtime.
- [ ] GNSS producer ownership: ensure GOATStore.gpsStore updates are consistent, timely, and include reliable valid/fix flags.
- [x] Timebase ownership: producer timestamps and av_timestamp are aligned on the shared monotonic app timebase.

Subsystem label: App/RTOS integration ownership
- [x] Raised kalmanTask stack budget to 8 KB baseline; still validate high-water on hardware load tests.
- [x] Ensure kalmanTask is not starved when IMU is silent by waking on lifecycle events and periodic timer.
- [x] Keep MX_USB_DEVICE_Init single-owned from defaultTask startup (single post-kernel call).
- [ ] Add one hardware validation pass with `-fstack-usage` artifacts plus FreeRTOS stack overflow check (`configCHECK_FOR_STACK_OVERFLOW=2`).

Subsystem label: Flight-logic ownership
- [ ] Consume event.apogee_detected in flight-state/actions path (parachute/deployment logic).
- [x] Catastrophic policy wired: catastrophic_failure forces ABORT_IN_FLIGHT from in-flight states.

Subsystem label: Telemetry/logging ownership
- [ ] Surface KalmanHealthStore snapshot in telemetry/log packets.
- [ ] Decide and wire persistent logging sink for estimator diagnostics if required for flight qualification.

Subsystem label: Build/release ownership
- [ ] Close deferred STM32 headless/Cube gate verification in CI or documented release process.

## 8. Validation State and Required Remaining Validation

Current validation state
- CMake baseline: 214/214 tests passing.
- Includes parity ports for:
  - orchestrator-adjacent estimator/apogee contracts
  - descent filter direct tests
  - virtual imu/baro and shadow direct subsets
  - gps rewind/baro innovation transport subset
  - output bridge and health store tests

Required remaining validation before flight-readiness signoff
- TR-P7.1: long-run load test with no queue overflows and stable counters.
- TR-P7.2: fault-injection scenarios for divergence/fallback and deterministic system behavior.
- TR-P7.3: integrated rehearsal where flight logic consumes apogee_detected and catastrophic_failure contracts end-to-end.

Practical risk consequence if remaining work is skipped
- Kalman internals can be correct while deployment/control logic still behaves incorrectly due incomplete store consumption, telemetry visibility gaps, or RTOS/resource issues.

## 9. Quick Reference: Key Integration Files

- Task scheduling and thread handoff:
  - Core/Src/main.c
  - Application/main.cpp
- IMU producer module:
  - Application/Modules/imu_modlue.hpp
- Kalman runtime loop and app-layer wiring:
  - Application/Kalman/kalman_process.cpp
  - Application/Kalman/kalman_lifecycle.cpp
- App-layer estimator and contracts:
  - Application/Kalman/AppLayer/eskf_estimator.hpp
  - Application/Kalman/AppLayer/eskf_estimator.cpp
  - Application/Kalman/AppLayer/state_estimator.hpp
  - Application/Kalman/AppLayer/apogee_hub.hpp
  - Application/Kalman/AppLayer/consensus_apogee.cpp
  - Application/Kalman/AppLayer/output_bridge.cpp
- Health telemetry contract:
  - Application/Kalman/kalman_health.hpp

## 10. Handoff Note

If non-Kalman team members modify producer semantics (IMU/baro/GNSS rates, validity rules, timestamping), Kalman contract tests must be rerun and updated in lockstep.
