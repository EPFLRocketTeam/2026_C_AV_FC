# Kalman flight readiness: changes, evidence and open items

**Date:** 2026-10-07
**Author:** Anas Himmi (Kalman)
**Base:** `origin/main` @ `498a770`
**Follows:** `summaries/2026-10-06-kalman-flight-readiness-review.md` (Codex review; section numbers below refer to it)

None of this has been run on the board yet. Everything builds (CubeIDE Debug, clean build, 0 errors). The estimator changes were validated by offline native replay of recorded flight data through the same estimator code (identical `virtual_imu.cpp`).

## 1. Branches

| Branch | Base | Content | Merge |
|---|---|---|---|
| `feat/rekalman` | `origin/main` | GNSS log-only in the Kalman; `imu_liftoff_detected` cleared on INIT; `kalman_request_reset()` | Kalman-only, low risk |
| `feat/kalman-liftoff-epoch` | `feat/rekalman` | Tare stationarity gate; liftoff detection enabled when the buzzer is not started; Kalman liftoff epoch from IMU detection during IGNITION | Recommended before flight; needs a board smoke test |
| `fix/acc-hold-params` | `feat/kalman-liftoff-epoch` | `vertical_acc_hold` on the body thrust axis with `FlightParams`; bumps `FLIGHT_PARAMS` to `a8e369a` | **Théo to review** (pin bump, threshold semantics) |
| `feat/sd-log-imu-pipeline` | PR #24 (`feat/sd-logger-low-rate`) | `ImuPipeline` gated together with raw IMU (storage and CPU) | Merge into PR #24 |
| `fix/gnss-init-nonfatal` | `feat/rekalman` | GPS init failure no longer stops the super-loop | **Théo to review** |
| `test/flight-readiness-all` | — | Local merge of everything above, for flashing during remote tests only | Do not merge |

## 2. New critical finding: a late liftoff epoch silently breaks the ESKF

The estimator freezes its preflight state at the liftoff epoch it receives from the FSM (BURN entry). Before this work, the per-IMU preflight tare was a tumbling 200-sample (~31 ms) window, **with no stationarity check**, that kept updating until that epoch. Any window completed between motion onset and the epoch was frozen as accelerometer bias, i.e. thrust was subtracted from every subsequent IMU sample.

Offline replay of a recorded high-acceleration flight (~15 g boost, 3 km apogee), with the liftoff epoch forced at motion onset + delay:

| Epoch delay | Before (max alt / apogee event) | With tare gate |
|---|---|---|
| 0 ms | 3049 m / 25.52 s | 3049 m / 25.51 s |
| 50 ms | 3051 m / 25.86 s | 3049 m / 25.51 s |
| 100 ms | **1798 m / never** | 3049 m / 25.52 s |
| 300 ms | **1951 m / never** | 3048 m / 25.52 s |
| 800 ms | **1769 m / never** | 3045 m / 25.42 s (transient up to 57 m/s) |
| 1100 ms | — | 3046 m / 25.52 s (transient up to 104 m/s) |
| 1500 ms | — | ESKF reports descent at 15 s; apogee 25.68 s |

"Never" means the consensus detector refused to fire because the (still correct) Flight Shadow disagreed: safe, but **no apogee event at all**, and with `AscentMaxDurationMs = INF_TIME` no fallback.

Why it matters for Firehorn:

- With the valve bypass removed, BURN comes from the cable or from `vertical_acc_hold`, whose window ends ~0.8 s after the expected thrust (`TotalTimeUntilHoldDownMs` + 800 ms). That is exactly in the broken range.
- The flight-tested dual-window detector **never ran** in the current code: `buzzer.start()` is commented out, so `g_liftoff_detection_allowed` never became true. Committed `cf4d066` has the same problem, yet the flight logged `imu_liftoff_detected = 1`. So the flown binary was built with local changes (probably `buzzer.start()` uncommented). Commit `cf4d066` is therefore only approximately the flown binary.
- In the ERT test flight, motion started ~60–80 ms before BURN. That is at the edge of the tare problem and may explain the 10.7 m ESKF/Flight Shadow disagreement at apogee.

Fixes (`feat/kalman-liftoff-epoch`):

1. **Tare gate.** A tare window is committed only if every sample has | ‖a‖ − g | ≤ 1.5 m/s² and ‖ω‖ ≤ 0.2 rad/s, and the window-mean bias is ≤ 1 m/s². Otherwise the previous tare is kept. On the pad, the specific force stays exactly g until thrust exceeds weight, so static windows are unaffected.
2. **Kalman liftoff epoch from IMU detection.** While the FSM is in IGNITION, the dual-window detector starts the estimator's flight mode immediately, using the timestamp of the detecting sample. The FSM stays the authority on flight states, and its later BURN is ignored by the Kalman. A detection latched before IGNITION (e.g. a bump during FILLING) is discarded on IGNITION entry. If the detector does not fire, BURN still initialises the estimator as before.
3. **Detector enable.** Liftoff detection is enabled 3 s after boot when the buzzer sequence is not started (unchanged when it is).

Measured detector latency on real data, with the firmware `LiftoffDetector` fed with the recorded samples: ERT test flight about 20 ms after strong thrust (41 ms before the flown FSM's BURN); replayed flight about 30–50 ms after onset (clean in the table above).

## 3. Codex items

| # | Item | Status |
|---|---|---|
| 1–2 | Liftoff FSM path / valve bypass | **Théo.** The FSM diagram's `IGNITION → BURN` = cable OR accel hold is kept as the authority. The Kalman no longer depends on its timing (section 2). The bypass removal itself is an FSM change (planned after coldflows). |
| 3 | `vertical_acc_hold` wrong axis and conflicting params | **Fixed in `fix/acc-hold-params`.** It read raw `sensor_x` (≈ 0 on the pad; the thrust axis is `−sensor_z`), so it always returned DID_NOT_HOLD → ABORT_ON_GROUND unless the cable fired first. Now it uses every raw sample of every IMU, rotated by the fixed mounting matrix only (still independent of estimator preprocessing). Samples are selected by timestamp in `[IGNITION + TotalTimeUntilHoldDownMs, + LiftoffAccelDurationMs)`. The decision is the median of per-IMU means vs `LiftoffAccelThreshold`. The `threshold.h` values are no longer used by the Kalman. Correction to Codex: the apogee high-accel lockout was **not** affected (it uses the virtual IMU body accel). Only `navigationData.accel` (DataDump) was raw sensor frame; it is now body frame. |
| 4 | `ColdflowMode` defaults to `true` | **Théo.** Must be observably `false` at arming for flight. |
| 5 | Ascent timeout infinite | **Théo / trajectory.** Needs a finite backup: suggestion (nominal apogee time − burn time) + margin from the Firehorn simulation. `AscentMaxDurationMs` is `FIXED` (reflash) and counted from ASCENT entry. |
| 6 | SepMech not actuated | **Théo / recovery.** Out of Kalman scope. |
| 7 | GNSS fused | **Fixed in `feat/rekalman`.** `KALMAN_GNSS_FUSION_ENABLE` defaults to 0. Fixes are still drained and counted, raw UBX is still logged by the GPS driver, and telemetry still gets GNSS from the GPS module. Separately, `fix/gnss-init-nonfatal`: GPS init failure no longer sets `ready = false` (which stopped the Kalman, FSM and SD), and the GPS is then simply not polled. |
| 8 | Reset | **Added `kalman_request_reset()`** (`kalman_process.h`). It only sets a flag (safe from any context) and runs at the start of the next `kalman_loop()`. Only in INIT, CALIBRATION, FILLING, ARMED, ABORT_ON_GROUND, LANDED. It resets the estimator, tares, shadows, ground references, turn-on bias, GNSS anchor, apogee hub, both liftoff detectors, health store, liftoff latch and the Kalman-owned EventStore flags. **To wire** to a CLI/uplink command (the shell commands are generated in the CLI submodule). Reconvergence needs tens of seconds to ~2 minutes stationary. **Not covered (FSM, Théo):** `AvState::has_lifted_off_` and the flight timer are never cleared on INIT. |
| 9 | `ImuPipeline` not gated | **Fixed in `feat/sd-log-imu-pipeline`** (on PR #24). Gated with raw IMU in `SdLogger`, and the estimator skips building the snapshot when the logger does not want it. These two streams were ~3 GB each of the ~6.2 GB decoded from the test flight. Everything else together was ~120 MB per 22 min and stays always on. Note PR #24 enables bulk logging on ARMED and only disables it on INIT, so it stays on during an ABORT_ON_GROUND hold until RECOVER. That keeps the post-abort data, but a long hold fills the card at full rate (~1 h on 8 GB); turning it off after N minutes in ABORT_ON_GROUND is an option. |
| 10 | IMU rate | Needs the board. Théo is improving the LoRa driver. Acceptance should be per-sensor gaps, not aggregate rate. |
| 11 | Hardware calibration placeholders | **Accepted.** Simulations show negligible impact on apogee detection. A calibration protocol and scripts exist (ask Anas) if time allows. |
| 6.3 | Dual-window detector timing depends on the IMU count | Kept as flown (effective windows 2.5/7.5 ms and 0.5 s arming with 4 IMUs). With fewer IMUs it only gets slower (max 10/30 ms). |

## 4. Questions for other members

1. **Firehorn thrust-to-weight during the first ~100 ms of motion?** The detector needs > 2 g excess (fast) and > 1 g (slow). The acc-hold needs a mean specific force > 15 m/s², i.e. net acceleration > 0.53 g. If the net acceleration is below 2 g, the detector never fires and the Kalman falls back to the FSM epoch (the tare gate keeps that degraded but working).
2. **Théo:** is `LiftoffAccelThreshold = 15` meant as specific force (accelerometer reading, ~9.81 on the pad), as implemented? (The old value 2 only made sense as gravity-free acceleration.)
3. **Théo:** is it OK to bump the `FLIGHT_PARAMS` pin to `a8e369a`? It also changes Pedro's DYNAMIC defaults. If the PRC boards compare config CRCs, they need the same pin.
4. **Whoever flashed the test flight:** was `buzzer.start()` uncommented, or were other local changes made? Please archive the exact source and build flags of the final flight binary (also check `FAKE_GNSS_ENABLE` and `KALMAN_DEBUG_FORCE_FLIGHT`).
5. **Théo:** the accel-hold verdict is purely time-scheduled. If the engine lights later than `TotalTimeUntilHoldDownMs` predicts, the window can close with little acceleration while the cable is still connected. That gives DID_NOT_HOLD, then ABORT_ON_GROUND and a broadcast abort, possibly just as the rocket lifts off. Is that the intended behaviour? (On the Kalman side, a self-latch followed by an abort is harmless: it integrates static data until RECOVER → INIT or `kalman_request_reset()`.)

## 5. Board checks (remote access)

- Boot: `[LIFTOFF] Detection enabled (3000ms after buzzer)` then `[LIFTOFF] Detector ARMED`, and no spurious `DETECTED` while static.
- Mounting matrix still valid for Firehorn: static on the rail, body +X (`navigationData.accel.x` on `fix/acc-hold-params`) reads ≈ +9.81 m/s². Both the estimator and the accel hold depend on it.
- Static run for 10 minutes or more: no `[LIFTOFF]` detection, apogee flag stays 0, `[SD]` and IMU rates are unchanged versus `main`.
- `kalman_request_reset()` once wired: `[KAL] Reset done`, and `Reset refused` in IGNITION or later.
- With the bulk logging gate: arena use before ARMED should drop sharply compared to PR #24 alone.
- Reading the SD without removing it: the board's USB already enumerates on the Raspberry Pi (CDC). A small test firmware using the STM32 USB Device MSC class over SDMMC would expose the card as a raw block device to the Pi, which can then `dd` it for offline Plume decoding. MTP is not needed.
