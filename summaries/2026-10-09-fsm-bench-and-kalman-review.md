# FSM bench walk-through, SD card stalls and Kalman review fixes — 2026-10-09

Follow-up to `2026-10-09-remote-fixes-results.md`, on the same remote bench:
- 4 IMUs, 2 barometers (indices 0 and 3), the telemetry board with both LoRa radios, the GPS;
- no ground station, PRCs not connected;
- the board lies on its side, so body X is horizontal.

Firmware: this branch, which also contains `fix/eskf-audit` and `fix/sd-raw-imu-headroom`. Captures and images are in the ignored `logs/bench/` (`fsm_bypass*`, `fsm_nobypass*`, `sd_headroom*`).

## FSM walk-through (real FSM, driven from the shell)

The tank pressures come from the PRCs and read 0, so PRESSURIZATION → IGNITION needs the nominal bounds widened. The shell writes go to a staging buffer and only apply after `config commit`:

```
calibrate
arm
config pressurize min_lox_nominal_pressure -1.0
config pressurize min_fuel_nominal_pressure -1.0
config pressurize hold_delay 5000
config commit
pressurize on
abort          # only with the bench bypass
recover
```

| Run | Sequence observed |
|---|---|
| Current code (IGNITION bench bypass present) | INIT → FILLING → ARMED → PRESSURIZATION → IGNITION (5 s hold) → acc-hold DID_NOT_HOLD (median −0.22 m/s²) at +5.9 s → stays in IGNITION, waiting for MO/ME → manual abort → ABORT_ON_GROUND → recover → INIT |
| Bypass removed (local build) | Same up to IGNITION → DID_NOT_HOLD at +5.9 s → IGNITION → ABORT_ON_GROUND automatically (cable reads connected) → recover → INIT |

- **Liftoff detection:** the IMU liftoff detector stayed armed and quiet, with no false liftoff or apogee in any run.
- **Logging switches:** full-rate raw logging started at ARMED and stopped at INIT.
- **Kalman reset on RECOVER → INIT:** consumed samples restart from 0 and keep advancing, ring high-water marks rebuild, and drops stay 0.
- **IMUs:** all 4 at 6362–6404 frames/s with zero hardware or application loss in every run.
- **Loop and radio:** worst loop 16.8 ms, never above 64 ms; radio TX 295/295 with no timeout.

Open point: the bench bypass in `fromIgnition` makes the automatic abort unreachable. It has to be removed (or kept deliberately) before flight.

## SD card stalls during full-rate logging

With full-rate raw logging on, this card regularly takes ~165–175 ms for one write, about once a minute. Normal writes take ~9 ms. At ~0.8 MB/s of raw IMU data, the 258 KB arena holds ~0.3 s, so each stall fills it:
- **Before `fix/sd-raw-imu-headroom`:** every record type failed during the stall. One stall lost 1,757 records and 5,849 raw IMU batches, and FSM transitions or the liftoff snapshot could have been among them.
- **After:** raw IMU and ImuPipeline writes stop at 3/4 arena occupancy. Over 4 min ARMED, with two stalls, 0 records failed. ~7,500 raw batches (~2 s of raw IMU data) were shed per stall, because the card only just keeps up after a stall.

The card's throughput margin at full raw rate is small. Options: lighter raw logging before liftoff, a faster card, or a pause-tolerant layout.

## Review of the shared Kalman code

An external review of code shared with this Kalman found these issues. Each was checked here:

| Finding | Here | Action |
|---|---|---|
| NIS re-counted on every IMU predict: one high-NIS baro update declares divergence ~10 samples later, and `predict()` then freezes on soft divergence | Present | Fixed: NIS only counted after measurement updates; soft divergence keeps propagating. Two regression tests in `test_eskf_core.cpp` fail before the fix and pass after. |
| Coast phase decided by one raw body-X sample | Present: it flips sign hundreds of times per second in early coast, and near apogee a ~0.1 m/s² bias or a tail-first slide can block every apogee decision | Fixed: thrust-off latch, filtered body-X specific force < 1 m/s² for 100 ms. Test-flight replay unchanged. VSFT synthetic flight unchanged (apogee 4 ms after truth); the degraded detector-off variant now follows the detector's near-zero rule, 0.2 s before apogee. |
| Rewind keeps checkpoints newer than the restore point, so a later rewind can drop an earlier GNSS correction | Present, latent (GNSS not fused in flight) | Fixed: newer checkpoints dropped on restore. With GNSS fusion forced on, the replay output changes by up to 0.76 m, as expected. |
| `last_innovation_` never set, so every correction logs 0 | Present | Fixed for the scalar corrections. |
| Preflight tare picks up thrust | Already fixed (stationarity gate) | — |
| Stack overflow in `processSyncedImuGroup` | Not applicable: 11.5 KB frame, 128 KB DTCM stack, worst chain ~30 KB | — |
| Samples stamped at FIFO drain time | Not applicable: each sample keeps its own FIFO timestamp | — |
| IMU liftoff monitor moves the FSM | Not applicable: only the Kalman epoch uses it | — |
| Others (boot wait on USB, link parser, watchdog, liftoff jack, PPS stamping, magnetometer, threads) | Platform-specific or not used here | — |

The existing `test_gps_rewind_full.cpp` / `test_gps_rewind_parity.cpp` have three failures before and after these changes. They are stale tests (late-baro innovation transport, out-of-order GPS contract, baro-pending contract), not regressions.

## SD stalls: cause and fix (branch `feat/sd-throughput`)

The ~170 ms writes are not new. Codex's own normal soaks (`N_normal`, `A_soak`) have them too. Without full-rate logging they only use ~30 KB of arena, so no counter moved.

What changed is the size of each SD write. Plume writes everything that is buffered (up to 64 blocks) on each tick.
- **Old firmware:** in the test flight, the loop took 3.3 ms on average and logging ran at 1.15 MB/s. Writes were large, and over 23 min the only failures were at boot.
- **Now:** the cache and `-O3` work cut the loop to ~0.8 ms, so the card receives ~350 writes/s of 1–8 blocks. With that pattern it stalls about once a minute.
- **Théo's 7.5 GB fill:** it watched `write_fail_count`, which did not include raw-batch failures before Oct 8.

Fixes:
- **Packed raw IMU:** `SD_LOG_IMU_RAW_PACKED` stores the sensor's 20-bit counts, ~22 B/sample instead of 42. It is lossless (bit-exact rebuild checked, with fallback to the full record), and the decoder writes the same `IMUSample.csv` rows.
- **Larger writes:** `SDCardInterface::tick()` starts a write only once 32 new blocks are buffered or 20 ms have passed.

Each variant: 541 s ARMED with full-rate logging, same board and card.

| Variant | Total rate | Blocks/write | Writes > 100 ms | Raw batches shed | Loop avg |
|---|---|---|---|---|---|
| Before (headroom only, 260 s) | 1.13 MB/s | 7.3 | 5 | 15,059 | 861 µs |
| Packed only | 0.75 MB/s | 4.0 | 2 | 8,828 | 907 µs |
| Larger writes only | 1.27 MB/s | 32.6 | 2 | 7,351 | 836 µs |
| Both | 0.76 MB/s | 29.7 | 0 (worst 55 ms) | 0 | 880 µs |

The first "both" run, with slower packing, gave the same result (0 stalls, 0 shed in 541 s). Over the two runs: 0 stalls in 18 min, where the other variants averaged one every ~4.5 min. Record failures were 0 everywhere, and IMU acquisition loss was 0.
