# Remote FC bench results — 2026-10-08

## Decision

The non-blocking radio change passes the local bench checks. The combined firmware is **not flight-ready**: raw acquisition can be healthy while the estimator's aligned input consumer stalls. Immediate bulk logging also overloads startup. These are separate from the radio improvement; a low `kal` execution time can mean starvation, not successful processing.

This session executed the telemetry A/B/C/D/E plan, safe two-radio downlink/uplink checks, bulk-logging comparisons, reset, host regressions, and a normal-build stationary soak. No arming, launch, valve, pyro, or recovery commands were sent. The real FSM remained INIT during every physical test, including the forced-estimator-flight variant.

## Setup and reproducibility

- STM32H743 FC remotely accessed through the Raspberry Pi, ST-Link and USB CDC. Four IMUs, two populated barometers (indices 0 and 3), SD card, and a telemetry board with two LoRa radios. Unpopulated barometers 1 and 2 returning `0xFF` are not failures.
- No external ground station. The two radios were used for loopback. GPS NAV-PVT traffic appeared during the session, approximately 16 messages/s with no UART overruns; position/fix quality was not validated.
- All successful flashes were verified by OpenOCD at approximately 3.21 V. An initial 0.016 V attach failure cleared after retry; the user also arranged a board reboot during initial IMU diagnosis.
- Local combined branch: `test/remote-bench`. It includes the flight-readiness stack, non-blocking telemetry, GPS DMA and the raw-IMU/pipeline logging gate. This is a test integration branch, not a production merge recommendation. Nothing was pushed.
- Separate review branch: `fix/sensor-bringup` at `e343ed9`, based on `feat/rekalman`. It isolates the sensor fixes from the combined bench harness. The combined branch was physically tested; the isolated branch was not independently flashed.
- `make build RUN=name BUILD_FLAGS='-D SYMBOL=1'` archives the image, flags, source/diff and SHA-256. `make deploy RUN=name FIRMWARE=logs/bench/name.bin CAPTURE_SECS=145` flashes that immutable image and captures serial output. Failed flash/capture commands are fatal; only the intended capture timeout is accepted. Long captures require `make usb-keepalive` because the Pi disables USB after 600 seconds.
- Firmware builds: 0 errors, 75 existing warnings. Source changes and report are local commits. Logs/binaries/build metadata are retained in ignored `logs/bench/`; they are not included in Git.
- `Application/Tests/bench/perf_summary.py` excludes the first 15 reports by default and normalizes frame counts using actual firmware elapsed time. E uses `--skip 20`; delayed logging uses `--skip 70`. Ground-mode `behind` is not a meaningful estimator latency because the ESKF is hibernating; queue drops alone are not a ground-mode acceptance failure. Freezing `consumed` and stale/zero body acceleration are additional evidence of consumer failure.

## Changes kept

1. SPI4/SPI5 were running at 32 MHz (64 MHz kernel divided by 2). They now use a divider of 8, giving 8 MHz for the shared IMU/barometer buses. The [BMP390 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390-ds002.pdf) specifies a 10 MHz maximum and the [ICM-45686 datasheet](https://invensense.tdk.com/wp-content/uploads/documentation/DS-000577_ICM-45686.pdf) specifies 24 MHz. Both source initialization and `.ioc` were updated.
2. IMU configuration verifies `PWR_MGMT0=0x0F` and retries the write before proceeding to MREG setup. After this and the bus correction, all four sensors streamed on repeated flashes. Slowing SPI alone had initially restored only two.
3. BMP390 timing now uses the common monotonic `app_timebase_now_us`, rather than resetting/maintaining a separate DWT clock. Before this fix, the populated barometers stopped refreshing after roughly 18–25 seconds.
4. Barometer triggers return an acceptance boolean. The acquisition state machine enters its pending-conversion state only when the trigger was accepted; a rate-limited rejected trigger no longer causes a false 50 ms pending wait. Mock support and a regression test cover rejection.
5. Raw IMU and `ImuPipeline` logging are gated together (off in INIT, on at ARMED). Safe bench switches separately exercise flight CPU load, bulk logging, delayed bulk logging, radio loopback and reset. Every switch defaults off.
6. Binary radio payload logging prints ID/length instead of treating the payload as a `%s` string. This avoids reading beyond a received binary payload.
7. Read-only diagnostics distinguish hardware FIFO loss, driver timestamp age, application ring backlog, estimator consumption, barometer conversions and SD failures. Cumulative IMU write counters allow rates to be checked across USB-log silence.

An experimental FIFO timestamp-anchor correction was evaluated in `E_anchor_20261008_224707.log`, failed to fix the problem, and was removed. It is **not** part of the retained sensor fixes.

## Physical test matrix

Steady-state maxima below are in microseconds. Rates are per IMU, not the misleading aggregate rate. Zero loss refers to hardware sample-gap counters, not estimator ingestion.

| Variant / capture | IMU frames/s range | Hardware loss | Radio max | Kalman max | SD write failures | Result |
|---|---:|---:|---:|---:|---:|---|
| A normal, `A_soak_20261008_231301.log` | 6362–6404 | 0 steady-state | 275 | 2,661 | 0 | Sensor/radio/SD/static-event pass; consumer intermittent |
| B old blocking TX, `B_final_20261008_230043.log` | 6068–6129 | 145,658 summed report losses | 92,072 | 254 | 0 | Confirms blocking radio loses samples |
| C radio loop disabled, `C_final_20261008_223803.log` | 6363–6404 | 0 | 0 | 3,682 | 0 | Acquisition/reference pass |
| D radios held reset, `D_final_20261008_224056.log` | 6362–6403 | 0 | 43 | 3,997 | 0 | Absent-radio behavior pass |
| E flight estimator only, `E_cpu_20261008_225224.log` | 6362–6404 | 0 | 241 | 3,065 | 0 | CPU path runs, estimator timeliness fails |
| L downlink loopback, `L_loopback_20261008_225534.log` | 6363–6404 | 0 | 262 | 3,760 | 0 | 130 sent / 130 decoded steady-state |
| U safe uplink loopback, `U_uplink_20261008_230643.log` | 6362–6403 | 0 | 435 | 243 | 0 | 129 sent / 129 decoded steady-state |
| F immediate bulk, `F_bulk_diag_20261008_225809.log` | 6288–6301 | 43,355 summed report losses | 258 | 255 | 6,201 | Startup overload and subsequent starvation |
| F delayed bulk, `F_delayed_20261008_230918.log` | 6362–6403 | 0 | 245 | 1,210 | 0 | SD/acquisition pass; consumer still intermittent |
| G reset at 60 s, `G_reset_20261008_230318.log` | 6363–6404 | 0 | 245 | 1,197 | 0 | API accepts reset; functional recovery fails |

B has one >64 ms loop stall per second (130 steady reports), a 95,008 us worst loop, and a FIFO high-water mark of 398 frames. Its cumulative-loss deltas over 129.119 s total 144,537 samples, about 1,119 samples/s across all four IMUs. This confirms the radio stall and counters on this board. Non-blocking D has both radios off, 128 absent probes, no TX timeouts or hardware loss.

Healthy C/D/E/L/U runs keep both populated barometers healthy in every steady-state health report at approximately 40 Hz. The normal no-radio loop still reaches 23,689 us: removing radio stalls does not eliminate other long sections. I2C peripherals and IMU SPI acquisition dominate remaining loop work. No >64 ms steady stalls were observed in those runs.

L physically decoded valid downlink Capsule ID `0x0C`, length 54 (144 packets total including startup). U transmitted normal uplink Capsule ID `0x08`, two-byte order-0/value-0 payload, and the normal RX decoder logged it before dispatch ignored unknown order 0. A bench whitelist rejected any other command. These verify both local RF/decoder directions, **not** external ground-station interoperability, range, or a valid actuator-affecting command. Normal downlink measured up to 93 ms airtime; uplink up to 102 ms. Non-blocking main-loop radio service remained below 1 ms.

## Blocking findings

### Estimator input timing / starvation

The evidence points to driver timestamp skew defeating the four-way aligned drain. The application rings hold 128 samples (about 20 ms at 6.4 kHz); observed inter-IMU clock skew/ages can exceed this window. `kalman_process.cpp` aligns each source to the newest front timestamp within 200 us, then stops processing when any healthy ring empties. A source whose newest sample predates that alignment floor is completely discarded; no complete group is consumed on that tick. Hardware loss counters can remain zero while consumption and body acceleration freeze. The driver currently anchors the last frame of a capped oldest-FIFO burst to read-completion time, then slews its offset by at most 32 us per burst with a 5 ms outlier gate/reacquisition policy. These interactions need a properly tested timestamp correction; increasing an alignment tolerance alone would risk fusing nonsynchronous samples.

In isolated E, flight-mode estimator lag reached **972,186 us**, despite zero hardware gaps and a 3,065 us maximum Kalman section. The artificially forced stationary flight produced an apogee flag; this is not a normal stationary false-apogee test. Real FSM state remained INIT.

G was already starved before reset (`consumed=18944`, `drop=9568`). At 60 s, `kalman_request_reset()` was accepted and printed `Reset done`. Body acceleration then became zero and stayed zero through the capture while all four IMUs continued streaming. **API acceptance passes; recovery/reconvergence does not.** The reset API does not establish that the lower-level acquisition alignment problem has recovered. An additional source-level defect explains why counters did not zero: `resetRuntime()` resets `KalmanHealthStore` but not its `KalmanRuntime::health` cache, which is republished at the end of the loop. This was documented, not patched during the final acceptance capture; fix it and add reset-counter regression coverage alongside functional recovery checks.

Immediate bulk logging produced startup Kalman work around 150 ms, loop stalls around 180 ms, exhausted the 258,048-byte arena, and generated 6,201 write failures. Hardware FIFOs subsequently reached 398 frames and suffered sustained gaps; driver timestamp ages grew to seconds. The combined immediate-bulk/forced-flight run `E_final_20261008_224400.log` was worse (9,226 write failures; estimator lag reached 54.743397 s). Neither is the isolated E CPU measurement.

Delayed bulk logging starts at 60 s to distinguish this startup-only overload from normal arming after settling. Over 128.837 s of steady loaded measurement, it produced **zero hardware gaps, zero SD write failures and zero raw-IMU batch failures**. Both populated barometers stayed healthy in all 129 reports (about 40.9 Hz), loop maximum was 22,990 us, SD service maximum 4,025 us, and raw IMU logging reached 142,491 KB. Thus the immediate-start SD failure must not be generalized to logging enabled later at ARMED. Estimator consumption still stopped intermittently, including before bulk logging was enabled. This test did not combine delayed bulk logging with estimator flight mode or validate SD file contents.

### Normal soak and USB logging off

The final image has all bench switches off, source `be9c4612b3b970f925cf96970313b4d7c7cb7ff7`, SHA-256 `2b2def60cdee12a42ced32a4e2031407aea62d6eef73333ea9e0da3a814e0599`. The image/build metadata are `logs/bench/A_soak.bin` and `A_soak.build-meta`. Metadata also preserves CubeIDE-generated project normalization, subsequently removed from the working tree. OpenOCD verified the image; capture completed naturally after 630 seconds. The board was left running this normal candidate, USB logging on, real FSM INIT. `Debug/2026_C_AV_FC.bin` has the same SHA-256.

After the startup exclusion there are 546 observed reports over **615.365 s**, including the USB silence. Per-sensor rates are **6362.4 / 6403.6 / 6362.7 / 6371.9 frames/s**, with zero steady-state sample gaps/loss. Both populated barometers are healthy in **546/546** observed reports, with 25,238 / 25,236 conversion deltas (about 41 Hz). SD finishes with `fail=0`, `imu=0/0` (bulk disabled), and no UART overruns. Radio sent 546/546 in observed reports, no timeouts, maximum air time 95 ms; counters during suppressed reports are not retained. Main-loop maximum is **20,734 us**, with zero observed >64 ms steady stalls. Liftoff detector enabled/armed, no detection messages, all observed liftoff/apogee/divergence flags zero. Consumer advancement remains intermittent; last-frame age reaches 34.143 ms on IMU 0 versus at most 10.375 / 11.827 / 9.935 ms on the others. Therefore the absence of false events does not prove detector coverage during starvation.

The soak boot lost 5,269 / 5,141 / 2,725 / 4,058 hardware samples respectively during startup. These losses occurred before the steady-state exclusion window; a zero steady-state result must not be read as loss-free boot.

USB output was disabled at 23:16:32 CEST and restored at 23:17:41 (69 seconds); capture remained attached. The firmware report gap spans 211,888 → 281,205 ms (69.317 s). Cumulative driver writes increased by 441,005 / 443,863 / 441,029 / 441,664, giving **6362.1 / 6403.4 / 6362.5 / 6371.7 frames/s**, with **zero cumulative loss delta on every IMU**. Both barometers completed 2,840 conversions across the gap and were healthy afterward; SD failures remained zero. Per-report CPU maxima during the quiet interval are not retained, so this is a continuity check, not a quantitative USB CPU-cost comparison. Operator timestamps are archived in `A_soak.operations.md`.

## Host regression evidence

- `make baro-tests`: **23 tests in two suites passed**, including rejected-trigger behavior. Logs: `baro_tests.log` and `build_baro_tests.log`.
- Native replay of a recorded four-IMU/four-barometer test flight runs the firmware Kalman implementation. Logs: `replay_ignition.log/.csv`. INIT → IGNITION at 1345.297064 s; IMU liftoff epoch at 1350.319 s; acceleration-hold median 53.36 m/s² yields DID_HOLD; BURN at 1351.240564 s; apogee at 1356.329064 s. ESKF peak 142.249 m versus approximately 143.5 m barometric reference.
- Replay reset accepted in INIT and refused in IGNITION (`replay_reset.log/.csv`), with the same apogee timing. This validates policy offline, not the failed board recovery.
- `replay_flown.csv` and `replay_fake_gps.csv` are byte-identical: injected bogus GNSS is not fused.

Host replay does not model real CPU time, FIFO service, SPI timing or SD load. Its passing flight results do not override the physical timing failures.

## Limits and next actions

1. Fix/retest IMU timestamp alignment and consumer liveness before relying on the estimator. Acceptance must include advancing consumed timestamps and bounded flight lag, not just zero hardware gaps or a `div=0` flag.
2. Logging enabled after settling passed acquisition/SD counters. Next test safe combined estimator-flight/delayed-bulk load after the timing problem is fixed. Inspect startup allocation/scheduling; preserve SD write-failure and raw-IMU batch-failure counters independently.
3. Mounting-on-rail check remains unverified. Bench gravity is predominantly body −Y, not the required nose-up body +X. **Subsequent user clarification on October 9:** the board is on its side, with raw IMU X looking down, so this bench gravity is expected and does not justify changing the mounting matrix. See `2026-10-09-remote-fixes-results.md` for the follow-up fixes and acceptance results.
4. No raw SD readback/decoding was performed. DMA/write counters and logging enable were checked, but recorded content integrity still needs card retrieval or a reviewed readback mechanism. USB MSC exposure would be a separate change; it was not implemented opportunistically.
5. No external ground station, RF range test, physical flight-state transitions, absent-GPS scenario, or genuine in-flight reset test. All four barometers cannot be tested on this two-barometer board.
6. Existing flight-system issues in the October 7 handoff remain: finite ascent fallback, recovery actuation, flight configuration/ColdflowMode, FSM latch/timer reset and CLI/uplink wiring for reset. Bench success does not close these.

Early diagnostic captures (`A_clock` interrupted by reflash, initial zero-frame SPI tests, removed timestamp-anchor experiment) are retained as diagnostic evidence and excluded from acceptance rows. One early `A_clock` metadata hash was corrected explicitly to its actual archived image; subsequent runs always flash named immutable artifacts.
