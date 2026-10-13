# Remote FC fixes — 2026-10-09

## Status

Bench acceptance is complete: reset recovery, immediate raw logging, combined estimator-flight/raw logging and final normal-image soak passed. This report supersedes the timing/reset findings in `2026-10-08-remote-bench-results.md`; that report remains the record of the original telemetry A/B tests and failed baseline. This closes the timing/reset/throughput bench failures, not every project issue or flight-readiness requirement.

All physical tests leave the real FSM in INIT. Forced-estimator-flight builds call the estimator lifecycle directly, without arming, launch, valve, pyro or recovery commands. A synthetic stationary flight can set an apogee flag; that is not a false-event test of the normal image. No changes were pushed.

The user confirmed four IMUs, two populated barometers (indices 0 and 3), and a sideways board. Predominantly body −Y gravity is therefore expected; the mounting rotation was not changed to make this bench look nose-up. GPS NAV-PVT traffic is present, but position quality and absence-of-GPS operation are not validated. There is no external ground station.

## Root causes and retained solutions

### Sensor timing and aligned ingestion

- The old mapper anchored the last frame of a capped **oldest-FIFO** burst to the present. It ignored the unread tail, including the erratum's retained last packet. This made old samples appear current and distorted inter-sensor alignment.
- `Drivers/InvIMU/fifo_clock.hpp` now maps the 16-bit counter against the authoritative FIFO-count transaction, subtracts the unread tail's age, and estimates oscillator phase **and rate** with bounded correction. It recovers additional complete counter wraps after genuine long service gaps and maintains monotonic timestamps. The driver rejects packets lacking the configured timestamp/high-resolution format rather than interpreting padding as time.
- FSYNC is tagging, not an external sampling clock. Free-running oscillator differences remain observable and are compensated by the mapper, not by pretending that PWM locks all four counters.
- The consumer rechecks alignment for **every** group, recomputes the alignment floor after discards, and waits nondestructively when a healthy source is empty. The 200 µs tolerance was not widened. Pending grouped data is cleared after an alignment discard.
- Acquisition diagnostics distinguish hardware timestamp gaps, application-ring overwrites and estimator-queue drops. Cumulative counters remain meaningful across USB output suppression.
- Acquisition is bounded to one poll per source per pass, rather than two speculative re-polls before consuming the rings. This limits redundant bus work but does not eliminate every startup overwrite.

Datasheet checks use the supplied `logs/ds-000577-icm-45686-datasheet.pdf` (DS-000577 revision 1.0): §3.5 permits 24 MHz SPI at VDDIO ≥1.71 V and 20 MHz below that; §15 explicitly says FIFO count/data default to **little endian**, although the register-map descriptions show big endian; §17.18–19 count packets, not bytes; §17.34 selects 1 µs timestamp resolution. The double-count-read/M−1 stream workaround remains in place.

### Cache-enabled SPI bring-up

Enabling instruction cache exposed a deterministic soft-reset failure on all four IMUs: WHO_AM_I remained `0xE9`, but the SDK never observed reset-done and subsequent reads showed data-ready. HAL reported no transfer error. Buffered tracing with extra timebase calls could hide the failure; minimal cycle-only tracing did not.

Retained corrections:

- All sensor/radio chip-select outputs start deasserted HIGH. Mixed GPIO initializations were split so pyro/buzzer outputs retain their original LOW state.
- SPI4/5 keep their IO state when disabled (`SPI_MASTER_KEEP_IO_STATE_ENABLE`), in both generated source and `.ioc`. This restored 4/4 initialization on repeated cache-enabled boots, including an independent run with the EOT guard disabled.
- A bounded master-SPI EOT delay before disabling SPE implements the separate [STM32H743 ES0392 §2.22.6 workaround](https://www.st.com/content/ccc/resource/technical/document/errata_sheet/group0/b8/f4/b7/a3/d1/a0/44/a6/DM00368411/files/DM00368411.pdf/jcr:content/translations/en.DM00368411.pdf). The configured 1 µs guard covers the board's slowest configured 4 MHz SCK. It uses DWT only when both trace and the cycle counter are enabled; otherwise it uses a conservative bounded CPU loop. This guard alone did **not** fix bring-up.
- Full-duplex register writes were tested and reverted because they did not solve the failure. Current FIFO reads remain single full-duplex transfers; register writes retain the original transport.

Instruction cache is enabled. Data cache is enabled only with an explicit DMA memory policy:

- AXI SRAM is cached, except the first 32 KiB reserved for the non-cacheable SDMMC IDMA bounce buffers.
- D2 SRAM1/2 (first 256 KiB at `0x30000000`) hold the **CPU-only cached SD arena**. SD writes copy into the bounce buffer; they never DMA directly from this arena. A data-memory barrier publishes the bounce-buffer stores before either DMA-write path starts.
- GPS DMA1's circular buffer resides in a separate non-cacheable 32 KiB SRAM3 region at `0x30040000`. Linker assertions protect both placements; startup zero-fill includes the moved GPS buffer.
- USB DMA and SPI DMA are disabled. A shared C/C++ default rejects enabling the unaudited SPI-DMA option alongside data cache.

Initially making all of D2 non-cacheable was safe for DMA but too expensive under combined estimator/raw-SD load. Caching the CPU-owned arena, while retaining the two explicit DMA carve-outs, is necessary for the final combined-load result. This is not a blanket cache enable without a memory audit.

SPI4/5 remain at **8 MHz** for registers and the shared BMP390 devices. Only serialized blocking ICM FIFO bursts temporarily use **16 MHz**, restoring the divider before another bus client runs. This is below either ICM voltage-dependent maximum and avoids exceeding the BMP390's 10 MHz limit.

### Full-rate estimator CPU load

- Ground hibernation keeps a rolling history, not an unread flight queue. Its intentional wrap no longer counts as data loss or copies a complete rewind checkpoint on every overwritten sample. Active-flight overflow still counts as loss and retains its checkpoint behavior.
- Group preparation no longer clears unused sample-array ranges. Only initialized, present-source ranges are passed to the virtual IMU.
- On-board profiling identified covariance propagation as the prediction hot spot: approximately 43 µs of a 51 µs prediction. GCC `O3` on that fixed-size sparse routine reduces covariance work to about 28 µs. Subsequent combined-load profiling justified the same optimization for the virtual-IMU translation unit and shadow-filter prediction. Applying it to the complete virtual-IMU unit lets its helper routines inline consistently. No fast-math, precision reduction, covariance decimation or changed equations were introduced; recorded-flight CSV outputs remained byte-identical.
- The application catch-up budget is now a bounded **4 ms**, rather than 2 ms. At 2 ms even the unprofiled optimized build eventually accumulated a roughly 0.5 s backlog and dropped estimator entries. At 4 ms it catches up and services the FIFOs between quanta.
- Hibernating ground history now reports zero estimator backlog, rather than the entire uptime since a stationary ESKF timestamp.

### Reset and logging

`resetRuntime()` now clears its cached health snapshot and main-loop timing atomics, not just the published store. A reset-generation counter resets application metric delta baselines, avoiding unsigned underflow after counters clear. Host and physical checks verify fresh ingestion resumes, not merely that the API accepts the request.

Restoring continuous ingestion revealed another SD bottleneck: the **352-byte preflight pipeline diagnostic** was constructed/logged at roughly 6.4 kHz. With its framing and full-rate raw IMUs this exceeds 3 MB/s, whereas the loaded ground loop drained about 2.4 MB/s. Enlarging the arena cannot solve sustained excess production. The formerly stalled consumer had hidden this load.

Pipeline diagnostics now use a per-estimator sample-time limiter, approximately **100 Hz before and after liftoff**, reset with the estimator. This matches the previous flight diagnostic rate. **Every raw IMU sample and every estimator update remain full-rate.** `ESKF_APP_IMU_PIPELINE_LOG_INTERVAL_US` defaults to 10,000; zero is an explicit unthrottled bench override, not a validated production SD load. Record layout is unchanged. The existing INIT-off/ARMED-on gate remains intact.

## Evidence

All named captures and immutable images/build flags/source/diffs/SHA-256 are in ignored `logs/bench/`. `make deploy` now leases both flash and capture, rejecting a competing deploy before it can reset the board. A held-lock rejection was tested without touching the board. Flash/capture failures remain fatal; only the intended capture timeout is accepted. Builds have 0 errors and 75 existing warnings.

| Capture | Result |
|---|---|
| `I_fastfifo_20261009_001522.log` | Four sensors at 6362–6403 Hz, zero hardware loss; preflight consumer continuously advances |
| `I_flight_20261009_001704.log` | I-cache alone insufficient: hardware loss and active estimator overflow |
| `I_dcache_20261009_002026.log` | D-cache restores full-rate acquisition, but 2 ms estimator quantum still overflows |
| `I_cov_O3_20261009_002735.log` | Covariance time improves; heavily instrumented 2 ms run still overflows |
| `I_cov_plain_20261009_003146.log` | Unprofiled 2 ms run also overflows; interrupted at approximately 118 s, negative diagnostic evidence only, not acceptance |
| `I_budget4_20261009_003355.log` | 145 s completed. Steady 113.444 s: 6363.3 / 6404.5 / 6363.6 / 6372.8 Hz, zero hardware loss, software overwrites and estimator drops; worst lag 13,280 µs; SD failures 0 |
| `J_reset_20261009_003644.log` | Reset at 60 s accepted; consumption 1,479,124 → 168 → 25,612 → 51,068 in successive reports; ring high-water marks clear and rebuild; no post-reset starvation or false events |
| `J_bulk_20261009_003918.log` | Reveals unbounded preflight diagnostic load: 386,994 ordinary-record failures and 2,639 raw-batch failures; hardware acquisition still full-rate |
| `K_bulk_20261009_004209.log` | 145 s completed with 100 Hz diagnostics and immediate full-rate raw logging: zero steady loss and SD failures, raw batches 468,563 / 0 failures |
| `K_combined_20261009_004449.log` | Combined flight estimator + delayed bulk still fails with non-cacheable arena: approximately 400 hardware samples/s/sensor lost; estimator drop=0 alone would mask this failure |
| `L_combined_20261009_004827.log` | Virtual/shadow `O3` improves flight-only scheduling but does not fix combined SD load: hardware loss and application overwrites persist |
| `M_arena_20261009_005219.log` | 200 s completed with cached CPU arena: steady 129.365 s at 6361.8 / 6403.0 / 6362.1 / 6371.3 Hz, zero hardware/application/estimator loss, lag ≤11,207 µs, SD failures 0 |
| `N_stress_20261009_005615.log` | Final defaults, immediate raw logging plus estimator flight from 20 s, 240 s completed: steady 209.151 s at 6362.4 / 6403.6 / 6362.7 / 6371.9 Hz, zero hardware/application/estimator loss, lag ≤12,323 µs, SD failures 0 |
| `N_normal_20261009_010125.log` | Normal firmware, 630 s completed naturally: steady 614.000 s at 6361.4 / 6402.7 / 6361.8 / 6371.0 Hz, zero new hardware/application/estimator loss, SD failures 0, no liftoff/apogee/divergence flags |

For final stress, the real FSM remained INIT throughout; synthetic stationary estimator flight produced the expected apogee flag. After the 30 s exclusion, all 208 reports had both populated barometers healthy (8,264 conversions each, about 39.5 Hz), GPS had no UART overruns, and radio TX completed 208/208 with no timeouts. Mean loop duration was 6,985 µs, worst loop 21,114 µs, worst Kalman section 7,831 µs. Raw SD logging ended at 320,846 batches / 0 failures, 242,653 KB, with arena occupancy 5,412 / 258,048 bytes. The 4 ms budget is a catch-up quantum, not a hard bound on each whole loop or on synchronous liftoff rewind.

Final stress had **20 startup application-ring overwrites on IMU 3**, with no later increase. The earlier cached-arena run had 13. The one-poll limit is a bounded scheduling improvement, not evidence that startup retention loss has been eliminated. Capped startup FIFOs and the initial estimator-flight rewind are excluded explicitly; steady-state acceptance does not prove complete capture from power-on.

The reset run completed 145 s; its last 79 s measured 6362.6 / 6403.9 / 6363.0 / 6372.2 Hz, zero hardware/application/estimator loss, SD failures 0, and both populated barometers healthy in all 80 reports. Ground ring overwrite totals included 14 startup samples on sensor 3, with no later increase. Initial FIFO high-water marks can reach 398 before super-loop acquisition starts; loss counters cannot establish completeness before the first valid timestamp. These are **steady-state** acceptance results, not a claim of complete capture from power-on.

Host checks:

- Clock/alignment tests: 8/8 passed, covering capped bursts, adjacent/multiple wraps, ±0.6% oscillator drift with polling jitter, internal gaps, empty healthy rings and unhealthy sources.
- IMU tests: 14/14 passed, including previous driver/module tests and new four-source publication/overwrite accounting and bounded-poll behavior. The legacy asynchronous mock's IRQ signature was repaired; module tests complete the mock transfer inside tick to match production's blocking path, while driver DMA tests retain their asynchronous model.
- Barometer tests: 23/23 passed.
- Virtual IMU, shadow, ESKF and math/parity tests: 96 passed, two skipped because they require covariance decimation, disabled in the production configuration. Together the four GoogleTest targets pass **141 tests, with two configuration-dependent skips**. `make clock-tests imu-tests baro-tests math-tests` reproduces them.
- Actual-runtime reset regression: cached health/timing clear, generation increments, fresh input resumes, ground history wrap is not false loss, active overflow still counts, and IGNITION reset is refused.
- Actual-estimator logging regression: bounded pipeline rate in ground/flight and after reset; disabled logging emits no snapshots; logged/unlogged navigation outputs are identical at every step.
- Recorded flight replay after the CPU changes: IMU epoch 1350.319 s, acceleration hold DID_HOLD at median 53.36 m/s², BURN 1351.240564 s, apogee event 1356.330064 s. ESKF peak 142.481 m at 1356.326064 s, shadow peak 142.543 m at 1356.376064 s. Compiler-optimized and preceding fixed replay CSVs are byte-identical; bogus-GPS/flown CSVs are byte-identical. Reset is accepted in INIT and refused in IGNITION.
- Replay build caching now tracks generated header dependencies, preventing stale configuration or incompatible class layouts when only a header changes. Host replay validates numerical behavior, not target CPU scheduling, SD integrity or physical flight readiness.

To reproduce the two actual-runtime assertions from the project root:

```sh
RUN_SOURCE="$PWD/Application/Tests/host_replay/test_runtime_reset.cpp" Application/Tests/host_replay/build.sh "$PWD" "$PWD/Debug/reset_final"
Debug/reset_final/harness
RUN_SOURCE="$PWD/Application/Tests/host_replay/test_pipeline_logging.cpp" Application/Tests/host_replay/build.sh "$PWD" "$PWD/Debug/pipeline_final"
Debug/pipeline_final/harness
```

### Final normal image

`N_normal.bin` has no build overrides: estimator-flight, bulk-at-boot, timed-reset and expensive profiling hooks are all disabled. Firmware source is `aae50eb48fef92112a4596d26b5d004c76812d52` (subsequent commit `aa9424c` changes host test/build tooling only). SHA-256 is `fec13d7efad15865cd1861d93b72615e413d604c4d7be6a97cb25d758922850d`; `Debug/2026_C_AV_FC.bin` matches. OpenOCD verified it; capture `N_normal_20261009_010125.log` completed naturally after 630 seconds, with deploy exit status 0.

After excluding the first 15 reports, 524 observed reports span **614.000 s**, including USB suppression. Cumulative per-sensor rates are **6361.4 / 6402.7 / 6361.8 / 6371.0 Hz**, with zero new hardware loss or application overwrites. Normal startup includes **10 application-ring overwrites on IMU 3**; its cumulative total then stays at 10. Both populated barometers are healthy in **524/524** reports, completing 28,679 conversions each (about 46.7 Hz). Radio sends 524/524 observed reports without timeouts; GPS has no UART overruns. Average loop time is 733 µs, worst loop 15,271 µs, worst Kalman section 3,248 µs; no >16 ms steady stalls are observed. SD ends at `wr=992203 fail=0`, `imu=0/0` as expected with normal INIT bulk logging off. Estimator ingestion reaches **16,007,648 samples**, with zero drops and no false liftoff, apogee or divergence. Zero ground backlog is intentional hibernating-history semantics; advancing consumption and sample ages, not that zero alone, establish ground liveness.

The last report at 630,006 ms shows per-source last-frame ages **1567 / 1498 / 1419 / 1348 µs**, no clock repairs, zero cumulative hardware loss and unchanged software-overwrite totals. The board was left running this normal image in **INIT**, with runtime USB diagnostics disabled at **01:12:04 CEST**, immediately before the capture closed. Build-generated `.cproject` normalization was removed from the working tree; immutable build metadata preserves it for provenance.

USB diagnostic output was suppressed from 01:02:52 to 01:04:23 CEST, while capture stayed attached. Firmware reports straddle 76,005 → 168,005 ms (92 s): cumulative writes increased by 585,320 / 589,114 / 585,352 / 586,198, corresponding to 6362.2 / 6403.4 / 6362.5 / 6371.7 Hz. Hardware loss and application-overwrite deltas are zero on every source. Estimator consumption increased by 2,341,280; both barometers completed 4,298 conversions, SD failure stayed zero, and no event flags appeared afterward. Per-interval CPU maxima are not retained through suppression; this is continuity evidence, not a quantitative USB CPU-cost comparison. Operator actions are archived in `N_normal.operations.md`.

## Boundaries and remaining work

Immediate bulk with the new diagnostic rate, combined estimator-flight plus delayed bulk, final-default immediate-bulk/estimator-flight stress, and the final normal-image soak have passed. Startup retention losses remain as documented above; these results must not be generalized to loss-free capture from power-on.

No raw SD readback/decoding, external ground-station interoperability/range test, real flight-state/actuation test, missing-GPS test, or four-barometer validation was performed. The unrelated FSM/recovery/configuration issues from the earlier handoff remain outside these bench fixes. Passing CPU/acquisition tests is **not a flight-readiness declaration**.

USB suppression was tested with a host still attached. Physical USB removal and a connected host that stops draining CDC are not validated here; the existing printf transport can wait up to 100 ms on BUSY. Runtime USB diagnostic printing is disabled at handoff to avoid depending on an unattended reader. `logs usb fc on` restores it when a monitor is attached. This does not disable SD logging or acquisition.

Maintenance: Cube regeneration should preserve CS defaults and SPI keep-IO settings via `.ioc`, and cache setup via USER CODE. HAL driver replacement can overwrite the explicit EOT workaround; reapply/review it. Changes to DMA clients or memory placement require a new cache audit. Retest the scheduling margin if ODR, precision, enabled sensors, logging or math cost changes.
