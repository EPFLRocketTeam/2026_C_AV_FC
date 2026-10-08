# Non-blocking telemetry: change summary and bench plan

Branch `fix/telemetry-nonblocking` (on `feat/sd-decoder`).

## Why

The IMU FIFOs are only read from the super loop. Any loop stall longer than the FIFO depth loses frames. One ICM-45686 at 6.4 kHz fills 2 KB in ~16 ms and 8 KB in ~64 ms. The old radio driver stalls the loop on every 1 Hz downlink:

| | Telemetry board present | Telemetry board absent |
|---|---|---|
| `SX127X_config()` on every packet (`HAL_Delay(15)`) | 16 ms | 16 ms |
| Wait for TxDone (59 B, SF7/BW250/CR4-7) | ~75 ms | — |
| `LoRaEntryTx` readback, 3000 polls, never matches | — | ~20 ms |
| `SX127X_hw_Reset()` (100 ms) + `config` again | — | ~117 ms |
| Stall per second | ~92 ms | ~154 ms |
| Expected IMU rate if the FIFO holds 64 ms | ~24.7k frames/s | ~23.3k frames/s |

These expected rates match the measured ~24.8k and ~23k frames/s, which suggests the FIFO really holds ~64 ms. The worst case is worse still: if a packet never completes, `LoRaTxPacket` waits 3000 × 1 ms, so all IMUs go blind for 3 s on every send.

## What changed

- **`sx127x`**: added `SX127X_isPresent()` (RegVersion = 0x12), `SX127X_txPrepare()`, `SX127X_txStart()`, `SX127X_txPoll()` (TxDone flag over SPI; DIO0 is not relied on) and `SX127X_rxStart()`. The blocking calls are unchanged.
- **`TxRadioModule`**:
  - The radio is configured once at init.
  - `send()` loads the packet and starts TX (a few SPI transfers, well under 1 ms); `tick()` picks up TxDone.
  - Timeout is 500 ms. After 3 consecutive timeouts, or when the radio is missing, it retries every 5 s with one register read, and does one full reconfiguration (16 ms) when the radio answers.
  - Before each packet, one register read catches a board unplugged since the last send.
- **`RxRadioModule`**:
  - Not polled while the radio is missing.
  - Every 1 s, one register read checks presence, plus RegOpMode. A radio that came back, or lost its configuration, is put back into continuous RX without the blocking `receive()`.
- **`main.c`**: `simple_radio_tick()` runs in the loop (`APP_RADIO_ENABLE`, default 1).
- **Instrumentation** (`APP_PERF_TRACE`, default 1, one report per `APP_PERF_REPORT_MS` = 1 s):
  - `[PERF]`: iterations, average and maximum loop time (start to start, including the radio tick and CAN drain in `main.c`), number of stalls over 5, 16 and 64 ms, and the worst time of each section.
  - `[IMU-ACQ]`, per IMU:
    - frames received (`fr`), timestamp gaps over 1.5 sample periods (`gap`), samples missing in them (`lost`, and `tot` since boot), and the largest step between frames (`dt`);
    - highest hardware FIFO_COUNT seen (`hwm`, in frames), reads capped at the 102-frame burst buffer (`cap`), and INT1_STATUS0.FIFO_FULL seen (`full`).
  - `[RADIO]`: TX `start`, `sent`, `busy`, `absent`, `tmo`, `reconf`, longest airtime (`air`); RX packets and reconfigurations.
- **Bench switches** (default 0):
  - `APP_RADIO_BLOCKING_TX` sends with the former blocking call, for A/B runs on the same build.
  - `APP_RADIO_HOLD_IN_RESET` holds both radios in reset after init, to simulate an absent telemetry board without unplugging it.

USB logging is not changed here. With `ENABLE_USB_LOG` (`app_printf.h`) or the shell toggle off, `_write` returns before touching USB. With it on, a printf can still wait up to 100 ms when the host does not read the port, so keep the serial capture running during the runs below.

## Builds for the remote session

| Build | How | Purpose |
|---|---|---|
| A | `feat/rekalman` (Kalman flight-readiness stack) + `fix/telemetry-nonblocking` | Candidate |
| B | A with `APP_RADIO_BLOCKING_TX 1` (`tx_radio_module.hpp`) | Old radio behaviour, A/B |
| C | A with `APP_RADIO_ENABLE 0` (`Core/Src/main.c`) | No radio, baseline |
| D | A with `APP_RADIO_HOLD_IN_RESET 1` (`radio_process.cpp`) | Telemetry board missing |
| E | A with `-DKALMAN_DEBUG_FORCE_FLIGHT=1` in the project defines (C and C++) | Estimator flight-mode CPU load |

Flash and capture with `make -f makefile.targets deploy` (60 s) or `flash-remote`, then `serial`, from `Debug/`. Let each run go for at least 2 minutes. Summarise with `python3 Application/Tests/bench/perf_summary.py logs/uart_*.log` (the first 15 reports are skipped).

## Runs and pass criteria

1. **A, board present.** Pass criteria:
   - all IMUs `fr` ≈ 6400 per report, `lost` = 0;
   - `[PERF]` `radio` worst < 1 ms, no stall > 16 ms caused by the radio;
   - `[RADIO]` `sent` = 1 per report, `air` ≈ 75 ms, `tmo` = 0;
   - the ground station receives the downlink.
2. **A, uplink.** Send a harmless command from the ground station (for example a camera command). Expect `rx pkt` to increment and the `Decoded packet` line. Also check that the boot log has no `Failed to enter in reception mode`.
3. **B, board present.** Expect `radio` ≈ 92 ms and ~0.8k frames/s lost in total. This confirms the cause and the counters, and `hwm` gives the real FIFO depth: ~409 frames for 8 KB, ~102 for 2 KB.
4. **C.** Reference for the loop sections without radio.
5. **D.** Expect `tx=off rx=off`, `absent` counting, `radio` worst < 1 ms, and `lost` = 0.
6. **A, USB logging off.** Turn USB logging off with the shell command `logs usb fc` (false) for 60 s, then back on. The `tot` counter covers the silent period. This shows the cost of the serial output itself.
7. **E.** Check `kal` and the loop maxima with the estimator in flight mode.

## Open points

- The RX module's DIO0 and reset lines through the spine: the spine document lists only the TX ones; the RX reset is confirmed. Nothing here depends on DIO0.
- The packet rate stays at 1 Hz. With the non-blocking path it could go up to roughly 1 packet per 100 ms if useful.
- `[IMU STATUS]` and `[BARO STATUS]` print 12 lines every second, each a USB write; they could be folded into one line.
