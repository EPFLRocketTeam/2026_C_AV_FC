# SD Logger + ESKF Integration — 2025-05-20

## Summary

Created an SD card logger that implements the `eskf::IEskfLogger` interface and logs sensor DataDumps + FSM transitions to the Plume SD card filesystem. Build succeeds; deploy failed due to remote host unreachable.

## Files Created

### `Application/Modules/sd_logger.hpp`
- Defines `SdLogRecordType` enum (DataDump, ESKF State, Covariance, Events, GPS Rejection, Rewind, RailShadow, FlightShadow, ImuPipeline, ImuDynamics, FSM Transition, Correction)
- Defines binary `SdLogHeader` (magic=0xAE, type, length, timestamp_ms) — 8 bytes
- Defines packed event structs for FSM transitions, ESKF events, corrections
- Class `SdLogger : public eskf::IEskfLogger` — concrete implementation backed by `SDCardInterface*`
- Public methods: `init()`, `logDataDump()`, `logFsmTransition()` + all IEskfLogger virtuals

### `Application/Modules/sd_logger.cpp`
- `writeRecord()`: Writes header + payload contiguously to Plume ring buffer (stack buffer for records ≤1KB, split write for larger)
- All 11 IEskfLogger virtual methods implemented (serialize struct directly)
- DataDump and FSM transition logging as standalone methods

## Files Modified

### `Drivers/Plume/plume_driver.hpp`
- Added `#pragma once` (was missing include guard, causing redefinition errors when included from multiple TUs)

### `Application/main.cpp`
- Added `#include "Modules/sd_logger.hpp"`
- Added `SdLogger g_sd_logger` instance in anonymous namespace
- In `app_super_loop_setup()`: After SD file open success, calls `g_sd_logger.init(&g_sd_interface)` and `eskf::setEskfLogger(&g_sd_logger)`
- In `app_super_loop_iterate()`: Replaced `if(false)` block with:
  - FSM transition detection (compares `dump.av_state` vs `g_last_fsm_state`)
  - Decimated DataDump logging at 62.5 Hz (16ms interval)
  - Always calls `g_sd_interface.tick()` to flush DMA batches

### `Debug/Application/Modules/subdir.mk`
- Added CPP build rule for `sd_logger.cpp` matching Application/subdir.mk include paths
- Added `sd_logger.o` to OBJS, `sd_logger.d` to CPP_DEPS
- Added clean targets for sd_logger

### `Debug/objects.list`
- Added `"./Application/Modules/sd_logger.o"`

## Build Result
```
text    data     bss     dec     hex filename
198648  33960  445760  678368  a59e0 2026_C_AV_FC.elf
```
Zero errors, only pre-existing warnings in virtual_baro.cpp/virtual_imu.cpp.

## Deploy Status
**FAILED** — `ssh: connect to host 100.126.7.111 port 22: Connection timed out`
Remote board not reachable. Needs network connectivity to flash.

## Architecture

```
┌─────────────────────────────────────────────────────┐
│ app_super_loop_iterate()                            │
│                                                     │
│  1. FSM transition → g_sd_logger.logFsmTransition() │
│  2. Every 16ms → g_sd_logger.logDataDump(&dump)     │
│  3. g_sd_interface.tick() [DMA batch flush]         │
│                                                     │
│  ... kalman_loop() ...                              │
│  ESKF internally calls:                             │
│    eskf::getEskfLogger()->logState(...)             │
│    eskf::getEskfLogger()->logCovariance(...)        │
│    eskf::getEskfLogger()->logEvent(...)             │
│    etc.                                             │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│ SdLogger::writeRecord(type, payload, len)           │
│   → SdLogHeader{0xAE, type, len, HAL_GetTick()}    │
│   → SDCardInterface::write(hdr+payload)            │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│ Plume Ring Buffer (64KB AXI SRAM)                   │
│   → tick() → multi-block DMA → SD card             │
└─────────────────────────────────────────────────────┘
```

## Binary Log Format
Every record on SD:
```
[0xAE][type:1B][length:2B LE][timestamp_ms:4B LE][payload:length bytes]
```

## What's Left
- Deploy and verify on hardware (needs network to board)
- Verify serial prints show "SD logger active, ESKF logger connected"
- Validate SD card file has parseable binary records
