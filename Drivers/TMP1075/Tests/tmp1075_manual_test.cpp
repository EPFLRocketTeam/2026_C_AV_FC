/**
 * @file  tmp1075_manual_test.cpp
 * @brief Manual on-target test for TMP1075_Driver over real I2C.
 *
 * ─── Setup ───────────────────────────────────────────────────────────────────
 *  1. Fill in the board-specific bindings below (I2C handle, address).
 *  2. Call TMP1075_ManualTest_Run() from main() after HAL_Init() and clock
 *     configuration, with printf retargeted to UART or SWO.
 *
 * ─── Expected console output (passing) ───────────────────────────────────────
 *  [TEMP] ===== TMP1075 Manual Test =====
 *  [TEMP] PASS  init()
 *  [TEMP] PASS  ping()
 *  [TEMP] PASS  configure() — no error flag
 *  [TEMP] PASS  readBlocking() — temp=24.31 C
 *  [TEMP] PASS  getFrame() refuses immediately after trigger (NOT_READY)
 *  [TEMP] PASS  getFrame() N/N+1 pattern — temp=24.38 C
 *  [TEMP] PASS  isConversionReady() time gate (2ms:no, 12ms:yes)
 *  [TEMP] PASS  rate-limit: second immediate trigger rejected
 *  [TEMP] PASS  shutdown() -> wake() -> read
 *  [TEMP] PASS  temperature plausibility (5–60 C)
 *  [TEMP] PASS  stats: triggers>=5 reads>=5 errors=0
 *  [TEMP] ===== 11 passed, 0 failed =====
 *  [TEMP] live readout — touch the sensor and watch it rise:
 *  [TEMP]   T=24.31 C
 *  [TEMP]   T=24.38 C   ...
 */

// ─── Board-specific bindings — EDIT THESE ────────────────────────────────────

#include "main.h"
#include "tmp1075_manual_test.h"
#include "../TMP1075.hpp"

#include <cstdio>

extern I2C_HandleTypeDef hi2c4;              // whichever I2C bus TMP1075 is on

static constexpr uint8_t TMP1075_ADDR7      = 0x4E;  // found via I2C bus scan (schematic doc note was off by one)
static constexpr bool    TMP1075_HAS_DIE_ID = true;  // TMP1075DSG has the die-ID register

// ─────────────────────────────────────────────────────────────────────────────

using namespace Drivers::TMP1075;

#define TEMP_LOG(fmt, ...) printf("[TEMP] " fmt "\r\n", ##__VA_ARGS__)

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT(cond, msg)                                            \
    do {                                                             \
        if (cond) {                                                  \
            ++g_pass;                                                \
            TEMP_LOG("PASS  " msg);                                  \
        } else {                                                     \
            ++g_fail;                                                \
            TEMP_LOG("FAIL  " msg " (line %d)", __LINE__);           \
        }                                                            \
    } while (0)

static void delay_ms(uint32_t ms) {
    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < ms) {}
}

// ─────────────────────────────────────────────────────────────────────────────

static void test_init_ping_configure(TMP1075_Driver& temp) {
    bool init_ok = temp.init();
    ASSERT(init_ok, "init()");
    if (!init_ok) {
        TEMP_LOG("      error: %s", temp.lastError());
        return;
    }

    bool ping_ok = temp.ping();
    ASSERT(ping_ok, "ping()");
    if (!ping_ok) {
        TEMP_LOG("      error: %s", temp.lastError());
    }

    temp.configure(ConversionRate::ms27_5, FaultCount::f1,
                   AlertPolarity::ActiveLow, AlertMode::Comparator);
    ASSERT((temp.getStatus() & TMP1075_STATUS_CONFIG_ERROR) == 0,
           "configure() — no error flag");
}

static void test_read_blocking(TMP1075_Driver& temp) {
    delay_ms(120);

    TempData d{};
    bool ok = temp.readBlocking(d);
    ASSERT(ok, "readBlocking()");
    if (ok) {
        TEMP_LOG("      temp=%.4f C, ts=%lu us",
                 (double)d.temperature_c, (unsigned long)d.timestamp_us);
    } else {
        TEMP_LOG("      error: %s", temp.lastError());
    }
}

static void test_getFrame_pattern(TMP1075_Driver& temp) {
    delay_ms(120);

    temp.triggerMeasurement();

    // Immediately after trigger the frame must NOT be available.
    TempData d{};
    bool early = temp.getFrame(d);
    bool flagged = (temp.getStatus() & TMP1075_STATUS_NOT_READY) != 0;
    ASSERT(!early && flagged, "getFrame() refuses immediately after trigger (NOT_READY)");

    // One loop iteration later (>10 ms) it must succeed.
    delay_ms(12);
    bool ok = temp.getFrame(d);
    ASSERT(ok, "getFrame() N/N+1 pattern");
    if (ok) {
        TEMP_LOG("      temp=%.4f C", (double)d.temperature_c);
    }
}

static void test_ready_time_gate(TMP1075_Driver& temp) {
    delay_ms(120);

    temp.triggerMeasurement();
    delay_ms(2);
    bool at2 = temp.isConversionReady();
    delay_ms(10);
    bool at12 = temp.isConversionReady();
    ASSERT(!at2 && at12, "isConversionReady() time gate (2ms:no, 12ms:yes)");

    TempData d{};
    temp.getFrame(d);   // drain so later tests start clean
}

static void test_rate_limit(TMP1075_Driver& temp) {
    delay_ms(120);

    bool first  = temp.triggerConversion();
    bool second = temp.triggerConversion();
    ASSERT(first && !second, "rate-limit: second immediate trigger rejected");

    delay_ms(12);
    TempData d{};
    temp.readConversion(d);
}

static void test_shutdown_wake(TMP1075_Driver& temp) {
    bool sd = temp.shutdown();
    delay_ms(50);
    bool wk = temp.wake();
    delay_ms(120);
    TempData d{};
    bool rd = temp.readBlocking(d);
    ASSERT(sd && wk && rd, "shutdown() -> wake() -> read");
}

static void test_plausibility(TMP1075_Driver& temp) {
    delay_ms(120);
    TempData d{};
    temp.readBlocking(d);
    ASSERT(d.temperature_c > 5.0f && d.temperature_c < 60.0f,
           "temperature plausibility (5-60 C)");
    TEMP_LOG("      temp=%.4f C", (double)d.temperature_c);
}

static void test_stats(TMP1075_Driver& temp) {
    const TMP1075Stats& s = temp.stats();
    TEMP_LOG("      triggers=%lu reads=%lu errors=%lu lastConv=%lu us",
             (unsigned long)s.numTriggers, (unsigned long)s.numReads,
             (unsigned long)s.numErrors,   (unsigned long)s.conversion_us_last);
    ASSERT(s.numTriggers >= 5 && s.numReads >= 5 && s.numErrors == 0,
           "stats: triggers>=5 reads>=5 errors=0");
}

static void scan_i2c_bus() {
    TEMP_LOG("scanning I2C4 bus (7-bit addr 0x03-0x77)...");
    int found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c4, static_cast<uint16_t>(addr << 1), 1, 5) == HAL_OK) {
            TEMP_LOG("  found device at 0x%02X", addr);
            found++;
        }
    }
    TEMP_LOG("scan done: %d device(s) found", found);
}

// ─────────────────────────────────────────────────────────────────────────────

void TMP1075_ManualTest_Run() {
    scan_i2c_bus();

    TMP1075_Driver::Config cfg{};
    cfg.hi2c          = &hi2c4;
    cfg.address7      = TMP1075_ADDR7;
    cfg.checkDeviceId = TMP1075_HAS_DIE_ID;
    cfg.opMode        = OpMode::OneShot;
    cfg.commandRateHz = 10;

    TMP1075_Driver temp(cfg);

    TEMP_LOG("===== TMP1075 Manual Test =====");

    test_init_ping_configure(temp);
    if (!temp.isHealthy()) {
        TEMP_LOG("===== aborted: init failed =====");
        return;
    }

    test_read_blocking(temp);
    test_getFrame_pattern(temp);
    test_ready_time_gate(temp);
    test_rate_limit(temp);
    test_shutdown_wake(temp);
    test_plausibility(temp);
    test_stats(temp);

    TEMP_LOG("===== %d passed, %d failed =====", g_pass, g_fail);

    // Live readout: ~30 s at 5 Hz. Touch the sensor and watch it rise.
    TEMP_LOG("live readout — touch the sensor and watch it rise:");
    for (int i = 0; i < 150; i++) {
        delay_ms(200);
        TempData d{};
        if (temp.readBlocking(d)) {
            TEMP_LOG("  T=%.4f C", (double)d.temperature_c);
        } else {
            TEMP_LOG("  read failed: %s", temp.lastError());
        }
    }
    TEMP_LOG("===== done =====");
}
