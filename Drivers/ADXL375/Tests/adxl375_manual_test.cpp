/**
 * @file  adxl375_manual_test.cpp
 * @brief Manual on-target test for ADXL375_Driver over real I2C.
 *
 * ─── Setup ───────────────────────────────────────────────────────────────────
 *  1. Fill in the board-specific bindings below (I2C handle, address).
 *  2. Call ADXL375_ManualTest_Run() from main() after HAL_Init() and clock
 *     configuration, with printf retargeted to UART or SWO.
 *
 * ─── Expected console output (passing) ───────────────────────────────────────
 *  [ACC] ===== ADXL375 Manual Test =====
 *  [ACC] PASS  init()
 *  [ACC] PASS  ping()
 *  [ACC] PASS  configure() — no error flag
 *  [ACC] PASS  readBlocking()
 *  [ACC] PASS  getFrame() N/N+1 pattern
 *  [ACC] PASS  gravity plausibility (0.5-2.0 g at rest)
 *  [ACC] PASS  shutdown() -> wake() -> read
 *  [ACC] PASS  stats: triggers>=3 reads>=3 errors=0
 *  [ACC] ===== 8 passed, 0 failed =====
 *  [ACC] live readout — move the sensor and watch it change:
 *  [ACC]   x=0.049 g y=-0.098 g z=0.980 g   ...
 */

// ─── Board-specific bindings — EDIT THESE ────────────────────────────────────

#include "main.h"
#include "adxl375_manual_test.h"
#include "../ADXL375.hpp"

#include <cmath>
#include <cstdio>

extern I2C_HandleTypeDef hi2c4;              // whichever I2C bus ADXL375 is on

static constexpr uint8_t ADXL375_ADDR7 = 0x1D;  // ALT ADDRESS pin strapped high (0x53 if low)

// ─────────────────────────────────────────────────────────────────────────────

using namespace Drivers::ADXL375;

#define ACC_LOG(fmt, ...) printf("[ACC] " fmt "\r\n", ##__VA_ARGS__)

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT(cond, msg)                                            \
    do {                                                             \
        if (cond) {                                                  \
            ++g_pass;                                                \
            ACC_LOG("PASS  " msg);                                   \
        } else {                                                     \
            ++g_fail;                                                \
            ACC_LOG("FAIL  " msg " (line %d)", __LINE__);            \
        }                                                             \
    } while (0)

static void delay_ms(uint32_t ms) {
    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < ms) {}
}

// ─────────────────────────────────────────────────────────────────────────────

static void test_init_ping_configure(ADXL375_Driver& acc) {
    bool init_ok = acc.init();
    ASSERT(init_ok, "init()");
    if (!init_ok) {
        ACC_LOG("      error: %s", acc.lastError());
        return;
    }

    bool ping_ok = acc.ping();
    ASSERT(ping_ok, "ping()");
    if (!ping_ok) {
        ACC_LOG("      error: %s", acc.lastError());
    }

    acc.configure(OutputDataRate::hz100);
    ASSERT((acc.getStatus() & ADXL375_STATUS_CONFIG_ERROR) == 0,
           "configure() — no error flag");
}

static void test_calibrate(ADXL375_Driver& acc) {
    delay_ms(150);

    // Assumes the board is flat with Z-axis vertical (+1 g) — override the
    // reference vector here if that assumption doesn't match the mounting.
    bool ok = acc.calibrate(0.0f, 0.0f, 1.0f, 10);
    ASSERT(ok, "calibrate()");

    delay_ms(150);
    AccelData d{};
    acc.readBlocking(d);
    float mag = sqrtf(d.x_g * d.x_g + d.y_g * d.y_g + d.z_g * d.z_g);
    ACC_LOG("      post-calibration |a|=%.3f g (x=%.3f y=%.3f z=%.3f)",
            (double)mag, (double)d.x_g, (double)d.y_g, (double)d.z_g);
    // Loose bound: confirms the OFSX/OFSY/OFSZ scale-factor assumption
    // (0.196 g/LSB — see impl comment on the datasheet's self-contradiction)
    // is in the right ballpark. If this is wildly off (e.g. ~8x under or
    // over 1 g), the 1.56 g/LSB datasheet value is the one to use instead.
    ASSERT(mag > 0.7f && mag < 1.3f, "post-calibration magnitude near 1.0 g");
}

static void test_read_blocking(ADXL375_Driver& acc) {
    delay_ms(150);

    AccelData d{};
    bool ok = acc.readBlocking(d);
    ASSERT(ok, "readBlocking()");
    if (ok) {
        ACC_LOG("      x=%.3f g y=%.3f g z=%.3f g, ts=%lu us",
                (double)d.x_g, (double)d.y_g, (double)d.z_g,
                (unsigned long)d.timestamp_us);
    } else {
        ACC_LOG("      error: %s", acc.lastError());
    }
}

static void test_getFrame_pattern(ADXL375_Driver& acc) {
    delay_ms(150);

    acc.triggerMeasurement();

    // Unlike TMP1075's one-shot gate, the ADXL375 free-runs continuously —
    // DATA_READY may already be set by the time we poll, so there is no
    // guaranteed "too early" window to assert against. Poll briefly instead.
    AccelData d{};
    bool ok = false;
    for (int i = 0; i < 20 && !ok; i++) {
        ok = acc.getFrame(d);
        if (!ok) delay_ms(2);
    }
    ASSERT(ok, "getFrame() N/N+1 pattern");
    if (ok) {
        ACC_LOG("      x=%.3f g y=%.3f g z=%.3f g", (double)d.x_g, (double)d.y_g, (double)d.z_g);
    }
}

static void test_gravity_plausibility(ADXL375_Driver& acc) {
    delay_ms(150);
    AccelData d{};
    acc.readBlocking(d);
    float mag = sqrtf(d.x_g * d.x_g + d.y_g * d.y_g + d.z_g * d.z_g);
    ACC_LOG("      |a|=%.3f g (x=%.3f y=%.3f z=%.3f)",
            (double)mag, (double)d.x_g, (double)d.y_g, (double)d.z_g);
    // Raw/uncalibrated 0 g offset can be up to ±6 g per axis (datasheet
    // Table 1), so this only catches gross failures (stuck-at-zero, garbage
    // reads) — it does not assert calibrated accuracy.
    ASSERT(mag > 0.3f && mag < 8.0f, "gravity plausibility (raw/uncalibrated, 0.3-8.0 g)");
}

static void test_shutdown_wake(ADXL375_Driver& acc) {
    bool sd = acc.shutdown();
    delay_ms(150);
    bool wk = acc.wake();
    delay_ms(150);
    AccelData d{};
    bool rd = acc.readBlocking(d);
    ASSERT(sd && wk && rd, "shutdown() -> wake() -> read");
}

static void test_stats(ADXL375_Driver& acc) {
    const ADXL375Stats& s = acc.stats();
    ACC_LOG("      triggers=%lu reads=%lu errors=%lu lastConv=%lu us",
            (unsigned long)s.numTriggers, (unsigned long)s.numReads,
            (unsigned long)s.numErrors, (unsigned long)s.conversion_us_last);
    ASSERT(s.numTriggers >= 3 && s.numReads >= 3 && s.numErrors == 0,
           "stats: triggers>=3 reads>=3 errors=0");
}

// ─────────────────────────────────────────────────────────────────────────────

void ADXL375_ManualTest_Run() {
    ADXL375_Driver::Config cfg{};
    cfg.hi2c          = &hi2c4;
    cfg.address7      = ADXL375_ADDR7;
    cfg.rate          = OutputDataRate::hz100;
    cfg.commandRateHz = 10;

    ADXL375_Driver acc(cfg);

    ACC_LOG("===== ADXL375 Manual Test =====");

    test_init_ping_configure(acc);
    if (!acc.isHealthy()) {
        ACC_LOG("===== aborted: init failed =====");
        return;
    }

    test_calibrate(acc);
    test_read_blocking(acc);
    test_getFrame_pattern(acc);
    test_gravity_plausibility(acc);
    test_shutdown_wake(acc);
    test_stats(acc);

    ACC_LOG("===== %d passed, %d failed =====", g_pass, g_fail);

    // Live readout: ~30 s at 5 Hz. Move the sensor and watch the values change.
    ACC_LOG("live readout — move the sensor and watch it change:");
    for (int i = 0; i < 150; i++) {
        delay_ms(200);
        AccelData d{};
        if (acc.readBlocking(d)) {
            ACC_LOG("  x=%.3f g y=%.3f g z=%.3f g", (double)d.x_g, (double)d.y_g, (double)d.z_g);
        } else {
            ACC_LOG("  read failed: %s", acc.lastError());
        }
    }
    ACC_LOG("===== done =====");
}
