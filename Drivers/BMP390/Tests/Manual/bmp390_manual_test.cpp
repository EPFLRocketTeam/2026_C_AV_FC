/**
 * @file  bmp390_manual_test.cpp
 * @brief Manual on-target test for BMP390_SDK over real SPI.
 *
 * ─── Setup ───────────────────────────────────────────────────────────────────
 *  1. Fill in the board-specific bindings below (SPI handle, CS port/pin).
 *  2. Call BMP390_ManualTest_Run() from main() after HAL_Init() and clock
 *     configuration, with a UART handle for output (or use SWO/semihosting).
 *
 * printf() is assumed to be retargeted to UART or SWO.
 *
 * ─── Expected console output (passing) ───────────────────────────────────────
 *  [BARO] ===== BMP390 SDK Manual Test =====
 *  [BARO] PASS  init()
 *  [BARO] PASS  ping()
 *  [BARO] PASS  configure() — no crash
 *  [BARO] PASS  triggerConversion() returned true
 *  [BARO] PASS  isConversionReady() — DRDY within timeout
 *  [BARO] PASS  readConversion()   — pressure=101234.56 Pa, temp=24.30 C
 *  [BARO] PASS  getFrame() N/N+1 pattern
 *  [BARO] PASS  readBlocking()    — pressure=101230.12 Pa, temp=24.29 C
 *  [BARO] PASS  rate-limit: second immediate trigger rejected
 *  [BARO] PASS  stats: triggers=3 reads=3 errors=0
 *  [BARO] PASS  pressure plausibility (80000–110000 Pa)
 *  [BARO] PASS  temperature plausibility (-40–85 C)
 *  [BARO] ===== 12 passed, 0 failed =====
 */

// ─── Board-specific bindings — EDIT THESE ────────────────────────────────────

#include "main.h"
#include "bmp390_manual_test.h"
#include "../../BMP390.hpp"

#include <cstdio>
#include <cmath>

extern SPI_HandleTypeDef hspi1;           // whichever SPI peripheral BMP390 is on

#define BMP390_TEST_HSPI    (&hspi1)
#define BMP390_TEST_CS_PORT GPIOA
#define BMP390_TEST_CS_PIN  GPIO_PIN_4

// ─────────────────────────────────────────────────────────────────────────────

using namespace Drivers::BMP390;

#define BARO_LOG(fmt, ...) printf("[BARO] " fmt "\n", ##__VA_ARGS__)

static int g_pass = 0;
static int g_fail = 0;

#define ASSERT(cond, msg)                                            \
    do {                                                             \
        if (cond) {                                                  \
            ++g_pass;                                                \
            BARO_LOG("PASS  " msg);                                  \
        } else {                                                     \
            ++g_fail;                                                \
            BARO_LOG("FAIL  " msg " (line %d)", __LINE__);          \
        }                                                            \
    } while (0)

static bool approxEq(float a, float b, float eps) {
    return fabsf(a - b) < eps;
}

static void delay_ms(uint32_t ms) {
    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < ms) {}
}

// ─────────────────────────────────────────────────────────────────────────────

static void test_init_ping_configure(BMP390_SDK& baro) {
    bool init_ok = baro.init();
    ASSERT(init_ok, "init()");
    if (!init_ok) {
        BARO_LOG("      error: %s", baro.lastError());
        return;
    }

    bool ping_ok = baro.ping();
    ASSERT(ping_ok, "ping()");
    if (!ping_ok) {
        BARO_LOG("      error: %s", baro.lastError());
    }

    baro.configure(OsrPressure::x4, OsrTemp::x1, IIRFilter::OFF);
    ASSERT((baro.getStatus() & BMP390_STATUS_SPI_ERROR) == 0,
           "configure() — no crash");
}

static void test_extended_api(BMP390_SDK& baro) {
    delay_ms(25);

    bool triggered = baro.triggerConversion();
    ASSERT(triggered, "triggerConversion() returned true");
    if (!triggered) {
        BARO_LOG("      error: %s", baro.lastError());
        return;
    }

    uint32_t deadline = HAL_GetTick() + 50;
    bool ready = false;
    while (HAL_GetTick() < deadline) {
        if (baro.isConversionReady()) { ready = true; break; }
    }
    ASSERT(ready, "isConversionReady() — DRDY within timeout");

    BaroData d{};
    bool read_ok = baro.readConversion(d);
    ASSERT(read_ok, "readConversion()");
    if (read_ok) {
        BARO_LOG("      pressure=%.2f Pa, temp=%.2f C, ts=%llu us",
                 (double)d.pressure_pa, (double)d.temperature_c,
                 (unsigned long long)d.timestamp_us);
    } else {
        BARO_LOG("      error: %s", baro.lastError());
    }
}

static void test_getFrame_pattern(BMP390_SDK& baro) {
    delay_ms(25);

    baro.triggerMeasurement();
    delay_ms(15);

    BaroData d{};
    bool ok = baro.getFrame(d);
    ASSERT(ok, "getFrame() N/N+1 pattern");
    if (ok) {
        BARO_LOG("      pressure=%.2f Pa, temp=%.2f C",
                 (double)d.pressure_pa, (double)d.temperature_c);
    }
}

static void test_read_blocking(BMP390_SDK& baro) {
    delay_ms(25);

    BaroData d{};
    bool ok = baro.readBlocking(d, 50000);
    ASSERT(ok, "readBlocking()");
    if (ok) {
        BARO_LOG("      pressure=%.2f Pa, temp=%.2f C",
                 (double)d.pressure_pa, (double)d.temperature_c);
    } else {
        BARO_LOG("      error: %s", baro.lastError());
    }
}

static void test_rate_limit(BMP390_SDK& baro) {
    delay_ms(25);

    bool first  = baro.triggerConversion();
    bool second = baro.triggerConversion();

    ASSERT(first  == true,  "rate-limit: first trigger accepted");
    ASSERT(second == false, "rate-limit: second immediate trigger rejected");

    delay_ms(25);
    BaroData d{};
    baro.readConversion(d);
}

static void test_stats(BMP390_SDK& baro) {
    const BMP390Stats& s = baro.stats();
    BARO_LOG("      stats: triggers=%lu reads=%lu errors=%lu conv_us=%lu",
             (unsigned long)s.numTriggers,
             (unsigned long)s.numReads,
             (unsigned long)s.numErrors,
             (unsigned long)s.conversion_us_last);

    ASSERT(s.numTriggers >= 3, "stats: triggers >= 3");
    ASSERT(s.numReads    >= 3, "stats: reads >= 3");
    ASSERT(s.numErrors   == 0, "stats: no errors");
}

static void test_plausibility(BMP390_SDK& baro) {
    delay_ms(25);
    BaroData d{};
    baro.readBlocking(d, 50000);

    ASSERT(d.pressure_pa > 80000.0f && d.pressure_pa < 110000.0f,
           "pressure plausibility (80000–110000 Pa)");
    ASSERT(d.temperature_c > -40.0f && d.temperature_c < 85.0f,
           "temperature plausibility (-40–85 C)");
}

// ─────────────────────────────────────────────────────────────────────────────

void BMP390_ManualTest_Run() {
    BARO_LOG("===== BMP390 SDK Manual Test =====");

    BMP390_SDK::Config cfg{};
    cfg.hspi          = BMP390_TEST_HSPI;
    cfg.cs_port       = BMP390_TEST_CS_PORT;
    cfg.cs_pin        = BMP390_TEST_CS_PIN;
    cfg.commandRateHz = 50;

    BMP390_SDK baro(cfg);

    test_init_ping_configure(baro);

    if (!baro.isHealthy()) {
        BARO_LOG("Init failed — aborting remaining tests.");
        BARO_LOG("===== %d passed, %d failed =====", g_pass, g_fail);
        return;
    }

    test_extended_api(baro);
    test_getFrame_pattern(baro);
    test_read_blocking(baro);
    test_rate_limit(baro);
    test_stats(baro);
    test_plausibility(baro);

    BARO_LOG("===== %d passed, %d failed =====", g_pass, g_fail);
}
