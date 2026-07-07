#include "../TMP1075.hpp"
#include <cstring>
#include <cstdio>

namespace Drivers { namespace TMP1075 {

// ── Register map ──────────────────────────────────────────────────────────────

static constexpr uint8_t  REG_TEMP  = 0x00;  // Temperature (RO), 12-bit left-justified
static constexpr uint8_t  REG_CFGR  = 0x01;  // Configuration
static constexpr uint8_t  REG_LLIM  = 0x02;  // Low limit (ALERT)
static constexpr uint8_t  REG_HLIM  = 0x03;  // High limit (ALERT)
static constexpr uint8_t  REG_DIEID = 0x0F;  // Device ID — TMP1075 only, not TMP1075N

static constexpr uint16_t TMP1075_DIE_ID = 0x7500;

// CFGR bit positions within the high byte (bits 15:8 of the register).
// CFGR is accessed as a single byte: supported on TMP1075, required on
// TMP1075N (§7.5.1.2), and sidesteps the reserved low byte entirely.
static constexpr uint8_t CFG_OS    = (1u << 7);  // one-shot trigger, reads 0
static constexpr uint8_t CFG_R_POS = 5;          // conversion rate  (bits 6:5)
static constexpr uint8_t CFG_F_POS = 3;          // fault count      (bits 4:3)
static constexpr uint8_t CFG_POL   = (1u << 2);  // ALERT polarity
static constexpr uint8_t CFG_TM    = (1u << 1);  // ALERT mode
static constexpr uint8_t CFG_SD    = (1u << 0);  // shutdown

// R[1:0] is read-only on the TMP1075N (fixed 11) — skip it when verifying.
static constexpr uint8_t CFG_R_MSK = (0x03u << CFG_R_POS);

// One-shot conversion: 4.5–7 ms (datasheet §6.5, "ADC Conversion time,
// one-shot mode"). No DRDY bit exists, so gate reads on elapsed time:
// 7 ms worst case + margin. Note the 27.5 ms figure elsewhere in the
// datasheet is the continuous-mode cycle period, not the conversion itself.
static constexpr uint32_t ONE_SHOT_CONV_US = 10000;

static constexpr uint32_t I2C_TIMEOUT_MS = 10;

// ── DWT microsecond timer (same approach as the BMP390 driver) ────────────────

static void tmp1075_enable_dwt() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint64_t tmp1075_now_us() {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        return HAL_GetTick() * 1000ULL;
    }
    static uint32_t prev_cyc = 0;
    static uint64_t accum_us = 0;
    uint32_t cyc  = DWT->CYCCNT;
    accum_us += static_cast<uint64_t>(cyc - prev_cyc) * 1000000ULL / SystemCoreClock;
    prev_cyc  = cyc;
    return accum_us;
}

// ── Temperature encoding ──────────────────────────────────────────────────────
// TEMP/LLIM/HLIM hold a 12-bit two's-complement value in bits [15:4],
// 0.0625 °C per LSB. As a full signed 16-bit word that is 1/256 °C per count.

static float rawToCelsius(uint16_t raw) {
    return static_cast<float>(static_cast<int16_t>(raw)) / 256.0f;
}

static uint16_t celsiusToRaw(float c) {
    if (c >  127.9375f) c =  127.9375f;
    if (c < -128.0f)    c = -128.0f;
    int16_t v = static_cast<int16_t>(c * 256.0f);
    return static_cast<uint16_t>(v) & 0xFFF0u;
}

// ── Constructor ───────────────────────────────────────────────────────────────

TMP1075_Driver::TMP1075_Driver(const Config& cfg) : cfg_(cfg) {}

// ── Raw register access ───────────────────────────────────────────────────────

bool TMP1075_Driver::readReg(uint8_t reg, uint16_t& out) {
    uint8_t buf[2] = {};
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(
        cfg_.hi2c, static_cast<uint16_t>(cfg_.address7 << 1),
        reg, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS);

    if (st != HAL_OK) {
        statusFlags_ |= TMP1075_STATUS_I2C_ERROR;
        stats_.numErrors++;
        snprintf(lastErr_, sizeof(lastErr_), "i2c_rd reg=0x%02X HAL=%d err=0x%lX",
                 reg, (int)st, (unsigned long)cfg_.hi2c->ErrorCode);
        return false;
    }
    out = static_cast<uint16_t>((buf[0] << 8) | buf[1]);
    return true;
}

bool TMP1075_Driver::writeReg(uint8_t reg, uint16_t val) {
    uint8_t buf[2] = { static_cast<uint8_t>(val >> 8),
                       static_cast<uint8_t>(val & 0xFF) };
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(
        cfg_.hi2c, static_cast<uint16_t>(cfg_.address7 << 1),
        reg, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS);

    if (st != HAL_OK) {
        statusFlags_ |= TMP1075_STATUS_I2C_ERROR;
        stats_.numErrors++;
        snprintf(lastErr_, sizeof(lastErr_), "i2c_wr reg=0x%02X HAL=%d err=0x%lX",
                 reg, (int)st, (unsigned long)cfg_.hi2c->ErrorCode);
        return false;
    }
    return true;
}

bool TMP1075_Driver::readCfgr(uint8_t& out) {
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(
        cfg_.hi2c, static_cast<uint16_t>(cfg_.address7 << 1),
        REG_CFGR, I2C_MEMADD_SIZE_8BIT, &out, 1, I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        statusFlags_ |= TMP1075_STATUS_I2C_ERROR;
        stats_.numErrors++;
        snprintf(lastErr_, sizeof(lastErr_), "i2c_rd cfgr HAL=%d err=0x%lX",
                 (int)st, (unsigned long)cfg_.hi2c->ErrorCode);
        return false;
    }
    return true;
}

bool TMP1075_Driver::writeCfgr(uint8_t val) {
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(
        cfg_.hi2c, static_cast<uint16_t>(cfg_.address7 << 1),
        REG_CFGR, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        statusFlags_ |= TMP1075_STATUS_I2C_ERROR;
        stats_.numErrors++;
        snprintf(lastErr_, sizeof(lastErr_), "i2c_wr cfgr HAL=%d err=0x%lX",
                 (int)st, (unsigned long)cfg_.hi2c->ErrorCode);
        return false;
    }
    return true;
}

uint8_t TMP1075_Driver::buildConfig() const {
    uint8_t cfg = 0;
    cfg |= static_cast<uint8_t>(static_cast<uint8_t>(cfg_.rate)   << CFG_R_POS);
    cfg |= static_cast<uint8_t>(static_cast<uint8_t>(cfg_.faults) << CFG_F_POS);
    if (cfg_.polarity  == AlertPolarity::ActiveHigh) cfg |= CFG_POL;
    if (cfg_.alertMode == AlertMode::Interrupt)      cfg |= CFG_TM;
    if (cfg_.opMode    == OpMode::OneShot)           cfg |= CFG_SD;
    return cfg;
}

// ── init ──────────────────────────────────────────────────────────────────────

bool TMP1075_Driver::init() {
    tmp1075_enable_dwt();

    printf("[TMP1075] init: hi2c=%p addr7=0x%02X mode=%s\r\n",
           (void*)cfg_.hi2c, cfg_.address7,
           cfg_.opMode == OpMode::OneShot ? "one-shot" : "continuous");

    if (!cfg_.hi2c) {
        printf("[TMP1075] ERROR: hi2c is NULL!\r\n");
        snprintf(lastErr_, sizeof(lastErr_), "hi2c=NULL");
        return false;
    }

    if (!ping()) {
        printf("[TMP1075] INIT FAILED: %s\r\n", lastErr_);
        return false;
    }

    // Apply defaults from config
    configure(cfg_.rate, cfg_.faults, cfg_.polarity, cfg_.alertMode);
    if (statusFlags_ & TMP1075_STATUS_CONFIG_ERROR) {
        printf("[TMP1075] INIT FAILED: %s\r\n", lastErr_);
        return false;
    }

    // Verify CFGR readback. Mask out OS (reads 0) and, when the part may be
    // a TMP1075N (no device-ID check), also R[1:0] — read-only, fixed 11
    // (250 ms) on that variant.
    {
        uint8_t rb = 0;
        if (readCfgr(rb)) {
            uint8_t mask = static_cast<uint8_t>(~CFG_OS);
            if (!cfg_.checkDeviceId) mask &= static_cast<uint8_t>(~CFG_R_MSK);
            uint8_t expect = buildConfig() & mask;
            uint8_t got    = rb & mask;
            printf("[TMP1075] CFGR readback: 0x%02X (expect masked 0x%02X)\r\n", rb, expect);
            if (got != expect) {
                printf("[TMP1075] WARNING: CFGR mismatch, rewriting\r\n");
                writeCfgr(buildConfig());
            }
        }
    }

    healthy_ = true;
    printf("[TMP1075] init complete, healthy=true\r\n");
    return true;
}

// ── ping ──────────────────────────────────────────────────────────────────────

bool TMP1075_Driver::ping() {
    if (cfg_.checkDeviceId) {
        uint16_t id = 0;
        if (!readReg(REG_DIEID, id)) {
            // readReg() already populated lastErr_ with the I2C failure detail
            // (NACK/timeout) — don't clobber it with a whoami message.
            return false;
        }
        if (id != TMP1075_DIE_ID) {
            statusFlags_ |= TMP1075_STATUS_WHOAMI_MISMATCH;
            snprintf(lastErr_, sizeof(lastErr_), "whoami=0x%04X (expected 0x%04X)",
                     id, TMP1075_DIE_ID);
            return false;
        }
        return true;
    }
    // TMP1075N: no die-ID register — an ACKed temp read is our presence check.
    uint16_t dummy = 0;
    return readReg(REG_TEMP, dummy);
}

// ── configure ─────────────────────────────────────────────────────────────────

void TMP1075_Driver::configure(ConversionRate rate, FaultCount faults,
                               AlertPolarity pol, AlertMode alertMode) {
    cfg_.rate      = rate;
    cfg_.faults    = faults;
    cfg_.polarity  = pol;
    cfg_.alertMode = alertMode;

    if (!writeCfgr(buildConfig())) {
        statusFlags_ |= TMP1075_STATUS_CONFIG_ERROR;
        return;
    }
    statusFlags_ &= ~static_cast<uint32_t>(TMP1075_STATUS_CONFIG_ERROR);

    // Conversion / period bookkeeping.
    // One-shot: fixed ~27.5 ms conversion. Continuous: device free-runs at
    // the R[1:0] rate; convTimeUs_ tracks how stale a frame can be.
    static constexpr uint32_t kRateUs[4] = { 27500, 55000, 110000, 220000 };
    convTimeUs_ = (cfg_.opMode == OpMode::OneShot)
                    ? ONE_SHOT_CONV_US
                    : kRateUs[static_cast<uint8_t>(rate)];

    uint32_t conv_based = convTimeUs_ + 500u;
    uint32_t rate_based = (cfg_.commandRateHz > 0) ? (1000000UL / cfg_.commandRateHz) : 0u;
    periodUs_ = (rate_based > conv_based) ? rate_based : conv_based;
}

// ── TMP1075_Interface: triggerMeasurement / getFrame ──────────────────────────

void TMP1075_Driver::triggerMeasurement() {
    // Force-clear pending so a new trigger always succeeds — caller invokes
    // this only after its own timeout, meaning any prior conversion is stale.
    pending_ = false;
    triggerConversion();
}

bool TMP1075_Driver::getFrame(TempData& out) {
    if (!pending_) return false;

    if (!isConversionReady()) {
        statusFlags_ |= TMP1075_STATUS_NOT_READY;
        return false;
    }
    statusFlags_ &= ~static_cast<uint32_t>(TMP1075_STATUS_NOT_READY);

    return readConversion(out);
}

// ── Extended API ──────────────────────────────────────────────────────────────

bool TMP1075_Driver::triggerConversion() {
    if (pending_) return false;

    // Rate limiting
    if ((tmp1075_now_us() - triggerTs_us_) < periodUs_) return false;

    triggerTs_us_ = tmp1075_now_us();

    if (cfg_.opMode == OpMode::OneShot) {
        // Write the CFGR byte with OS=1 (SD=1 already included by
        // buildConfig() in one-shot mode). Full-byte write — no R-M-W hazard.
        // Caveat (§7.4.4): any write to the TM bit deactivates ALERT and
        // clears the fault count, so the ALERT fault queue is only really
        // meaningful in Continuous mode where CFGR isn't rewritten per sample.
        if (!writeCfgr(buildConfig() | CFG_OS)) {
            return false;
        }
    }
    // Continuous mode: nothing to write — device free-runs. The trigger only
    // stamps the timestamp so getFrame() timing/stats stay meaningful.

    pending_ = true;
    stats_.numTriggers++;
    return true;
}

bool TMP1075_Driver::isConversionReady() {
    if (!pending_) return false;

    if (cfg_.opMode == OpMode::Continuous) {
        return true;  // TEMP register always holds the latest conversion
    }
    // No DRDY bit on this part — time-gate the read.
    return (tmp1075_now_us() - triggerTs_us_) >= convTimeUs_;
}

bool TMP1075_Driver::readConversion(TempData& out) {
    if (!pending_) return false;

    uint16_t raw = 0;
    if (!readReg(REG_TEMP, raw)) {
        return false;
    }

    out.temperature_c = rawToCelsius(raw);
    out.timestamp_us  = triggerTs_us_;

    stats_.conversion_us_last = static_cast<uint32_t>(tmp1075_now_us() - triggerTs_us_);
    pending_ = false;
    stats_.numReads++;
    return true;
}

bool TMP1075_Driver::readBlocking(TempData& out, uint32_t timeout_us) {
    if (!triggerConversion()) return false;

    uint64_t start = tmp1075_now_us();
    while (!isConversionReady()) {
        if ((tmp1075_now_us() - start) > timeout_us) {
            pending_ = false;
            snprintf(lastErr_, sizeof(lastErr_), "timeout");
            return false;
        }
        // Short yield — DWT-based busy-wait (~100 µs)
        volatile uint32_t d = 100 * (SystemCoreClock / 1000000UL);
        while (d--) {}
    }
    return readConversion(out);
}

// ── ALERT limits / power control ──────────────────────────────────────────────

bool TMP1075_Driver::setLimits(float low_c, float high_c) {
    if (!writeReg(REG_LLIM, celsiusToRaw(low_c)))  return false;
    if (!writeReg(REG_HLIM, celsiusToRaw(high_c))) return false;
    return true;
}

bool TMP1075_Driver::shutdown() {
    return writeCfgr(buildConfig() | CFG_SD);
}

bool TMP1075_Driver::wake() {
    pending_ = false;
    return writeCfgr(buildConfig());
}

}} // namespace Drivers::TMP1075
