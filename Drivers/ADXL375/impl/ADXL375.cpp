#include "../ADXL375.hpp"
#include <cstring>
#include <cstdio>

namespace Drivers { namespace ADXL375 {

// ── Register map (ADXL345/ADXL375-compatible) ──────────────────────────────────

static constexpr uint8_t REG_DEVID       = 0x00;  // fixed 0xE5, shared with ADXL345
static constexpr uint8_t REG_OFSX        = 0x1E;  // OFSX/OFSY/OFSZ, offset trim
static constexpr uint8_t REG_OFSY        = 0x1F;
static constexpr uint8_t REG_OFSZ        = 0x20;
static constexpr uint8_t REG_BW_RATE     = 0x2C;
static constexpr uint8_t REG_POWER_CTL   = 0x2D;
static constexpr uint8_t REG_INT_SOURCE  = 0x30;
static constexpr uint8_t REG_DATA_FORMAT = 0x31;
static constexpr uint8_t REG_DATAX0      = 0x32;  // DATAX0..DATAZ1, 6-byte burst
static constexpr uint8_t REG_FIFO_CTL    = 0x38;

static constexpr uint8_t ADXL375_DEVICE_ID = 0xE5;

static constexpr uint8_t POWER_CTL_MEASURE     = (1u << 3);
static constexpr uint8_t INT_SOURCE_DATA_READY = (1u << 7);

// DATA_FORMAT (Register 0x31): unlike the ADXL345, D3/D1/D0 are NOT feature
// bits on the ADXL375 — the datasheet's own bit table lists them as fixed
// values (1, 1, 1) that must be written as-is. Only SELF_TEST(D7)/SPI(D6)/
// INT_INVERT(D5)/Justify(D2) are actually configurable.
static constexpr uint8_t DATA_FORMAT_DEFAULT = 0x0B;  // 0b0000'1011: D3=1, Justify=0 (right-justified), D1=1, D0=1

// ADXL375 has one fixed ±200 g range: 20.5 LSB/g typ, i.e. ~49 mg/LSB
// (datasheet Table 1, Sensitivity/Scale Factor).
static constexpr float SCALE_G_PER_LSB = 0.049f;

// OFSX/OFSY/OFSZ (Registers 0x1E-0x20): the Rev B datasheet contradicts
// itself on this constant — the Register Descriptions section (explicitly
// flagged as revised in the Rev A->B changelog) states 0.196 g/LSB, while
// the separate Offset Calibration application note states 1.56 g/LSB (~8x
// off). 0.196 g/LSB is used here: it's the value in the section ADI actually
// revised for this register, and it gives fine enough resolution to null a
// typical sub-1g bias (1.56 g/LSB could only correct in ~1.56 g steps).
static constexpr float OFFSET_SCALE_G_PER_LSB = 0.196f;

static constexpr uint32_t I2C_TIMEOUT_MS = 10;

// ── DWT microsecond timer (same approach as the TMP1075/BMP390 drivers) ───────

static void adxl375_enable_dwt() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // CYCCNT is a single MCU-wide counter shared with other drivers (TMP1075,
    // BMP390, ...) — only reset it if nothing has started it yet, otherwise
    // resetting mid-flight corrupts every other driver's elapsed-time math.
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        DWT->CYCCNT = 0;
        DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static uint64_t adxl375_now_us() {
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

// ── Output data rate → sample period lookup (datasheet Table 8) ───────────────

static uint32_t odrToPeriodUs(OutputDataRate rate) {
    switch (rate) {
        case OutputDataRate::hz12_5: return 80000;
        case OutputDataRate::hz25:   return 40000;
        case OutputDataRate::hz50:   return 20000;
        case OutputDataRate::hz100:  return 10000;
        case OutputDataRate::hz200:  return 5000;
        case OutputDataRate::hz400:  return 2500;
        case OutputDataRate::hz800:  return 1250;
        case OutputDataRate::hz1600: return 625;
        case OutputDataRate::hz3200: return 313;
    }
    return 10000;
}

// ── Constructor ───────────────────────────────────────────────────────────────

ADXL375_Driver::ADXL375_Driver(const Config& cfg) : cfg_(cfg) {}

// ── Raw register access ───────────────────────────────────────────────────────

bool ADXL375_Driver::readReg(uint8_t reg, uint8_t& out) {
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(
        cfg_.hi2c, static_cast<uint16_t>(cfg_.address7 << 1),
        reg, I2C_MEMADD_SIZE_8BIT, &out, 1, I2C_TIMEOUT_MS);

    if (st != HAL_OK) {
        statusFlags_ |= ADXL375_STATUS_I2C_ERROR;
        stats_.numErrors++;
        snprintf(lastErr_, sizeof(lastErr_), "i2c_rd reg=0x%02X HAL=%d err=0x%lX",
                 reg, (int)st, (unsigned long)cfg_.hi2c->ErrorCode);
        return false;
    }
    return true;
}

bool ADXL375_Driver::writeReg(uint8_t reg, uint8_t val) {
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(
        cfg_.hi2c, static_cast<uint16_t>(cfg_.address7 << 1),
        reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS);

    if (st != HAL_OK) {
        statusFlags_ |= ADXL375_STATUS_I2C_ERROR;
        stats_.numErrors++;
        snprintf(lastErr_, sizeof(lastErr_), "i2c_wr reg=0x%02X HAL=%d err=0x%lX",
                 reg, (int)st, (unsigned long)cfg_.hi2c->ErrorCode);
        return false;
    }
    return true;
}

bool ADXL375_Driver::readAxes(int16_t& x, int16_t& y, int16_t& z) {
    uint8_t buf[6] = {};
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(
        cfg_.hi2c, static_cast<uint16_t>(cfg_.address7 << 1),
        REG_DATAX0, I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf), I2C_TIMEOUT_MS);

    if (st != HAL_OK) {
        statusFlags_ |= ADXL375_STATUS_I2C_ERROR;
        stats_.numErrors++;
        snprintf(lastErr_, sizeof(lastErr_), "i2c_rd axes HAL=%d err=0x%lX",
                 (int)st, (unsigned long)cfg_.hi2c->ErrorCode);
        return false;
    }
    // DATAx0 = LSB, DATAx1 = MSB — little-endian, already a 16-bit sign-extended
    // two's-complement value (datasheet §Register 0x32–0x37).
    x = static_cast<int16_t>(static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8));
    y = static_cast<int16_t>(static_cast<uint16_t>(buf[2]) | (static_cast<uint16_t>(buf[3]) << 8));
    z = static_cast<int16_t>(static_cast<uint16_t>(buf[4]) | (static_cast<uint16_t>(buf[5]) << 8));
    return true;
}

// ── init ──────────────────────────────────────────────────────────────────────

bool ADXL375_Driver::init() {
    adxl375_enable_dwt();

    printf("[ADXL375] init: hi2c=%p addr7=0x%02X\r\n", (void*)cfg_.hi2c, cfg_.address7);

    if (!cfg_.hi2c) {
        printf("[ADXL375] ERROR: hi2c is NULL!\r\n");
        snprintf(lastErr_, sizeof(lastErr_), "hi2c=NULL");
        return false;
    }

    if (!ping()) {
        printf("[ADXL375] INIT FAILED: %s\r\n", lastErr_);
        return false;
    }

    // Bypass FIFO — the driver polls DATA_READY per-sample, no queuing needed.
    if (!writeReg(REG_FIFO_CTL, 0x00)) {
        statusFlags_ |= ADXL375_STATUS_CONFIG_ERROR;
        printf("[ADXL375] INIT FAILED: %s\r\n", lastErr_);
        return false;
    }

    // Right-justified, sign-extended, with the mandatory reserved bits set
    // per the datasheet's DATA_FORMAT bit table.
    if (!writeReg(REG_DATA_FORMAT, DATA_FORMAT_DEFAULT)) {
        statusFlags_ |= ADXL375_STATUS_CONFIG_ERROR;
        printf("[ADXL375] INIT FAILED: %s\r\n", lastErr_);
        return false;
    }

    configure(cfg_.rate);
    if (statusFlags_ & ADXL375_STATUS_CONFIG_ERROR) {
        printf("[ADXL375] INIT FAILED: %s\r\n", lastErr_);
        return false;
    }

    // Enter measurement mode (POR default is standby).
    if (!writeReg(REG_POWER_CTL, POWER_CTL_MEASURE)) {
        statusFlags_ |= ADXL375_STATUS_CONFIG_ERROR;
        printf("[ADXL375] INIT FAILED: %s\r\n", lastErr_);
        return false;
    }

    healthy_ = true;
    printf("[ADXL375] init complete, healthy=true\r\n");
    return true;
}

// ── ping ──────────────────────────────────────────────────────────────────────

bool ADXL375_Driver::ping() {
    uint8_t id = 0;
    if (!readReg(REG_DEVID, id)) {
        // readReg() already populated lastErr_ with the I2C failure detail.
        return false;
    }
    if (id != ADXL375_DEVICE_ID) {
        statusFlags_ |= ADXL375_STATUS_WHOAMI_MISMATCH;
        snprintf(lastErr_, sizeof(lastErr_), "whoami=0x%02X (expected 0x%02X)",
                 id, ADXL375_DEVICE_ID);
        return false;
    }
    return true;
}

// ── configure ─────────────────────────────────────────────────────────────────

void ADXL375_Driver::configure(OutputDataRate rate) {
    cfg_.rate = rate;

    if (!writeReg(REG_BW_RATE, static_cast<uint8_t>(rate))) {
        statusFlags_ |= ADXL375_STATUS_CONFIG_ERROR;
        return;
    }
    statusFlags_ &= ~static_cast<uint32_t>(ADXL375_STATUS_CONFIG_ERROR);

    convTimeUs_ = odrToPeriodUs(rate);

    uint32_t conv_based = convTimeUs_ + 500u;
    uint32_t rate_based = (cfg_.commandRateHz > 0) ? (1000000UL / cfg_.commandRateHz) : 0u;
    periodUs_ = (rate_based > conv_based) ? rate_based : conv_based;
}

// ── ADXL375_Interface: triggerMeasurement / getFrame ──────────────────────────

void ADXL375_Driver::triggerMeasurement() {
    pending_ = false;
    triggerConversion();
}

bool ADXL375_Driver::getFrame(AccelData& out) {
    if (!pending_) return false;

    if (!isConversionReady()) {
        statusFlags_ |= ADXL375_STATUS_NOT_READY;
        return false;
    }
    statusFlags_ &= ~static_cast<uint32_t>(ADXL375_STATUS_NOT_READY);

    return readConversion(out);
}

// ── Extended API ──────────────────────────────────────────────────────────────

bool ADXL375_Driver::triggerConversion() {
    if (pending_) return false;

    // Rate limiting
    if ((adxl375_now_us() - triggerTs_us_) < periodUs_) return false;

    triggerTs_us_ = adxl375_now_us();

    // Device free-runs once in measurement mode — nothing to write here, the
    // trigger only stamps the timestamp so getFrame() timing/stats stay
    // meaningful (mirrors TMP1075's Continuous-mode behaviour).
    pending_ = true;
    stats_.numTriggers++;
    return true;
}

bool ADXL375_Driver::isConversionReady() {
    if (!pending_) return false;

    uint8_t src = 0;
    if (!readReg(REG_INT_SOURCE, src)) return false;
    return (src & INT_SOURCE_DATA_READY) != 0;
}

bool ADXL375_Driver::readConversion(AccelData& out) {
    if (!pending_) return false;

    int16_t rawX = 0, rawY = 0, rawZ = 0;
    if (!readAxes(rawX, rawY, rawZ)) {
        return false;
    }

    out.x_g = static_cast<float>(rawX) * SCALE_G_PER_LSB;
    out.y_g = static_cast<float>(rawY) * SCALE_G_PER_LSB;
    out.z_g = static_cast<float>(rawZ) * SCALE_G_PER_LSB;
    out.timestamp_us = triggerTs_us_;

    stats_.conversion_us_last = static_cast<uint32_t>(adxl375_now_us() - triggerTs_us_);
    pending_ = false;
    stats_.numReads++;
    return true;
}

bool ADXL375_Driver::readBlocking(AccelData& out, uint32_t timeout_us) {
    if (!triggerConversion()) {
        snprintf(lastErr_, sizeof(lastErr_), "trigger rejected (pending or rate-limited)");
        return false;
    }

    uint64_t start = adxl375_now_us();
    while (!isConversionReady()) {
        if ((adxl375_now_us() - start) > timeout_us) {
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

// ── Power control ──────────────────────────────────────────────────────────────

bool ADXL375_Driver::shutdown() {
    return writeReg(REG_POWER_CTL, 0x00);
}

bool ADXL375_Driver::wake() {
    pending_ = false;
    return writeReg(REG_POWER_CTL, POWER_CTL_MEASURE);
}

// ── Offset calibration ──────────────────────────────────────────────────────

bool ADXL375_Driver::calibrate(float refX_g, float refY_g, float refZ_g, uint8_t numSamples) {
    if (numSamples == 0) numSamples = 1;

    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    uint8_t collected = 0;
    uint8_t attempts = 0;
    const uint8_t maxAttempts = static_cast<uint8_t>(numSamples * 4u + 10u);

    while (collected < numSamples && attempts < maxAttempts) {
        attempts++;
        AccelData d{};
        if (!readBlocking(d)) {
            // A failed attempt is almost always the rate-limit gate rejecting
            // an instant retry — without a delay here, all maxAttempts burn
            // in a fraction of a millisecond, long before periodUs_ elapses.
            HAL_Delay(15);
            continue;
        }
        sumX += d.x_g;
        sumY += d.y_g;
        sumZ += d.z_g;
        collected++;
    }

    if (collected == 0) {
        snprintf(lastErr_, sizeof(lastErr_), "calibrate: no samples collected");
        return false;
    }

    float measX = sumX / collected;
    float measY = sumY / collected;
    float measZ = sumZ / collected;

    auto toCode = [](float offset_g) -> int8_t {
        float raw = offset_g / OFFSET_SCALE_G_PER_LSB;
        if (raw > 127.0f)  raw = 127.0f;
        if (raw < -128.0f) raw = -128.0f;
        return static_cast<int8_t>(raw >= 0.0f ? (raw + 0.5f) : (raw - 0.5f));
    };

    int8_t codeX = toCode(refX_g - measX);
    int8_t codeY = toCode(refY_g - measY);
    int8_t codeZ = toCode(refZ_g - measZ);

    bool ok = writeReg(REG_OFSX, static_cast<uint8_t>(codeX))
           && writeReg(REG_OFSY, static_cast<uint8_t>(codeY))
           && writeReg(REG_OFSZ, static_cast<uint8_t>(codeZ));

    printf("[ADXL375] calibrate: measured=(%.3f,%.3f,%.3f)g target=(%.3f,%.3f,%.3f)g "
           "samples=%u/%u -> OFS=(%d,%d,%d) %s\r\n",
           (double)measX, (double)measY, (double)measZ,
           (double)refX_g, (double)refY_g, (double)refZ_g,
           (unsigned)collected, (unsigned)numSamples,
           (int)codeX, (int)codeY, (int)codeZ,
           ok ? "OK" : "FAILED");

    return ok;
}

}} // namespace Drivers::ADXL375
