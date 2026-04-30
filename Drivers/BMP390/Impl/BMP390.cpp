#include "../BMP390.hpp"
#include <cstring>
#include <cstdio>

namespace Drivers { namespace BMP390 {

// ── Constructor ───────────────────────────────────────────────────────────────

BMP390_SDK::BMP390_SDK(const Config& cfg) : cfg_(cfg) {}

// ── init ──────────────────────────────────────────────────────────────────────

bool BMP390_SDK::init() {
    bmp3_enable_dwt();

    // Wire up the SDK device struct
    spiCtx_       = { cfg_.hspi, cfg_.cs_port, cfg_.cs_pin };
    dev_.intf     = BMP3_SPI_INTF;
    dev_.intf_ptr = &spiCtx_;
    dev_.read     = bmp3_spi_read;
    dev_.write    = bmp3_spi_write;
    dev_.delay_us = bmp3_delay_us_hal;

    // bmp3_init: validates chip-id, soft-resets, reads NVM calibration
    int8_t rs = bmp3_init(&dev_);
    if (rs != BMP3_OK) {
        uint8_t raw_id = 0;
        bmp3_get_regs(BMP3_REG_CHIP_ID, &raw_id, 1, &dev_);
        snprintf(lastErr_, sizeof(lastErr_), "bmp3_init=%d chip_id=0x%02X", (int)rs, raw_id);
        return false;
    }

    // Apply defaults from config
    configure(static_cast<OsrPressure>(cfg_.osr_p),
              static_cast<OsrTemp>(cfg_.osr_t),
              static_cast<IIRFilter>(cfg_.iir));

    healthy_ = true;
    return true;
}

// ── ping ──────────────────────────────────────────────────────────────────────

bool BMP390_SDK::ping() {
    uint8_t chip_id = 0;
    int8_t rs = bmp3_get_regs(BMP3_REG_CHIP_ID, &chip_id, 1, &dev_);
    if (rs != BMP3_OK || chip_id != BMP390_CHIP_ID) {
        statusFlags_ |= BMP390_STATUS_WHOAMI_MISMATCH;
        snprintf(lastErr_, sizeof(lastErr_), "whoami=0x%02X (expected 0x%02X)",
                 chip_id, BMP390_CHIP_ID);
        return false;
    }
    return true;
}

// ── configure ─────────────────────────────────────────────────────────────────
// OsrPressure / OsrTemp enum values (0–5) map directly to BMP3_OVERSAMPLING_*X.
// IIRFilter enum values (0–6) map directly to BMP3_IIR_FILTER_DISABLE / COEFF_*.

void BMP390_SDK::configure(OsrPressure osr_p, OsrTemp osr_t, IIRFilter filter) {
    devSettings_.press_en = BMP3_ENABLE;
    devSettings_.temp_en  = BMP3_ENABLE;
    devSettings_.odr_filter.press_os   = static_cast<uint8_t>(osr_p);
    devSettings_.odr_filter.temp_os    = static_cast<uint8_t>(osr_t);
    devSettings_.odr_filter.iir_filter = static_cast<uint8_t>(filter);

    uint32_t sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                   BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_IIR_FILTER;

    int8_t rs = bmp3_set_sensor_settings(sel, &devSettings_, &dev_);
    if (rs != BMP3_OK) {
        snprintf(lastErr_, sizeof(lastErr_), "set_settings=%d", (int)rs);
        statusFlags_ |= BMP390_STATUS_SPI_ERROR;
        stats_.numErrors++;
        return;
    }

    // Recompute conversion time from datasheet §3.9.2:
    //   T_conv [µs] = 234 + (392 + 2^osr_p × 2020) + (163 + 2^osr_t × 2020)
    uint32_t osr_p_mult = 1u << static_cast<uint8_t>(osr_p);
    uint32_t osr_t_mult = 1u << static_cast<uint8_t>(osr_t);
    convTimeUs_ = 234u + (392u + osr_p_mult * 2020u) + (163u + osr_t_mult * 2020u);

    // Rate limiter: must be at least convTimeUs_ so we never trigger into an
    // ongoing conversion. commandRateHz acts as an optional slower cap (e.g. to
    // match a slow main loop), but can never push periodUs_ below convTimeUs_.
    uint32_t conv_based = convTimeUs_ + 500u;
    uint32_t rate_based = (cfg_.commandRateHz > 0) ? (1000000UL / cfg_.commandRateHz) : 0u;
    periodUs_ = (rate_based > conv_based) ? rate_based : conv_based;
}

// ── BMP390_Interface: triggerMeasurement / getFrame ───────────────────────────

void BMP390_SDK::triggerMeasurement() {
    triggerConversion(); // ignores rate-limit return value — matches raw driver behaviour
}

bool BMP390_SDK::getFrame(BaroData& out) {
    if (!pending_) return false;

    if (!isConversionReady()) {
        statusFlags_ |= BMP390_STATUS_NOT_READY;
        return false;
    }
    statusFlags_ &= ~static_cast<uint32_t>(BMP390_STATUS_NOT_READY);

    return readConversion(out);
}

// ── Extended API ──────────────────────────────────────────────────────────────

bool BMP390_SDK::triggerConversion() {
    if (pending_) return false;

    // Rate limiting
    if ((bmp3_now_us() - triggerTs_us_) < periodUs_) return false;

    triggerTs_us_ = bmp3_now_us();

    devSettings_.op_mode = BMP3_MODE_FORCED;
    int8_t rs = bmp3_set_op_mode(&devSettings_, &dev_);
    if (rs != BMP3_OK) {
        snprintf(lastErr_, sizeof(lastErr_), "set_op_mode=%d", (int)rs);
        statusFlags_ |= BMP390_STATUS_SPI_ERROR;
        stats_.numErrors++;
        return false;
    }

    pending_ = true;
    stats_.numTriggers++;
    return true;
}

bool BMP390_SDK::isConversionReady() {
    if (!pending_) return false;

    bmp3_status status{};
    int8_t rs = bmp3_get_status(&status, &dev_);
    if (rs != BMP3_OK) return false;

    return status.intr.drdy == BMP3_ENABLE;
}

bool BMP390_SDK::readConversion(BaroData& out) {
    if (!pending_) return false;

    bmp3_data data{};
    int8_t rs = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev_);
    if (rs != BMP3_OK) {
        snprintf(lastErr_, sizeof(lastErr_), "get_sensor_data=%d", (int)rs);
        statusFlags_ |= BMP390_STATUS_SPI_ERROR;
        stats_.numErrors++;
        return false;
    }

    out.pressure_pa   = static_cast<float>(data.pressure);
    out.temperature_c = static_cast<float>(data.temperature);
    out.timestamp_us  = triggerTs_us_;

    stats_.conversion_us_last = static_cast<uint32_t>(bmp3_now_us() - triggerTs_us_);
    pending_ = false;
    stats_.numReads++;
    return true;
}

bool BMP390_SDK::readBlocking(BaroData& out, uint32_t timeout_us) {
    if (!triggerConversion()) return false;

    uint64_t start = bmp3_now_us();
    while (!isConversionReady()) {
        if ((bmp3_now_us() - start) > timeout_us) {
            pending_ = false;
            snprintf(lastErr_, sizeof(lastErr_), "timeout");
            return false;
        }
        // Short yield — DWT-based busy-wait
        volatile uint32_t d = 100 * (SystemCoreClock / 1000000UL);
        while (d--) {}
    }
    return readConversion(out);
}

}} // namespace Drivers::BMP390
