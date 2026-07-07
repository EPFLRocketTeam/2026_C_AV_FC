#pragma once

#include "ADXL375.h"
#include "stm32h7xx_hal.h"
#include <cstdint>

namespace Drivers {
namespace ADXL375 {

// ── Diagnostics ───────────────────────────────────────────────────────────────

struct ADXL375Stats {
    uint32_t numTriggers;         // read windows armed
    uint32_t numReads;            // successful data reads
    uint32_t numErrors;           // I2C errors
    uint32_t conversion_us_last;  // measured time from trigger → read
};

// ── ADXL375_Driver ────────────────────────────────────────────────────────────
// Concrete implementation of ADXL375_Interface. Talks to the sensor directly
// over HAL I2C (no vendor SDK needed — register map is ADXL345-compatible).
//
// Usage:
//   accel.triggerMeasurement();  // iteration N   → arms the read window
//   accel.getFrame(data);        // iteration N+1 → reads once DATA_READY is set

class ADXL375_Driver : public ADXL375_Interface {
public:
    struct Config {
        I2C_HandleTypeDef* hi2c     = nullptr;
        uint8_t            address7 = 0x1D;   // 0x1D (ALT ADDRESS high) or 0x53 (low)

        // Sensor defaults — can be overridden before init(), or via configure()
        OutputDataRate rate          = OutputDataRate::hz100;
        uint16_t       commandRateHz = 10;
    };

    explicit ADXL375_Driver(const Config& cfg);

    // ── ADXL375_Interface ─────────────────────────────────────────────────────
    bool     init()                          override;
    bool     ping()                          override;
    void     configure(OutputDataRate rate)  override;
    void     triggerMeasurement()            override;
    bool     getFrame(AccelData& out)        override;
    uint32_t getStatus() const               override { return statusFlags_; }

    // ── Extended API ──────────────────────────────────────────────────────────
    bool isHealthy()              const { return healthy_; }
    const char*          lastError() const { return lastErr_; }
    const ADXL375Stats&  stats()     const { return stats_; }

    /// Arm a read window. Returns false if already pending or rate-limited.
    /// The device itself free-runs — this only stamps the timestamp used for
    /// getFrame() timing/stats.
    bool triggerConversion();

    /// Poll the DATA_READY status bit (INT_SOURCE). Non-blocking.
    bool isConversionReady();

    /// Read DATAX/Y/Z. Timestamp is set to trigger time.
    bool readConversion(AccelData& out);

    /// Blocking convenience: trigger → wait → read. Timeout in microseconds.
    bool readBlocking(AccelData& out, uint32_t timeout_us = 20000);

    /// Enter / leave standby (Measure bit). wake() restores the configured rate.
    bool shutdown();
    bool wake();

    /// Zero out the sensor's static offset by writing OFSX/OFSY/OFSZ so that,
    /// under the conditions present *right now*, each axis reads (refX_g,
    /// refY_g, refZ_g). Defaults to (0, 0, +1) — the standard flat/Z-up
    /// convention — but the sensor's actual mounting orientation determines
    /// what "flat" means, so pass explicit values if that assumption is
    /// wrong (e.g. Z-down, or a known tilt).
    ///
    /// Averages numSamples readings (datasheet-recommended ~10 samples at
    /// ODR >= 100 Hz) before computing the correction. The offset registers
    /// do not retain their value across a power cycle (reset to 0x00), so
    /// this must be re-run on every boot.
    bool calibrate(float refX_g = 0.0f, float refY_g = 0.0f, float refZ_g = 1.0f,
                   uint8_t numSamples = 10);

    uint64_t lastTriggerUs() const { return triggerTs_us_; }

private:
    // Raw register access — DEVID/BW_RATE/POWER_CTL/etc are single bytes;
    // DATAX0..DATAZ1 is a 6-byte burst (auto-incrementing on I2C).
    bool readReg (uint8_t reg, uint8_t& out);
    bool writeReg(uint8_t reg, uint8_t val);
    bool readAxes(int16_t& x, int16_t& y, int16_t& z);

    Config cfg_;

    bool     healthy_      = false;
    bool     pending_      = false;
    uint64_t triggerTs_us_ = 0;
    uint32_t convTimeUs_   = 10000; // derived from configured ODR, updated by configure()
    uint32_t periodUs_     = 100000; // min period between triggers

    uint32_t     statusFlags_ = ADXL375_STATUS_OK;
    ADXL375Stats stats_{};
    char         lastErr_[64] = {};
};

} // namespace ADXL375
} // namespace Drivers
