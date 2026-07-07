#pragma once

#include "TMP1075.h"
#include "stm32h7xx_hal.h"
#include <cstdint>

namespace Drivers {
namespace TMP1075 {

// ── Diagnostics ───────────────────────────────────────────────────────────────

struct TMP1075Stats {
    uint32_t numTriggers;         // conversions triggered
    uint32_t numReads;            // successful data reads
    uint32_t numErrors;           // I2C / config errors
    uint32_t conversion_us_last;  // measured time from trigger → read
};

// ── TMP1075_Driver ────────────────────────────────────────────────────────────
// Concrete implementation of TMP1075_Interface. Talks to the sensor directly
// over HAL I2C (no vendor SDK needed — 5-register device).
//
// Usage (one-shot mode — N/N+1 pattern):
//   temp.triggerMeasurement();   // iteration N   → starts one-shot conversion
//   temp.getFrame(data);         // iteration N+1 → reads result (time-gated)

class TMP1075_Driver : public TMP1075_Interface {
public:
    struct Config {
        I2C_HandleTypeDef* hi2c     = nullptr;
        uint8_t            address7 = 0x48;   // 7-bit address. TMP1075: 32 options in
                                              // 0x40–0x5F (A2..A0 to GND/V+/SDA/SCL,
                                              // datasheet Table 7-2). TMP1075N: 0x48–0x4B.

        // Sensor defaults — can be overridden before init(), or via configure()
        OpMode         opMode        = OpMode::OneShot;
        ConversionRate rate          = ConversionRate::ms27_5;  // continuous mode only
        FaultCount     faults        = FaultCount::f1;
        AlertPolarity  polarity      = AlertPolarity::ActiveLow;
        AlertMode      alertMode     = AlertMode::Comparator;
        uint16_t       commandRateHz = 10;

        // TMP1075N has no die-ID register — set false for the N variant.
        bool checkDeviceId = true;
    };

    explicit TMP1075_Driver(const Config& cfg);

    // ── TMP1075_Interface ─────────────────────────────────────────────────────
    bool     init()                                                   override;
    bool     ping()                                                   override;
    void     configure(ConversionRate rate, FaultCount faults,
                       AlertPolarity pol, AlertMode alertMode)        override;
    void     triggerMeasurement()                                     override;
    bool     getFrame(TempData& out)                                  override;
    uint32_t getStatus() const                                        override { return statusFlags_; }

    // ── Extended API ──────────────────────────────────────────────────────────
    bool isHealthy()               const { return healthy_; }
    const char*          lastError() const { return lastErr_; }
    const TMP1075Stats&  stats()     const { return stats_; }

    /// Start a one-shot conversion. Returns false if already pending or rate-limited.
    /// In Continuous mode this only stamps the timestamp (device free-runs).
    bool triggerConversion();

    /// Time-based readiness check (no DRDY bit on this part). Non-blocking.
    bool isConversionReady();

    /// Read the temperature register. Timestamp is set to trigger time.
    bool readConversion(TempData& out);

    /// Blocking convenience: trigger → wait → read. Timeout in microseconds.
    /// One-shot conversion is 4.5–7 ms (datasheet §6.5), so 25 ms is generous.
    bool readBlocking(TempData& out, uint32_t timeout_us = 25000);

    /// ALERT pin thresholds in °C (LLIM / HLIM registers).
    bool setLimits(float low_c, float high_c);

    /// Enter / leave shutdown (SD bit). wake() restores the configured OpMode.
    bool shutdown();
    bool wake();

    uint64_t lastTriggerUs() const { return triggerTs_us_; }

private:
    // Raw register access (16-bit big-endian over I2C) — TEMP/LLIM/HLIM/DIEID
    bool readReg (uint8_t reg, uint16_t& out);
    bool writeReg(uint8_t reg, uint16_t val);

    // CFGR is accessed as a single byte (bits 15:8). Single-byte access is
    // supported on the TMP1075 and *required* on the TMP1075N (§7.5.1.2).
    bool readCfgr (uint8_t& out);
    bool writeCfgr(uint8_t val);

    /// Build the CFGR high byte from cached settings (without the OS bit).
    uint8_t buildConfig() const;

    Config cfg_;

    bool     healthy_      = false;
    bool     pending_      = false;
    uint64_t triggerTs_us_ = 0;
    uint32_t convTimeUs_   = 10000;  // one-shot conversion time incl. margin
    uint32_t periodUs_     = 100000; // min period between triggers

    uint32_t     statusFlags_ = TMP1075_STATUS_OK;
    TMP1075Stats stats_{};
    char         lastErr_[64] = {};
};

} // namespace TMP1075
} // namespace Drivers
