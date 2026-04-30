#pragma once

#include "BMP390.h"
#include "SDK/BMP390_config.h"
#include "SDK/BMP390_transport_stm32.h"
#include "bmp3.h"
#include <cstdint>

namespace Drivers {
namespace BMP390 {

// ── Diagnostics ───────────────────────────────────────────────────────────────

struct BMP390Stats {
    uint32_t numTriggers;         // conversions triggered
    uint32_t numReads;            // successful data reads
    uint32_t numErrors;           // SDK / SPI errors
    uint32_t conversion_us_last;  // measured time from trigger → read
};

// ── BMP390_SDK ────────────────────────────────────────────────────────────────
// Concrete implementation of BMP390_Interface. Uses Bosch BMP3_SensorAPI (bmp3.h)
// for calibration, compensation, and sensor control.
//
// Usage (forced mode — N/N+1 pattern):
//   baro.triggerMeasurement();   // iteration N   → starts forced-mode conversion
//   baro.getFrame(data);         // iteration N+1 → reads result (checks drdy first)

class BMP390_SDK : public BMP390_Interface {
public:
    struct Config {
        SPI_HandleTypeDef* hspi;
        GPIO_TypeDef*      cs_port;
        uint16_t           cs_pin;

        // Sensor defaults — can be overridden before calling init(), or via configure()
        uint8_t  osr_p         = BMP390_DEFAULT_OSR_P;
        uint8_t  osr_t         = BMP390_DEFAULT_OSR_T;
        uint8_t  iir           = BMP390_DEFAULT_IIR;
        uint16_t commandRateHz = BMP390_DEFAULT_COMMAND_RATE_HZ;
    };

    explicit BMP390_SDK(const Config& cfg);

    // ── BMP390_Interface ──────────────────────────────────────────────────────
    bool     init()                                                   override;
    bool     ping()                                                   override;
    void     configure(OsrPressure osr_p, OsrTemp osr_t,
                       IIRFilter filter)                              override;
    void     triggerMeasurement()                                     override;
    bool     getFrame(BaroData& out)                                  override;
    uint32_t getStatus() const                                        override { return statusFlags_; }

    // ── Extended API ──────────────────────────────────────────────────────────
    bool isHealthy()           const { return healthy_; }
    const char*        lastError() const { return lastErr_; }
    const BMP390Stats& stats()     const { return stats_; }

    /// Start a forced-mode conversion. Returns false if already pending or rate-limited.
    bool triggerConversion();

    /// Poll the DRDY status bit. Non-blocking.
    bool isConversionReady();

    /// Read the last completed conversion. Timestamp is set to trigger time.
    bool readConversion(BaroData& out);

    /// Blocking convenience: trigger → wait → read. Timeout in microseconds.
    bool readBlocking(BaroData& out, uint32_t timeout_us = 20000);

    uint64_t lastTriggerUs() const { return triggerTs_us_; }

private:
    Config        cfg_;
    bmp3_dev      dev_{};
    Bmp3SpiCtx    spiCtx_{};
    bmp3_settings devSettings_{};    // cached so configure() can accumulate changes

    bool     healthy_      = false;
    bool     pending_      = false;
    uint64_t triggerTs_us_ = 0;
    uint32_t convTimeUs_   = 6000;   // estimated conversion time, updated by configure()
    uint32_t periodUs_     = 20000;  // min period between triggers

    uint32_t    statusFlags_ = BMP390_STATUS_OK;
    BMP390Stats stats_{};
    char        lastErr_[64] = {};
};

} // namespace BMP390
} // namespace Drivers
