#pragma once

#include <cstdint>

/**
 * @brief  Driver Interface for TI TMP1075 Digital Temperature Sensor (I2C)
 * @target STM32H7 (Flight Computer)
 *
 * Usage (main loop, one-shot mode — N/N+1 pattern):
 *   temp.triggerMeasurement();   // iteration N   → starts one-shot conversion
 *   temp.getFrame(data);         // iteration N+1 → reads result (time-gated)
 *
 * Note: the TMP1075 has no data-ready status bit. Readiness is time-based
 * (~27.5 ms typical conversion). In Continuous mode the temperature register
 * always holds the latest conversion and getFrame() reads immediately.
 */

namespace Drivers {
namespace TMP1075 {

// Conversion rate in continuous mode (CFGR.R[1:0]).
// In one-shot mode a single conversion always takes ~27.5 ms.
enum class ConversionRate : uint8_t {
    ms27_5 = 0x00,   // 27.5 ms  (~36 Hz)
    ms55   = 0x01,   // 55 ms
    ms110  = 0x02,   // 110 ms
    ms220  = 0x03    // 220 ms (default)
};

// Consecutive fault count before ALERT asserts (CFGR.F[1:0]).
// Datasheet Table 7-8: 1/2/3/4 faults on TMP1075; codes 10/11 mean
// 4/6 faults on the TMP1075N variant.
enum class FaultCount : uint8_t {
    f1 = 0x00,       // 1 fault (default)
    f2 = 0x01,       // 2 faults
    f3 = 0x02,       // 3 faults (4 on TMP1075N)
    f4 = 0x03        // 4 faults (6 on TMP1075N)
};

// ALERT pin polarity (CFGR.POL)
enum class AlertPolarity : uint8_t {
    ActiveLow  = 0,  // default
    ActiveHigh = 1
};

// ALERT pin function (CFGR.TM)
enum class AlertMode : uint8_t {
    Comparator = 0,  // thermostat behaviour (default)
    Interrupt  = 1   // latched, cleared by reading a register
};

// Driver operating mode
enum class OpMode : uint8_t {
    Continuous = 0,  // device free-runs at ConversionRate; getFrame() reads latest
    OneShot    = 1   // device in shutdown; each trigger starts one conversion
};

struct TempData {
    float    temperature_c;  // °C, 0.0625 °C resolution
    uint64_t timestamp_us;   // DWT timestamp captured at triggerMeasurement()
};

enum TMP1075StatusFlags : uint32_t {
    TMP1075_STATUS_OK              = 0u,
    TMP1075_STATUS_I2C_ERROR       = (1u << 0),
    TMP1075_STATUS_WHOAMI_MISMATCH = (1u << 1),
    TMP1075_STATUS_CONFIG_ERROR    = (1u << 2),
    TMP1075_STATUS_NOT_READY       = (1u << 3),  // conversion still running — not an error
};

class TMP1075_Interface {
public:
    virtual ~TMP1075_Interface() = default;

    virtual bool     init()                                                   = 0;
    virtual bool     ping()                                                   = 0;
    virtual void     configure(ConversionRate rate, FaultCount faults,
                               AlertPolarity pol, AlertMode alertMode)        = 0;
    virtual void     triggerMeasurement()                                     = 0;
    virtual bool     getFrame(TempData& out)                                  = 0;
    virtual uint32_t getStatus() const                                        = 0;
};

} // namespace TMP1075
} // namespace Drivers
