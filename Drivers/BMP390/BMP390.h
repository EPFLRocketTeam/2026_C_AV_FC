#pragma once

#include <cstdint>

/**
 * @brief  Driver Interface for Bosch BMP390 Barometric Pressure Sensor
 * @target STM32H7 (Flight Computer)
 *
 * Usage (main loop):
 *   baro.triggerMeasurement();   // iteration N   → starts forced-mode conversion
 *   baro.getFrame(data);         // iteration N+1 → reads result (checks drdy first)
 */

namespace Drivers {
namespace BMP390 {

enum class OsrPressure : uint8_t {
    x1  = 0x00, x2  = 0x01, x4  = 0x02,
    x8  = 0x03, x16 = 0x04, x32 = 0x05
};

enum class OsrTemp : uint8_t {
    x1  = 0x00, x2  = 0x01, x4  = 0x02,
    x8  = 0x03, x16 = 0x04, x32 = 0x05
};

// IIR low-pass filter coefficient (applied to pressure and temperature output)
enum class IIRFilter : uint8_t {
    OFF  = 0x00, c2  = 0x01, c4   = 0x02, c8   = 0x03,
    c16  = 0x04, c32 = 0x05, c64  = 0x06, c128 = 0x07
};

struct BaroData {
    float    pressure_pa;    // Compensated pressure in Pascal
    float    temperature_c;  // Compensated temperature in °C
    uint64_t timestamp_us;   // DWT timestamp captured at triggerMeasurement()
};

enum BMP390StatusFlags : uint32_t {
    BMP390_STATUS_OK              = 0u,
    BMP390_STATUS_SPI_ERROR       = (1u << 0),
    BMP390_STATUS_WHOAMI_MISMATCH = (1u << 1),
    BMP390_STATUS_CALIB_ERROR     = (1u << 2),
    BMP390_STATUS_NOT_READY       = (1u << 3),  // drdy not set yet — not an error
};

class BMP390_Interface {
public:
    virtual ~BMP390_Interface() = default;

    virtual bool     init()                                                    = 0;
    virtual bool     ping()                                                    = 0;
    virtual void     configure(OsrPressure osr_p, OsrTemp osr_t,
                               IIRFilter filter)                               = 0;
    virtual void     triggerMeasurement()                                      = 0;
    virtual bool     getFrame(BaroData& out)                                   = 0;
    virtual uint32_t getStatus() const                                         = 0;
};

} // namespace BMP390
} // namespace Drivers
