#pragma once

#include <cstdint>

/**
 * @brief  Driver Interface for Analog Devices ADXL375 High-g Accelerometer (I2C)
 * @target STM32H7 (Flight Computer)
 *
 * Usage (main loop, N/N+1 pattern):
 *   accel.triggerMeasurement();   // iteration N   → arms the read window
 *   accel.getFrame(data);         // iteration N+1 → reads once DATA_READY is set
 *
 * The ADXL375 free-runs continuously once measurement mode is enabled — there
 * is no one-shot mode. triggerMeasurement()/getFrame() only gate *reads*
 * against the DATA_READY status bit; they do not start/stop conversions.
 */

namespace Drivers {
namespace ADXL375 {

// Output data rate (BW_RATE register D3:D0). Also sets the internal filter
// bandwidth to ODR/2 (datasheet Table 8).
enum class OutputDataRate : uint8_t {
    hz12_5 = 0x07,
    hz25   = 0x08,
    hz50   = 0x09,
    hz100  = 0x0A,  // default (POR value)
    hz200  = 0x0B,
    hz400  = 0x0C,
    hz800  = 0x0D,
    hz1600 = 0x0E,
    hz3200 = 0x0F,
};

struct AccelData {
    float    x_g;
    float    y_g;
    float    z_g;
    uint64_t timestamp_us;  // DWT timestamp captured at triggerMeasurement()
};

enum ADXL375StatusFlags : uint32_t {
    ADXL375_STATUS_OK              = 0u,
    ADXL375_STATUS_I2C_ERROR       = (1u << 0),
    ADXL375_STATUS_WHOAMI_MISMATCH = (1u << 1),
    ADXL375_STATUS_CONFIG_ERROR    = (1u << 2),
    ADXL375_STATUS_NOT_READY       = (1u << 3),  // conversion still running — not an error
};

class ADXL375_Interface {
public:
    virtual ~ADXL375_Interface() = default;

    virtual bool     init()                          = 0;
    virtual bool     ping()                          = 0;
    virtual void     configure(OutputDataRate rate)  = 0;
    virtual void     triggerMeasurement()            = 0;
    virtual bool     getFrame(AccelData& out)        = 0;
    virtual uint32_t getStatus() const               = 0;
};

} // namespace ADXL375
} // namespace Drivers
