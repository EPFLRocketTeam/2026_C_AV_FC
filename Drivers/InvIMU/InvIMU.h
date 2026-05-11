#pragma once

#include <cstdint>

/**
 * @brief  Unified Driver Interface for TDK InvenSense ICM-456xx Family
 * @supports ICM-45686 (High Range) and ICM-45605 (Standard Range)
 * @target STM32H7 (Flight Computer)
 */

namespace Drivers {
namespace InvIMU {

    // ======================================================================
    // CONFIGURATION ENUMS
    // ======================================================================

    enum class AccelRange : uint8_t {
        _32G = 0x00, _16G = 0x01, _8G  = 0x02, _4G  = 0x03, _2G  = 0x04
    };

    enum class GyroRange : uint8_t {
        _4000DPS = 0x00, _2000DPS = 0x01, _1000DPS = 0x02, _500DPS  = 0x03
    };

    enum class ODR : uint8_t {
        _6_4kHz   = 0x03, 
        _3_2kHz   = 0x04,
        _1_6kHz   = 0x05,
        _800Hz    = 0x06,
        _100Hz    = 0x09
    };

    // ======================================================================
    // DATA STRUCTURES
    // ======================================================================

    struct IMUData {
        float accel_x; float accel_y; float accel_z;
        float gyro_x;  float gyro_y;  float gyro_z;
        float temperature;
        uint64_t timestamp_us; 
    };

    /**
     * @brief DAQ/Driver health flags required by the FD&N design document.
     *
     * These flags are intended to be read by the Sensor Manager / Virtual IMU
     * voting layer so that hard-failed sensors can be excluded from voting.
     */
    enum IMUStatusFlags : uint32_t {
        IMU_STATUS_OK                = 0u,
        IMU_STATUS_SPI_ERROR         = (1u << 0),
        IMU_STATUS_WHOAMI_MISMATCH   = (1u << 1),
        IMU_STATUS_FIFO_OVERFLOW     = (1u << 2),
        IMU_STATUS_FRAME_ALIGN_ERR   = (1u << 3),
        IMU_STATUS_EMPTY_PACKET      = (1u << 4),
        IMU_STATUS_TIMESTAMP_DESYNC  = (1u << 5),
    };

    // ======================================================================
    // Virtual INTERFACE
    // ======================================================================

    class InvIMU_Interface {
    public:
        virtual ~InvIMU_Interface() = default;

        virtual bool init() = 0;
        virtual bool ping() = 0;

        virtual void configure(AccelRange ar, GyroRange gr, ODR odr) = 0;
        virtual void configureFifo() = 0;
        virtual void enableFsync() = 0;
        
        virtual bool getFrame(IMUData& out_data) = 0;

        // Optional observability hooks for module-level health monitoring.
        virtual uint32_t statusFlags() const { return IMU_STATUS_OK; }
        virtual uint32_t dropCount() const { return 0u; }

        // Frame header statistics for FIFO parsing diagnostics.
        virtual uint32_t frameCount0x78() const { return 0u; }
        virtual uint32_t frameCount0xF0() const { return 0u; }
        virtual uint32_t frameCountOther() const { return 0u; }
        
        // irq_us: absolute time (µs) of the hardware EXTI event, captured at ISR
        // entry for accurate FIFO timestamp alignment. Pass 0 to fall back to
        // now_us() at the call site (less accurate, may trigger TIMESTAMP_DESYNC).
        virtual void onInterrupt(uint64_t irq_us = 0) = 0;
        virtual void tick() = 0;          // Called by Main Loop (Processes flag)
        virtual void onDmaComplete() = 0; // Called by DMA ISR
    };

} // namespace InvIMU
} // namespace Drivers