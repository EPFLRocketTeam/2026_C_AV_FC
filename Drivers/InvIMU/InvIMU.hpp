#pragma once

#include "InvIMU.h"
#include "stm32hal.h" // Abstraction for STM32 HAL types
#include <cstring>


// ======================================================================
// MODEL SELECTION & TDK INCLUDES
// ======================================================================

#ifdef USE_ICM45605
    #include "icm45605/inv_imu_driver.h"
    #include "icm45605/inv_imu_driver_advanced.h"
    
    static constexpr float INV_IMU_MAX_ACCEL_G = 16.0f;
    static constexpr float INV_IMU_MAX_GYRO_DPS = 2000.0f;
    static constexpr uint8_t INV_IMU_WHO_AM_I_VAL = 0xE0;
#else
    // Default: ICM-45686
    #include "inv_imu_driver.h"
    #include "inv_imu_driver_advanced.h"

    static constexpr float INV_IMU_MAX_ACCEL_G = 32.0f;
    static constexpr float INV_IMU_MAX_GYRO_DPS = 4000.0f;
    static constexpr uint8_t INV_IMU_WHO_AM_I_VAL = 0xE9; 
#endif

namespace Drivers {
namespace InvIMU {

    // ======================================================================
    // Compile-time tuning knobs (RAM safety)
    // ======================================================================
    // Default ring capacity: enough for batching + look-ahead, without eating RAM.
    // Override at build time with -DINVIMU_RING_CAPACITY=...
    #ifndef INVIMU_RING_CAPACITY
    #define INVIMU_RING_CAPACITY 512u
    #endif

    // Default linker section for the ring buffer. Put this in D2/AXI SRAM (DMA-friendly),
    // not in DTCM (precious for stack/KF math). Override with -DINVIMU_RING_SECTION='".foo"'
    // or define INVIMU_DISABLE_SECTION_ATTR to omit the attribute.
    #ifndef INVIMU_RING_SECTION
    #define INVIMU_RING_SECTION ".ram_d2"
    #endif

    // Default linker section for the DMA buffer. Put this in D2/AXI SRAM (DMA-friendly),
    // not in DTCM (precious for stack/KF math). Override with -DINVIMU_DMA_SECTION='".foo"'
    // or define INVIMU_DISABLE_SECTION_ATTR to omit the attribute. 
    #ifndef INVIMU_DMA_SECTION
    #define INVIMU_DMA_SECTION ".ram_d2"
    #endif

    struct Config {
        SPI_HandleTypeDef* hspi;      
        GPIO_TypeDef* cs_port;        
        uint16_t cs_pin;              

        // --- FD&N compliance options ---
        // Body-frame alignment matrix (Body = FRD) applied to scaled outputs.
        // Defaults to identity.
        float R_body_from_sensor[3][3] = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f}
        };

        // Use DWT_CYCCNT for microsecond timestamps.
        bool use_dwt_timestamps = true;
    };

    class InvIMU_STM32 : public InvIMU_Interface {
    public:
        InvIMU_STM32(const Config& config);
        virtual ~InvIMU_STM32() = default;

        bool init() override;
        bool ping() override;
        void configure(AccelRange accel_range, GyroRange gyro_range, ODR odr) override;
        void configureFifo() override;
        void enableFsync() override;
        
        bool getFrame(IMUData& out_data) override;

        // --- Health / compliance helpers ---
        uint32_t getStatusFlags() const { return _status_flags; }
        void clearStatusFlags(uint32_t mask = 0xFFFFFFFFu) { _status_flags &= ~mask; }

        // Ring buffer diagnostics (tape-style)
        uint64_t getWriteCount() const { return _ring_write_count; }
        uint32_t getRingOverflowCount() const { return _ring_overflow_count; }

        // --- Event Handling ---
        void onInterrupt() override; 
        void tick() override;        
        void onDmaComplete() override;

    private:
        Config _hw;
        inv_imu_device_t _dev; 

        // --- Health/Status ---
        volatile uint32_t _status_flags = IMU_STATUS_OK;

        // Sensitivity Scales
        float _accel_scale = 0.0f;
        float _gyro_scale = 0.0f;

        // Ring Buffer
        // - Sized to comfortably absorb FIFO bursts and downstream scheduling jitter.
        // - Placed in a configurable RAM region (see INVIMU_RING_SECTION above).
        static constexpr size_t kRingCapacity = (size_t)INVIMU_RING_CAPACITY;

        #if !defined(INVIMU_DISABLE_SECTION_ATTR)
        __attribute__((section(INVIMU_RING_SECTION)))
        #endif
        IMUData _ring_buffer[kRingCapacity];
        volatile uint16_t _head = 0;
        volatile uint16_t _tail = 0;

        // Architecture compliance: overflow detection & write counter
        volatile uint32_t _ring_overflow_count = 0;
        volatile uint64_t _ring_write_count = 0;

        // DMA Buffer
        static constexpr size_t kRawBufferSize = 2048;
        static constexpr uint8_t kFrameSize = 20;
        static constexpr size_t kMaxFrames = (kRawBufferSize / kFrameSize) + 1u;

        #if !defined(INVIMU_DISABLE_SECTION_ATTR)
        __attribute__((section(INVIMU_DMA_SECTION)))
        #endif
        alignas(32) uint8_t _dma_rx_buffer[kRawBufferSize];

        // Burst scratch buffer (avoids large stack use; processed in task context)
        #if !defined(INVIMU_DISABLE_SECTION_ATTR)
        __attribute__((section(INVIMU_RING_SECTION)))
        #endif
        IMUData _burst_buffer[kMaxFrames];
        // Flags
        volatile bool _dma_busy = false;      // DMA transfer currently in progress
        volatile bool _rx_pending = false;    // DMA completed, data not yet processed
        volatile bool _irq_pending = false;   // Watermark interrupt pending
        volatile uint16_t _last_dma_size = 0;
        volatile uint64_t _dma_irq_time_us = 0;   // EXTI timestamp associated with this DMA burst
        volatile uint32_t _invalidate_size = 0;   // DCache invalidate length (bytes)

        bool _timestamp_initialized = false;
        volatile uint16_t _last_fifo_ts = 0;  
        volatile uint64_t _last_unwrapped_ts = 0; 

        // Timestamp anchoring: FIFO timestamp (sensor-time) -> absolute (µs).
        // We anchor the most recent FIFO sample in a burst to the EXTI timestamp
        // captured at watermark interrupt ("trigger-time" semantics).
        volatile uint64_t _irq_time_us = 0;  // captured in EXTI ISR (monotonic)
        int64_t _fifo_to_abs_offset_us = 0;  // absolute_us = fifo_unwrapped_us + offset
        bool _fifo_to_abs_offset_initialized = false;

        // Alignment (sensor frame -> Body FRD). Copied from Config.
        float _R_body_from_sensor[3][3];

        // Timing helpers
        static void enable_dwt_cyccnt();
        static bool dwt_is_running();
        static uint64_t now_us(bool use_dwt);
        void applyAlignment(IMUData& io);

        // --- Helper Functions for Clamping ---
        // Note: These need TDK types which are available via the includes above
        accel_config0_accel_ui_fs_sel_t toAccelFsr(AccelRange ar);
        gyro_config0_gyro_ui_fs_sel_t   toGyroFsr(GyroRange gr);
        accel_config0_accel_odr_t       toAccelOdr(ODR odr);
        gyro_config0_gyro_odr_t         toGyroOdr(ODR odr);
        
        // Internal Helpers
        void processPendingRx();
        bool parseFrameInto(IMUData& out, const uint8_t* p, uint8_t frame_size);
        
        int spi_read(uint8_t reg, uint8_t* rbuffer, uint32_t len);
        int spi_write(uint8_t reg, uint8_t data); 
        int spi_write_burst(uint8_t reg, const uint8_t* data, uint32_t len); 

        // TDK Trampolines (Must be static to use as function pointers)
        static int spi_read_ctx(void* ctx, uint8_t reg, uint8_t* data, uint32_t len);
        static int spi_write_ctx(void* ctx, uint8_t reg, const uint8_t* data, uint32_t len);
        static void sleep_us(uint32_t us);
    };

} // namespace InvIMU
} // namespace Drivers