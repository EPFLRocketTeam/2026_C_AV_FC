#pragma once

#include "InvIMU.h"
#include <cstring>
#include "stm32h7xx_hal.h"

// ======================================================================
// MODEL SELECTION & TDK INCLUDES
// ======================================================================

#ifdef USE_ICM45605
    #include "imu/inv_imu_driver.h"
    #include "imu/inv_imu_driver_advanced.h"
    #include "imu/inv_imu_edmp.h"

    static constexpr float INV_IMU_MAX_ACCEL_G = 16.0f;
    static constexpr float INV_IMU_MAX_GYRO_DPS = 2000.0f;
    static constexpr uint8_t INV_IMU_WHO_AM_I_VAL = 0xE0;
#else
    // Default: ICM-45686
    #include "imu/inv_imu_driver.h"
    #include "imu/inv_imu_driver_advanced.h"
    #include "imu/inv_imu_edmp.h"

    static constexpr float INV_IMU_MAX_ACCEL_G = 32.0f;
    static constexpr float INV_IMU_MAX_GYRO_DPS = 4000.0f;
    static constexpr uint8_t INV_IMU_WHO_AM_I_VAL = 0xE9;
#endif

namespace Drivers {
namespace InvIMU {

    // ======================================================================
    // Compile-time tuning knobs (RAM safety)
    // ======================================================================
    #ifndef INVIMU_RING_CAPACITY
    #define INVIMU_RING_CAPACITY 512u
    #endif

    // Default linker section for the ring buffer and burst buffer.
    // On STM32H7, place in D2/AXI SRAM (DMA-accessible), not DTCM.
    // Override with -DINVIMU_RING_SECTION='".foo"' or define
    // INVIMU_DISABLE_SECTION_ATTR to omit the attribute entirely.
    #ifndef INVIMU_RING_SECTION
    #define INVIMU_RING_SECTION ".ram_d2"
    #endif

    // Default linker section for the DMA receive buffer.
    // MUST be in DMA-accessible SRAM on STM32H7 (not DTCM).
    #ifndef INVIMU_DMA_SECTION
    #define INVIMU_DMA_SECTION ".ram_d2"
    #endif

    // Optional DMA mode for FIFO burst reads.
    // 0 = blocking SPI reads (safe default), 1 = HAL SPI RX DMA path.
    #ifndef INVIMU_USE_DMA_DEFAULT
    #define INVIMU_USE_DMA_DEFAULT 0u
    #endif

    // KTP-aligned digital filtering defaults.
    #ifndef INVIMU_ACCEL_BW_DIV
    #define INVIMU_ACCEL_BW_DIV 0u
    #endif

    #ifndef INVIMU_GYRO_BW_DIV
    #define INVIMU_GYRO_BW_DIV 0u
    #endif

    #ifndef INVIMU_ACCEL_LP_AVG
    #define INVIMU_ACCEL_LP_AVG 1u
    #endif

    #ifndef INVIMU_GYRO_LP_AVG
    #define INVIMU_GYRO_LP_AVG 1u
    #endif

    struct Config {
        SPI_HandleTypeDef* hspi;      
        GPIO_TypeDef* cs_port;        
        uint16_t cs_pin;              

        float R_body_from_sensor[3][3] = {
            {1.0f, 0.0f, 0.0f},
            {0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f}
        };

        bool use_dwt_timestamps = true;
        bool use_dma = (INVIMU_USE_DMA_DEFAULT != 0u);
        uint8_t accel_bw_div = INVIMU_ACCEL_BW_DIV;
        uint8_t gyro_bw_div = INVIMU_GYRO_BW_DIV;
        uint8_t accel_lp_avg = INVIMU_ACCEL_LP_AVG;
        uint8_t gyro_lp_avg = INVIMU_GYRO_LP_AVG;
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
        uint32_t statusFlags() const override { return _status_flags; }
        uint32_t dropCount() const override { return _ring_overflow_count; }

        uint32_t getStatusFlags() const { return _status_flags; }
        void clearStatusFlags(uint32_t mask = 0xFFFFFFFFu) { _status_flags &= ~mask; }

        uint64_t getWriteCount() const { return _ring_write_count; }
        uint32_t getRingOverflowCount() const { return _ring_overflow_count; }

        // Frame header statistics for diagnosing FIFO parsing issues.
        uint32_t frameCount0x78() const override { return _frame_count_0x78; }
        uint32_t frameCount0xF0() const override { return _frame_count_0xF0; }
        uint32_t frameCountOther() const override { return _frame_count_other; }

        void onInterrupt(uint64_t irq_us = 0) override;
        void tick() override;        
        void onDmaComplete() override;

    private:
        Config _hw;
        inv_imu_device_t _dev; 

        volatile uint32_t _status_flags = IMU_STATUS_OK;

        float _accel_scale = 0.0f;      // configured FSR (used for 0xF0 ext_header frames)
        float _gyro_scale = 0.0f;       // configured FSR (used for 0xF0 ext_header frames)
        float _accel_scale_hires = 0.0f; // fixed 32G  (used for 0x78 normal hires frames)
        float _gyro_scale_hires = 0.0f;  // fixed 4000DPS (used for 0x78 normal hires frames)

        static constexpr size_t kRingCapacity = (size_t)INVIMU_RING_CAPACITY;

        // NOTE: GCC does not allow __attribute__((section(...))) on non-static class
        // members. To ensure _ring_buffer lives in DMA-accessible SRAM (not DTCM),
        // declare the InvIMU_STM32 object itself in the correct section at the call site:
        //   __attribute__((section(".ram_d2"))) InvIMU_STM32 g_imu(config);
        IMUData _ring_buffer[kRingCapacity];

        volatile uint16_t _head = 0;
        volatile uint16_t _tail = 0;

        volatile uint32_t _ring_overflow_count = 0;
        volatile uint64_t _ring_write_count = 0;

        static constexpr size_t kRawBufferSize = 2048;
        static constexpr uint8_t kFrameSize = 20;
        static constexpr size_t kMaxFrames = (kRawBufferSize / kFrameSize) + 1u;

        // alignas(32) ensures 32-byte D-Cache line alignment for SCB_InvalidateDCache_by_Addr.
        // For DMA accessibility on STM32H7, the containing InvIMU_STM32 object must be
        // placed in D2/AXI SRAM — see note above on _ring_buffer.
        alignas(32) uint8_t _dma_rx_buffer[kRawBufferSize];
        uint8_t _tx_dummy_buf[kRawBufferSize + 1]; // TX dummy for spi_read_fifo TransmitReceive

        IMUData _burst_buffer[kMaxFrames];

        volatile bool _dma_busy = false;
        volatile bool _rx_pending = false;
        volatile bool _irq_pending = false;
        volatile uint16_t _last_dma_size = 0;
        volatile uint64_t _dma_irq_time_us = 0;
        volatile uint32_t _invalidate_size = 0;

        bool _timestamp_initialized = false;
        volatile uint16_t _last_fifo_ts = 0;  
        volatile uint64_t _last_unwrapped_ts = 0; 

        volatile uint64_t _irq_time_us = 0;
        int64_t _fifo_to_abs_offset_us = 0;
        bool _fifo_to_abs_offset_initialized = false;
        uint64_t _last_emitted_ts_us = 0;

        // Frame header statistics
        uint32_t _frame_count_0x78 = 0;
        uint32_t _frame_count_0xF0 = 0;
        uint32_t _frame_count_other = 0;

        float _R_body_from_sensor[3][3];

        static void enable_dwt_cyccnt();
        static bool dwt_is_running();
        static uint64_t now_us(bool use_dwt);
        void applyAlignment(IMUData& io);

        accel_config0_accel_ui_fs_sel_t toAccelFsr(AccelRange ar);
        gyro_config0_gyro_ui_fs_sel_t   toGyroFsr(GyroRange gr);
        accel_config0_accel_odr_t       toAccelOdr(ODR odr);
        gyro_config0_gyro_odr_t         toGyroOdr(ODR odr);
        ipreg_sys2_reg_131_accel_ui_lpfbw_t toAccelBwDiv(uint8_t div);
        ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t toGyroBwDiv(uint8_t div);
        ipreg_sys2_reg_129_accel_lp_avg_sel_t toAccelLpAvg(uint8_t avg);
        ipreg_sys1_reg_170_gyro_lp_avg_sel_t toGyroLpAvg(uint8_t avg);
        
        void processPendingRx();
        bool parseFrameInto(IMUData& out, const uint8_t* p, uint8_t frame_size);
        // POlling helper
        int spi_read_fifo(uint8_t reg, uint8_t* data, uint16_t len);

        
        int spi_read(uint8_t reg, uint8_t* rbuffer, uint32_t len);
        int spi_write(uint8_t reg, uint8_t data); 
        int spi_write_burst(uint8_t reg, const uint8_t* data, uint32_t len); 

        // ---------------------------------------------------------------
        // Multi-instance transport trampolines
        //
        // This SDK version's transport struct has no ctx/userdata field —
        // function pointers are plain (reg, buf, len) with no way to carry
        // instance identity.  We work around this with a static registry of
        // up to INVIMU_MAX_INSTANCES pointers and one dedicated trampoline
        // pair per slot.  Each instance claims the next free slot in init().
        //
        // To support more than 4 IMUs, raise INVIMU_MAX_INSTANCES and add
        // the corresponding trampoline specialisations in InvIMU.cpp.
        // ---------------------------------------------------------------
        #ifndef INVIMU_MAX_INSTANCES
        #define INVIMU_MAX_INSTANCES 4u
        #endif

        uint8_t _instance_idx = 0xFF;  // slot in _registry[], set by init()

        static InvIMU_STM32* _registry[INVIMU_MAX_INSTANCES];

        static int spi_read_0 (uint8_t r, uint8_t* d, uint32_t l);
        static int spi_read_1 (uint8_t r, uint8_t* d, uint32_t l);
        static int spi_read_2 (uint8_t r, uint8_t* d, uint32_t l);
        static int spi_read_3 (uint8_t r, uint8_t* d, uint32_t l);
        static int spi_write_0(uint8_t r, const uint8_t* d, uint32_t l);
        static int spi_write_1(uint8_t r, const uint8_t* d, uint32_t l);
        static int spi_write_2(uint8_t r, const uint8_t* d, uint32_t l);
        static int spi_write_3(uint8_t r, const uint8_t* d, uint32_t l);

        static void sleep_us(uint32_t us);
    };

} // namespace InvIMU
} // namespace Drivers
