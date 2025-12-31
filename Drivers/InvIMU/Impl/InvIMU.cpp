#include "InvIMU.hpp"

// Registers used for manual SPI access
static constexpr uint8_t REG_BANK_SEL       = 0x76; 
static constexpr uint8_t REG_FIFO_COUNTH    = 0x12; 
static constexpr uint8_t REG_FIFO_DATA      = 0x14; 
static constexpr uint8_t REG_WHO_AM_I       = 0x72;

namespace Drivers {
namespace InvIMU {

// ======================================================================
// 0. TIMING HELPERS
// ======================================================================

void InvIMU_STM32::enable_dwt_cyccnt() {
    // Enable TRC
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // Enable CYCCNT if not already running (do NOT reset if already enabled)
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

bool InvIMU_STM32::dwt_is_running() {
    // Check that tracing and CYCCNT are enabled and the counter is moving.
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) return false;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) return false;
    const uint32_t a = DWT->CYCCNT;
    __NOP();
    const uint32_t b = DWT->CYCCNT;
    return (a != b);
}


uint64_t InvIMU_STM32::now_us(bool use_dwt) {
    // FD&N compliance note:
    // - DWT->CYCCNT is a 32-bit counter and WILL wrap (~8.95s @ 480MHz).
    // - We must return a MONOTONIC microsecond timebase.
    // This function unwraps CYCCNT into a 64-bit accumulator.

    if (use_dwt) {
        static uint32_t last_cyccnt = 0u;
        static uint64_t acc_us = 0u;

        const uint32_t cycles_per_us = (SystemCoreClock / 1000000u);
        if (cycles_per_us == 0u) return acc_us;

        const uint32_t cyccnt = DWT->CYCCNT;
        const uint32_t delta_cycles = (uint32_t)(cyccnt - last_cyccnt); // wrap-safe
        last_cyccnt = cyccnt;

        // Convert cycles -> µs (integer). The remainder is intentionally dropped;
        // for higher precision, keep a remainder accumulator.
        acc_us += (uint64_t)(delta_cycles / cycles_per_us);
        return acc_us;
    }

    // Fallback: extend the 1ms HAL tick into µs. Not as good as DWT, but monotonic.
    static uint32_t last_ms = 0u;
    static uint64_t acc_us = 0u;
    const uint32_t ms = HAL_GetTick();
    acc_us += (uint64_t)((uint32_t)(ms - last_ms)) * 1000ull; // wrap-safe
    last_ms = ms;
    return acc_us;
}

void InvIMU_STM32::applyAlignment(IMUData& d) {
    const float ax = d.accel_x, ay = d.accel_y, az = d.accel_z;
    const float gx = d.gyro_x,  gy = d.gyro_y,  gz = d.gyro_z;

    d.accel_x = _R_body_from_sensor[0][0]*ax + _R_body_from_sensor[0][1]*ay + _R_body_from_sensor[0][2]*az;
    d.accel_y = _R_body_from_sensor[1][0]*ax + _R_body_from_sensor[1][1]*ay + _R_body_from_sensor[1][2]*az;
    d.accel_z = _R_body_from_sensor[2][0]*ax + _R_body_from_sensor[2][1]*ay + _R_body_from_sensor[2][2]*az;

    d.gyro_x  = _R_body_from_sensor[0][0]*gx + _R_body_from_sensor[0][1]*gy + _R_body_from_sensor[0][2]*gz;
    d.gyro_y  = _R_body_from_sensor[1][0]*gx + _R_body_from_sensor[1][1]*gy + _R_body_from_sensor[1][2]*gz;
    d.gyro_z  = _R_body_from_sensor[2][0]*gx + _R_body_from_sensor[2][1]*gy + _R_body_from_sensor[2][2]*gz;
}

// ======================================================================
// 1. MAPPING HELPERS
// ======================================================================

accel_config0_accel_ui_fs_sel_t InvIMU_STM32::toAccelFsr(AccelRange ar) {
    float requested_g = 16.0f;
    switch(ar) {
        case AccelRange::_32G: requested_g = 32.0f; break;
        case AccelRange::_16G: requested_g = 16.0f; break;
        case AccelRange::_8G:  requested_g = 8.0f;  break;
        case AccelRange::_4G:  requested_g = 4.0f;  break;
        case AccelRange::_2G:  requested_g = 2.0f;  break;
    }

    // Clamp to model-specific maximum
    if (requested_g > INV_IMU_MAX_ACCEL_G) {
        requested_g = INV_IMU_MAX_ACCEL_G;
    }

    if (requested_g >= 32.0f) {
        #ifndef USE_ICM45605
            return ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G;
        #else
            return ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G; 
        #endif
    }
    if (requested_g >= 16.0f) return ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;
    if (requested_g >= 8.0f)  return ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G;
    if (requested_g >= 4.0f)  return ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;
    return ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G;
}

gyro_config0_gyro_ui_fs_sel_t InvIMU_STM32::toGyroFsr(GyroRange gr) {
    float requested_dps = 2000.0f;
    switch(gr) {
        case GyroRange::_4000DPS: requested_dps = 4000.0f; break;
        case GyroRange::_2000DPS: requested_dps = 2000.0f; break;
        case GyroRange::_1000DPS: requested_dps = 1000.0f; break;
        case GyroRange::_500DPS:  requested_dps = 500.0f;  break;
    }

    // Clamp to model-specific maximum
    if (requested_dps > INV_IMU_MAX_GYRO_DPS) {
        requested_dps = INV_IMU_MAX_GYRO_DPS;
    }

    if (requested_dps >= 4000.0f) {
        #ifndef USE_ICM45605
            return GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS;
        #else
            return GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS; 
        #endif
    }
    if (requested_dps >= 2000.0f) return GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;
    if (requested_dps >= 1000.0f) return GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS;
    return GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS;
}

accel_config0_accel_odr_t InvIMU_STM32::toAccelOdr(ODR odr) {
    switch(odr) {
        case ODR::_6_4kHz: return ACCEL_CONFIG0_ACCEL_ODR_6400_HZ;
        case ODR::_3_2kHz: return ACCEL_CONFIG0_ACCEL_ODR_3200_HZ;
        case ODR::_1_6kHz: return ACCEL_CONFIG0_ACCEL_ODR_1600_HZ;
        case ODR::_800Hz:  return ACCEL_CONFIG0_ACCEL_ODR_800_HZ;
        default:           return ACCEL_CONFIG0_ACCEL_ODR_100_HZ;
    }
}

gyro_config0_gyro_odr_t InvIMU_STM32::toGyroOdr(ODR odr) {
    switch(odr) {
        case ODR::_6_4kHz: return GYRO_CONFIG0_GYRO_ODR_6400_HZ;
        case ODR::_3_2kHz: return GYRO_CONFIG0_GYRO_ODR_3200_HZ;
        case ODR::_1_6kHz: return GYRO_CONFIG0_GYRO_ODR_1600_HZ;
        case ODR::_800Hz:  return GYRO_CONFIG0_GYRO_ODR_800_HZ;
        default:           return GYRO_CONFIG0_GYRO_ODR_100_HZ;
    }
}

// ======================================================================
// 2. CONTEXT WRAPPERS
// ======================================================================

int InvIMU_STM32::spi_read_ctx(void* ctx, uint8_t reg, uint8_t* data, uint32_t len) {
    return static_cast<InvIMU_STM32*>(ctx)->spi_read(reg, data, len);
}

int InvIMU_STM32::spi_write_ctx(void* ctx, uint8_t reg, const uint8_t* data, uint32_t len) {
    return static_cast<InvIMU_STM32*>(ctx)->spi_write_burst(reg, data, len);
}

void InvIMU_STM32::sleep_us(uint32_t us) {
    // Must never hang if DWT is disabled/not running.
    // Use DWT cycle counter when available; otherwise fall back to HAL_Delay (ms-resolution).
    const uint32_t cycles_per_us = (SystemCoreClock / 1000000u);
    if (cycles_per_us != 0u && dwt_is_running()) {
        const uint32_t start = DWT->CYCCNT;
        const uint32_t wait_cycles = us * cycles_per_us;
        while ((uint32_t)(DWT->CYCCNT - start) < wait_cycles) { __NOP(); }
        return;
    }

    // Fallback: coarse delay. (Used only if DWT timestamps are disabled/unavailable.)
    const uint32_t ms = us / 1000u;
    if (ms) HAL_Delay(ms);
    // Best-effort sub-ms delay: attempt to enable DWT without resetting if it exists.
    const uint32_t rem = us - (ms * 1000u);
    if (rem && cycles_per_us != 0u) {
        enable_dwt_cyccnt();
        if (dwt_is_running()) {
            const uint32_t start = DWT->CYCCNT;
            const uint32_t wait_cycles = rem * cycles_per_us;
            while ((uint32_t)(DWT->CYCCNT - start) < wait_cycles) { __NOP(); }
        }
    }
}

// ======================================================================
// 3. MAIN METHODS
// ======================================================================

InvIMU_STM32::InvIMU_STM32(const Config& config) : _hw(config) {
    std::memset(&_dev, 0, sizeof(_dev));
    std::memset((void*)_dma_rx_buffer, 0, kRawBufferSize);

    // Copy alignment matrix
    std::memcpy(_R_body_from_sensor, _hw.R_body_from_sensor, sizeof(_R_body_from_sensor));
}

bool InvIMU_STM32::init() {
    if (_hw.use_dwt_timestamps) {
        enable_dwt_cyccnt();
    }

    _dev.transport.ctx = this;
    _dev.transport.read_reg_ctx = spi_read_ctx;
    _dev.transport.write_reg_ctx = spi_write_ctx;
    _dev.transport.sleep_us = sleep_us;
    _dev.transport.serif_type = UI_SPI4;
    
    if (inv_imu_adv_init(&_dev) != 0) return false;
    
    return true;
}

bool InvIMU_STM32::getFrame(IMUData& out_data) {
    if (_tail == _head) return false;

    out_data = _ring_buffer[_tail];
    __DMB();
    _tail = (_tail + 1u) % kRingCapacity;
    return true;
}

void InvIMU_STM32::configure(AccelRange ar, GyroRange gr, ODR odr) {
    auto tdk_accel_fsr = toAccelFsr(ar);
    auto tdk_gyro_fsr  = toGyroFsr(gr);
    auto tdk_accel_odr = toAccelOdr(odr);
    auto tdk_gyro_odr  = toGyroOdr(odr);

    inv_imu_set_accel_fsr(&_dev, tdk_accel_fsr);
    inv_imu_set_gyro_fsr(&_dev, tdk_gyro_fsr);
    inv_imu_set_accel_frequency(&_dev, tdk_accel_odr);
    inv_imu_set_gyro_frequency(&_dev, tdk_gyro_odr);

    // Set Low Noise Mode ( "startAccel"/"startGyro")
    inv_imu_set_accel_mode(&_dev, PWR_MGMT0_ACCEL_MODE_LN);
    inv_imu_set_gyro_mode(&_dev, PWR_MGMT0_GYRO_MODE_LN);

    // Update Scales for Parser logic
    // (Recalculating from enum to ensure consistency with clamping)
    float final_g = 16.0f;
    if (tdk_accel_fsr == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G) final_g = 32.0f; 
    else if (tdk_accel_fsr == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G) final_g = 16.0f;
    else if (tdk_accel_fsr == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G) final_g = 8.0f;
    else if (tdk_accel_fsr == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G) final_g = 4.0f;
    else final_g = 2.0f;

    // For 20-bit signed: range is ±2^19 = ±524288
    _accel_scale = (final_g / 524288.0f) * 9.80665f;

    float final_dps = 2000.0f;
    if (tdk_gyro_fsr == GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS) final_dps = 4000.0f; 
    else if (tdk_gyro_fsr == GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS) final_dps = 2000.0f;
    else if (tdk_gyro_fsr == GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS) final_dps = 1000.0f;
    else final_dps = 500.0f;

    _gyro_scale = (final_dps /524288.0f) * (3.14159265f / 180.0f);  // // Div by 2^19 for 20-bit
}

void InvIMU_STM32::configureFifo() {

    // Disable APEX to ensure full 8KB FIFO
    // (ICM-45605 / ICM-45686 specific)

    if (spi_write(0x29, 0x00) != 0) { _status_flags |= IMU_STATUS_SPI_ERROR; return; }
    if (spi_write(0x2A, 0x04) != 0) { _status_flags |= IMU_STATUS_SPI_ERROR; return; }

    inv_imu_fifo_config_t f{};
    f.accel_en = INV_IMU_ENABLE;
    f.gyro_en = INV_IMU_ENABLE;
    f.temp_en = INV_IMU_ENABLE; 
    f.hires_en = INV_IMU_ENABLE;
    f.fifo_wm_th = 32;           // SET 5ms LATENCY (32 samples)    
    f.fifo_mode = FIFO_CONFIG0_FIFO_MODE_STREAM_TO_FIFO; 

    inv_imu_set_fifo_config(&_dev, &f);

    inv_imu_int_state_t int_config;
    std::memset(&int_config, 0, sizeof(int_config));
    int_config.INV_FIFO_THS = INV_IMU_ENABLE;
    
    inv_imu_set_config_int(&_dev, INV_IMU_INT1, &int_config);
    
    // Optional: Verify HiRes is properly enabled
    if (_dev.fifo_frame_size != 20) {
        // ERROR: HiRes not enabled! Frame size should be 20 bytes
        // Consider logging or asserting here
    }
}

void InvIMU_STM32::enableFsync() {
    inv_imu_adv_set_int2_pin_usage(&_dev, IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_FSYNC);
    inv_imu_adv_configure_fsync_ap_tag(&_dev, FSYNC_CONFIG0_AP_FSYNC_TEMP);
    inv_imu_adv_enable_fsync(&_dev);
}



// ======================================================================
// 4. DATA PROCESSING 
// ======================================================================

bool InvIMU_STM32::parseFrameInto(IMUData& out, const uint8_t* p, uint8_t /*frame_size*/) {
    // Validate header
    uint8_t header = p[0];
    if (header & 0x80) {
        _status_flags |= IMU_STATUS_EMPTY_PACKET;
        return false; // Empty packet
    }
    
    // Packet 3 (20-byte HiRes) Layout (as per diagram):
    // [0]     Header
    // [1-2]   Accel X High bytes (Ax H, Ax L)
    // [3-4]   Accel Y High bytes (Ay H, Ay L)
    // [5-6]   Accel Z High bytes (Az H, Az L)
    // [7-8]   Gyro X High bytes (Gx H, Gx L)
    // [9-10]  Gyro Y High bytes (Gy H, Gy L)
    // [11-12] Gyro Z High bytes (Gz H, Gz L)
    // [13-14] Temperature (Temp H, Temp L) - 16-bit!
    // [15-16] Timestamp (Timestamp H, Timestamp L)
    // [17]    Ax LSB (high nibble) | Gx LSB (low nibble)
    // [18]    Ay LSB (high nibble) | Gy LSB (low nibble)
    // [19]    Az LSB (high nibble) | Gz LSB (low nibble)
    
    // Read 16-bit high bytes (with sign extension)
    int16_t reg16_a[3], reg16_g[3];
    reg16_a[0] = (int16_t)((p[1] << 8) | p[2]);
    reg16_a[1] = (int16_t)((p[3] << 8) | p[4]);
    reg16_a[2] = (int16_t)((p[5] << 8) | p[6]);
    reg16_g[0] = (int16_t)((p[7] << 8) | p[8]);
    reg16_g[1] = (int16_t)((p[9] << 8) | p[10]);
    reg16_g[2] = (int16_t)((p[11] << 8) | p[12]);
    
    // Reconstruct 20-bit signed values
    // Method: (16-bit signed << 4) | (4-bit LSB extension)
    int32_t raw_ax = ((int32_t)reg16_a[0] << 4) | (p[17] >> 4);
    int32_t raw_ay = ((int32_t)reg16_a[1] << 4) | (p[18] >> 4);
    int32_t raw_az = ((int32_t)reg16_a[2] << 4) | (p[19] >> 4);
    
    int32_t raw_gx = ((int32_t)reg16_g[0] << 4) | (p[17] & 0x0F);
    int32_t raw_gy = ((int32_t)reg16_g[1] << 4) | (p[18] & 0x0F);
    int32_t raw_gz = ((int32_t)reg16_g[2] << 4) | (p[19] & 0x0F);
    
    // Apply scales (calculated in configure() for 20-bit range)
    out.accel_x = raw_ax * _accel_scale;
    out.accel_y = raw_ay * _accel_scale;
    out.accel_z = raw_az * _accel_scale;
    
    out.gyro_x = raw_gx * _gyro_scale;
    out.gyro_y = raw_gy * _gyro_scale;
    out.gyro_z = raw_gz * _gyro_scale;
    
    // Temperature: Bytes 13-14 are 16-bit signed (Temp H, Temp L)
    int16_t raw_temp = (int16_t)((p[13] << 8) | p[14]);
    // Formula from ICM-456xx datasheet: Temp(°C) = 25 + (TEMP_DATA / 128)
    out.temperature = (raw_temp / 128.0f) + 25.0f + 273.15f; // Convert to Kelvin
    
    // Timestamp unwrapping (16-bit to 64-bit)
    // Bytes 15-16 (Timestamp H, Timestamp L)
    uint16_t raw_ts = (uint16_t)((p[15] << 8) | p[16]);
    
    if (!_timestamp_initialized) {
        _last_fifo_ts = raw_ts;
        _timestamp_initialized = true;
    }
    
    // Calculate 16-bit delta (handles natural overflow)
    uint16_t delta = raw_ts - _last_fifo_ts;
    _last_fifo_ts = raw_ts;
    
    // Unwrap the timestamp (16-bit delta added to 64-bit counter)
    _last_unwrapped_ts += delta;
    
    // NOTE: this is the *unwrapped FIFO timestamp* (sensor-time). It will be
    // anchored to absolute time in onDmaComplete() to satisfy FD&N "trigger time".
    out.timestamp_us = _last_unwrapped_ts;
    return true;
}


void InvIMU_STM32::processPendingRx() {
    if (!_rx_pending) return;
    _rx_pending = false;

    // Invalidate DCache in task context BEFORE reading DMA buffer.
    const uint32_t inv_len = _invalidate_size;
    if (inv_len > 0u) {
        SCB_InvalidateDCache_by_Addr((uint32_t*)_dma_rx_buffer, inv_len);
    }

    const uint16_t total_bytes = _last_dma_size;
    constexpr uint8_t frame_size = kFrameSize;
    const uint16_t n_frames = (uint16_t)(total_bytes / frame_size);
    if (n_frames == 0u) return;

    // Parse into burst buffer (valid frames only).
    uint16_t parsed = 0u;
    for (uint16_t i = 0; i < n_frames && parsed < (uint16_t)kMaxFrames; ++i) {
        const uint16_t off = (uint16_t)(i * frame_size);
        IMUData d{};
        if (parseFrameInto(d, &_dma_rx_buffer[off], frame_size)) {
            _burst_buffer[parsed++] = d;
        }
    }
    if (parsed == 0u) return;

    // Anchor FIFO timestamps to absolute time using the most recent valid sample.
    const uint64_t last_fifo_us = _burst_buffer[parsed - 1u].timestamp_us;
    const uint64_t irq_us = _dma_irq_time_us;

    if (!_fifo_to_abs_offset_initialized) {
        _fifo_to_abs_offset_us = (int64_t)irq_us - (int64_t)last_fifo_us;
        _fifo_to_abs_offset_initialized = true;
    } else {
        const int64_t predicted_irq = (int64_t)last_fifo_us + _fifo_to_abs_offset_us;
        const int64_t err = (int64_t)irq_us - predicted_irq;
        if (err > 500 || err < -500) {
            _fifo_to_abs_offset_us += err;
            _status_flags |= IMU_STATUS_TIMESTAMP_DESYNC;
        }
    }

    // Commit burst into ring buffer with absolute timestamps + alignment.
    for (uint16_t i = 0; i < parsed; ++i) {
        IMUData d = _burst_buffer[i];
        d.timestamp_us = (uint64_t)((int64_t)d.timestamp_us + _fifo_to_abs_offset_us);
        applyAlignment(d);

        const uint16_t next_head = (uint16_t)((_head + 1u) % kRingCapacity);
        if (next_head == _tail) {
            // Ring full: overwrite oldest (keep newest) + count overflow.
            _tail = (uint16_t)((_tail + 1u) % kRingCapacity);
            _ring_overflow_count++;
            _status_flags |= IMU_STATUS_FIFO_OVERFLOW;
        }
        _ring_buffer[_head] = d;
        __DMB();
        _head = next_head;
        _ring_write_count++;
    }
}

// ======================================================================
// 5. INTERRUPT & DMA
// ======================================================================

void InvIMU_STM32::onInterrupt() {
    _irq_pending = true;

    // Capture EXTI time as the "trigger-time" anchor.
    _irq_time_us = now_us(_hw.use_dwt_timestamps);
}

void InvIMU_STM32::tick() {
    // Process completed DMA in task context (architecture requirement: keep ISRs minimal).
    if (_rx_pending) {
        processPendingRx();
    }
    if (!_irq_pending) return;
    _irq_pending = false;

    if (_dma_busy || _rx_pending) return;

    if (spi_write(REG_BANK_SEL, 0x00) != 0) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return;
    }

    uint8_t counts[2];
    if (spi_read(REG_FIFO_COUNTH, counts, 2) != 0) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return;
    }
    
    uint16_t count = (counts[0] << 8) | counts[1];
    
    if (count == 0) return;
    if (count > kRawBufferSize) {
        _status_flags |= IMU_STATUS_FIFO_OVERFLOW;
        count = kRawBufferSize;
    }

    // Enforce frame alignment (20-byte HiRes frames)
    constexpr uint8_t frame_size = 20;
    if ((count % frame_size) != 0u) {
        _status_flags |= IMU_STATUS_FRAME_ALIGN_ERR;
        count = (count / frame_size) * frame_size;
        if (count == 0u) return;
    }

    _dma_busy = true;
    _last_dma_size = count;
    _dma_irq_time_us = _irq_time_us; 
    
    // Start DMA
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_RESET);
    uint8_t reg = REG_FIFO_DATA | 0x80; 
    if (HAL_SPI_Transmit(_hw.hspi, &reg, 1, 10) != HAL_OK) {
        HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
        _dma_busy = false;
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return;
    }

    if (HAL_SPI_Receive_DMA(_hw.hspi, _dma_rx_buffer, count) != HAL_OK) {
        HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
        _dma_busy = false;
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return;
    }
}

void InvIMU_STM32::onDmaComplete() {
    // DMA-complete is typically ISR context on STM32 HAL.
    // Architecture requirement: keep ISR work minimal (<1µs).
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);

    const uint16_t total_bytes = _last_dma_size;
    if (total_bytes > 0u) {
        _invalidate_size = ((uint32_t)total_bytes + 31u) & ~31u;
    } else {
        _invalidate_size = (uint32_t)kRawBufferSize;
    }

    _dma_busy = false;
    _rx_pending = true;
}

bool InvIMU_STM32::ping() {
    uint8_t who_am_i = 0;
    if (spi_read(REG_WHO_AM_I, &who_am_i, 1) != 0) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return false;
    }
    const bool ok = (who_am_i == INV_IMU_WHO_AM_I_VAL);
    if (!ok) _status_flags |= IMU_STATUS_WHOAMI_MISMATCH;
    return ok;
}

int InvIMU_STM32::spi_write(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg, data };
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_RESET);
    int rc = HAL_SPI_Transmit(_hw.hspi, tx, 2, 10);
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
    if (rc != HAL_OK) _status_flags |= IMU_STATUS_SPI_ERROR;
    return (rc == HAL_OK) ? 0 : -1;
}

int InvIMU_STM32::spi_write_burst(uint8_t reg, const uint8_t* data, uint32_t len) {
    if (len > 64) return -1;
    uint8_t tx_buf[65];
    tx_buf[0] = reg;
    std::memcpy(&tx_buf[1], data, len);

    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_RESET);
    int rc = HAL_SPI_Transmit(_hw.hspi, tx_buf, len + 1, 100); 
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
    if (rc != HAL_OK) _status_flags |= IMU_STATUS_SPI_ERROR;
    return (rc == HAL_OK) ? 0 : -1;
}

int InvIMU_STM32::spi_read(uint8_t reg, uint8_t* data, uint32_t len) {
    uint8_t reg_addr = reg | 0x80;
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(_hw.hspi, &reg_addr, 1, 10) != HAL_OK) {
        HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return -1;
    }
    int rc = HAL_SPI_Receive(_hw.hspi, data, len, 100);
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
    if (rc != HAL_OK) _status_flags |= IMU_STATUS_SPI_ERROR;
    return (rc == HAL_OK) ? 0 : -1;
}

} // namespace InvIMU
} // namespace Drivers