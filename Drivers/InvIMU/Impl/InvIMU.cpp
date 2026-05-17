#include "InvIMU.hpp"
#include "Application/app_timebase.h"
#include <stdio.h>

// Registers used for manual SPI access
static constexpr uint8_t REG_BANK_SEL       = 0x76; 
static constexpr uint8_t REG_FIFO_COUNTH    = 0x12; 
static constexpr uint8_t REG_FIFO_DATA      = 0x14; 
static constexpr uint8_t REG_WHO_AM_I       = 0x72;
static constexpr uint32_t INV_IMU_SPI_TX_TIMEOUT_MS = 10u;
static constexpr uint32_t INV_IMU_SPI_FIFO_RX_TIMEOUT_MS = 10u;

namespace Drivers {
namespace InvIMU {

// Static instance registry — one slot per IMU, indexed by _instance_idx.
InvIMU_STM32* InvIMU_STM32::_registry[INVIMU_MAX_INSTANCES] = {};

// ======================================================================
// 0. TIMING HELPERS
// ======================================================================

void InvIMU_STM32::enable_dwt_cyccnt() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

bool InvIMU_STM32::dwt_is_running() {
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) return false;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) return false;
    const uint32_t a = DWT->CYCCNT;
    __NOP();
    const uint32_t b = DWT->CYCCNT;
    return (a != b);
}

uint64_t InvIMU_STM32::now_us(bool use_dwt) {
    if (use_dwt) {
        return app_timebase_now_us();
    }
    return static_cast<uint64_t>(HAL_GetTick()) * 1000ULL;
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
    if (requested_g > INV_IMU_MAX_ACCEL_G) requested_g = INV_IMU_MAX_ACCEL_G;
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
    if (requested_dps > INV_IMU_MAX_GYRO_DPS) requested_dps = INV_IMU_MAX_GYRO_DPS;
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

ipreg_sys2_reg_131_accel_ui_lpfbw_t InvIMU_STM32::toAccelBwDiv(uint8_t div) {
    switch (div) {
        case 0:   return IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_NO_FILTER;
        case 4:   return IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4;
        case 8:   return IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_8;
        case 16:  return IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_16;
        case 32:  return IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_32;
        case 64:  return IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_64;
        case 128: return IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_128;
        default:  return IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4;
    }
}

ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t InvIMU_STM32::toGyroBwDiv(uint8_t div) {
    switch (div) {
        case 0:   return IPREG_SYS1_REG_172_GYRO_UI_LPFBW_NO_FILTER;
        case 4:   return IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4;
        case 8:   return IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8;
        case 16:  return IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_16;
        case 32:  return IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_32;
        case 64:  return IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_64;
        case 128: return IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_128;
        default:  return IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4;
    }
}

ipreg_sys2_reg_129_accel_lp_avg_sel_t InvIMU_STM32::toAccelLpAvg(uint8_t avg) {
    switch (avg) {
        case 1:  return IPREG_SYS2_REG_129_ACCEL_LP_AVG_1;
        case 2:  return IPREG_SYS2_REG_129_ACCEL_LP_AVG_2;
        case 4:  return IPREG_SYS2_REG_129_ACCEL_LP_AVG_4;
        case 5:  return IPREG_SYS2_REG_129_ACCEL_LP_AVG_5;
        case 7:  return IPREG_SYS2_REG_129_ACCEL_LP_AVG_7;
        case 8:  return IPREG_SYS2_REG_129_ACCEL_LP_AVG_8;
        case 10: return IPREG_SYS2_REG_129_ACCEL_LP_AVG_10;
        case 11: return IPREG_SYS2_REG_129_ACCEL_LP_AVG_11;
        case 16: return IPREG_SYS2_REG_129_ACCEL_LP_AVG_16;
        case 18: return IPREG_SYS2_REG_129_ACCEL_LP_AVG_18;
        case 20: return IPREG_SYS2_REG_129_ACCEL_LP_AVG_20;
        case 32: return IPREG_SYS2_REG_129_ACCEL_LP_AVG_32;
        case 64: return IPREG_SYS2_REG_129_ACCEL_LP_AVG_64;
        default: return IPREG_SYS2_REG_129_ACCEL_LP_AVG_1;
    }
}

ipreg_sys1_reg_170_gyro_lp_avg_sel_t InvIMU_STM32::toGyroLpAvg(uint8_t avg) {
    switch (avg) {
        case 1:  return IPREG_SYS1_REG_170_GYRO_LP_AVG_1;
        case 2:  return IPREG_SYS1_REG_170_GYRO_LP_AVG_2;
        case 4:  return IPREG_SYS1_REG_170_GYRO_LP_AVG_4;
        case 5:  return IPREG_SYS1_REG_170_GYRO_LP_AVG_5;
        case 7:  return IPREG_SYS1_REG_170_GYRO_LP_AVG_7;
        case 8:  return IPREG_SYS1_REG_170_GYRO_LP_AVG_8;
        case 10: return IPREG_SYS1_REG_170_GYRO_LP_AVG_10;
        case 11: return IPREG_SYS1_REG_170_GYRO_LP_AVG_11;
        case 16: return IPREG_SYS1_REG_170_GYRO_LP_AVG_16;
        case 18: return IPREG_SYS1_REG_170_GYRO_LP_AVG_18;
        case 20: return IPREG_SYS1_REG_170_GYRO_LP_AVG_20;
        case 32: return IPREG_SYS1_REG_170_GYRO_LP_AVG_32;
        case 64: return IPREG_SYS1_REG_170_GYRO_LP_AVG_64;
        default: return IPREG_SYS1_REG_170_GYRO_LP_AVG_1;
    }
}

// ======================================================================
// 2. CONTEXT WRAPPERS (Bridging static callbacks to instance)
// ======================================================================

// One read/write trampoline pair per registry slot.
// The TDK transport has no ctx field, so each slot needs its own function address.
int InvIMU_STM32::spi_read_0 (uint8_t r, uint8_t* d, uint32_t l) { return _registry[0]->spi_read(r,d,l); }
int InvIMU_STM32::spi_read_1 (uint8_t r, uint8_t* d, uint32_t l) { return _registry[1]->spi_read(r,d,l); }
int InvIMU_STM32::spi_read_2 (uint8_t r, uint8_t* d, uint32_t l) { return _registry[2]->spi_read(r,d,l); }
int InvIMU_STM32::spi_read_3 (uint8_t r, uint8_t* d, uint32_t l) { return _registry[3]->spi_read(r,d,l); }
int InvIMU_STM32::spi_write_0(uint8_t r, const uint8_t* d, uint32_t l) { return _registry[0]->spi_write_burst(r,d,l); }
int InvIMU_STM32::spi_write_1(uint8_t r, const uint8_t* d, uint32_t l) { return _registry[1]->spi_write_burst(r,d,l); }
int InvIMU_STM32::spi_write_2(uint8_t r, const uint8_t* d, uint32_t l) { return _registry[2]->spi_write_burst(r,d,l); }
int InvIMU_STM32::spi_write_3(uint8_t r, const uint8_t* d, uint32_t l) { return _registry[3]->spi_write_burst(r,d,l); }

void InvIMU_STM32::sleep_us(uint32_t us) {
    const uint32_t cycles_per_us = (SystemCoreClock / 1000000u);
    // Fast path: DWT already running — spin for the exact number of cycles.
    if (cycles_per_us != 0u && dwt_is_running()) {
        const uint32_t start = DWT->CYCCNT;
        const uint32_t wait_cycles = us * cycles_per_us;
        while ((uint32_t)(DWT->CYCCNT - start) < wait_cycles) { __NOP(); }
        return;
    }
    // Fallback: DWT not yet running.
    // Cover the ms-level portion with HAL_Delay, then spin DWT for the
    // sub-ms remainder.  Without the remainder, a call like sleep_us(4)
    // would produce zero delay, corrupting MREG accesses inside the TDK
    // driver (SMC_CONTROL_0 and friends require ≥4 µs between writes).
    const uint32_t ms = us / 1000u;
    if (ms) HAL_Delay(ms);
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
    std::memset(_tx_dummy_buf, 0, sizeof(_tx_dummy_buf));
    std::memcpy(_R_body_from_sensor, _hw.R_body_from_sensor, sizeof(_R_body_from_sensor));
}

bool InvIMU_STM32::init() {
    // Always enable DWT cycle counter — required for MREG timing.
    // The TDK transport layer calls sleep_us(4) between each IREG_ADDR/DATA
    // write. Without DWT the sleep_us fallback uses HAL_Delay(ms) where
    // 4us → 0ms → no delay at all, corrupting MREG accesses (SMC_CONTROL_0
    // at 0xa258 is one such register written by inv_imu_set_fifo_config).
    enable_dwt_cyccnt();

    // Claim the next free slot in the registry.
    _instance_idx = 0xFF;
    for (uint8_t i = 0; i < INVIMU_MAX_INSTANCES; ++i) {
        if (_registry[i] == nullptr) { _registry[i] = this; _instance_idx = i; break; }
    }
    if (_instance_idx == 0xFF) return false;  // too many instances

    // Wire the trampoline pair for this slot.
    using ReadFn  = int(*)(uint8_t, uint8_t*, uint32_t);
    using WriteFn = int(*)(uint8_t, const uint8_t*, uint32_t);
    static const ReadFn  read_fns [INVIMU_MAX_INSTANCES] = { spi_read_0,  spi_read_1,  spi_read_2,  spi_read_3  };
    static const WriteFn write_fns[INVIMU_MAX_INSTANCES] = { spi_write_0, spi_write_1, spi_write_2, spi_write_3 };

    _dev.transport.read_reg   = read_fns [_instance_idx];
    _dev.transport.write_reg  = write_fns[_instance_idx];
    _dev.transport.sleep_us   = sleep_us;
    _dev.transport.serif_type = UI_SPI4;
    if (inv_imu_adv_init(&_dev) != 0) return false;
    // NOTE: inv_imu_edmp_disable() is NOT called here. EDMP_APEX_EN1 is an MREG
    // register that requires MCLK to be running. MCLK only starts when accel/gyro
    // are enabled in LN mode (done in configure()). Calling it here would silently
    // fail and the EDMP would keep generating 0xF0 frames into the FIFO.
    // inv_imu_edmp_disable() is called at the END of configure() instead.
    return true;
}

bool InvIMU_STM32::getFrame(IMUData& out_data) {
    if (_tail == _head) return false;
    out_data = _ring_buffer[_tail];
    __DMB();
    _tail = (uint16_t)((_tail + 1u) % kRingCapacity);
    return true;
}

void InvIMU_STM32::configure(AccelRange ar, GyroRange gr, ODR odr) {
    auto tdk_accel_fsr = toAccelFsr(ar);
    auto tdk_gyro_fsr  = toGyroFsr(gr);
    auto tdk_accel_odr = toAccelOdr(odr);
    auto tdk_gyro_odr  = toGyroOdr(odr);

    // Direct-register settings (don't need MCLK).
    inv_imu_set_accel_fsr(&_dev, tdk_accel_fsr);
    inv_imu_set_gyro_fsr(&_dev, tdk_gyro_fsr);
    inv_imu_set_accel_frequency(&_dev, tdk_accel_odr);
    inv_imu_set_gyro_frequency(&_dev, tdk_gyro_odr);

    // Bring accel/gyro into LN mode FIRST. This starts MCLK, which is required for
    // any MREG access. Without MCLK running, the LN_BW/LP_AVG writes below would
    // silently fail (their target registers IPREG_SYS1/2 live in MREG space at
    // addresses ≥0xa000), leaving the filter in an invalid state and the FIFO
    // emitting 0x8000-sentinel frames that parseFrameInto rejects.
    //
    // The TDK helpers inv_imu_set_accel_mode / inv_imu_set_gyro_mode each do a
    // read-modify-write on PWR_MGMT0. Issued back-to-back, the second read
    // observes accel_mode=OFF (the part hasn't committed the first write yet)
    // and writes 0 back into the accel field, leaving the part with
    // PWR_MGMT0=0x0C (accel=OFF, gyro=LN). Write both fields atomically instead:
    //   bits[1:0] = accel_mode (LN=3), bits[3:2] = gyro_mode (LN=3) -> 0x0F.
    spi_write(0x10, 0x0F);
    HAL_Delay(1);  // let MCLK stabilise before any MREG write

    // MREG-backed bandwidth / averaging settings — must come after MCLK is up.
    inv_imu_set_accel_ln_bw(&_dev, toAccelBwDiv(_hw.accel_bw_div));
    inv_imu_set_accel_lp_avg(&_dev, toAccelLpAvg(_hw.accel_lp_avg));
    inv_imu_set_gyro_ln_bw(&_dev, toGyroBwDiv(_hw.gyro_bw_div));
    inv_imu_set_gyro_lp_avg(&_dev, toGyroLpAvg(_hw.gyro_lp_avg));

    // Sanity-check PWR_MGMT0: bits[1:0]=accel_mode, bits[3:2]=gyro_mode, LN=0b11.
    {
        uint8_t pwr = 0xFF;
        spi_read(0x10, &pwr, 1);
        printf("PWR_MGMT0 readback=0x%02X (expect 0x0F: accel=LN gyro=LN)\r\n", pwr);
    }

    // Disable the EDMP engine so it stops writing 0xF0 EDMP-output frames into
    // the FIFO. EDMP_APEX_EN1 is an MREG register and likewise requires MCLK.
    int edmp_rc = inv_imu_edmp_disable(&_dev);
    printf("EDMP disable rc=%d\r\n", edmp_rc);
    // Read back EDMP registers to confirm the disable took effect.
    // EDMP_APEX_EN0=0x29 (direct), EDMP_APEX_EN1=0x2a (direct, bit6=edmp_enable).
    // Both should be 0x00 after reset + disable.
    {
        uint8_t en0 = 0xFF, en1 = 0xFF;
        spi_read(0x29, &en0, 1);
        spi_read(0x2a, &en1, 1);
        printf("EDMP readback: EN0=0x%02X (expect 0x00) EN1=0x%02X (expect 0x00, bit6=edmp_enable)\r\n", en0, en1);
        if (en1 & 0x40u) printf("WARNING: edmp_enable(bit6) still SET in EN1!\r\n");
    }

    float final_g = 16.0f;
    if (tdk_accel_fsr == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G) final_g = 32.0f; 
    else if (tdk_accel_fsr == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G) final_g = 16.0f;
    else if (tdk_accel_fsr == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G) final_g = 8.0f;
    else if (tdk_accel_fsr == ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G) final_g = 4.0f;
    else final_g = 2.0f;
    _accel_scale = (final_g / 524288.0f) * 9.80665f;
    // _accel_scale_hires will be overridden to 32G by configureFifo(); default to same.
    _accel_scale_hires = _accel_scale;

    float final_dps = 2000.0f;
    if (tdk_gyro_fsr == GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS) final_dps = 4000.0f;
    else if (tdk_gyro_fsr == GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS) final_dps = 2000.0f;
    else if (tdk_gyro_fsr == GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS) final_dps = 1000.0f;
    else final_dps = 500.0f;
    _gyro_scale = (final_dps / 524288.0f) * (3.14159265f / 180.0f);
    _gyro_scale_hires = _gyro_scale;
}

void InvIMU_STM32::configureFifo() {
    spi_write(0x76, 0x00);  // bank 0

    // TDK API handles BYPASS→STREAM safely and sets _dev.fifo_frame_size = 20
    inv_imu_fifo_config_t fifo_cfg = {};
    fifo_cfg.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_STREAM;
    fifo_cfg.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_MAX;
    fifo_cfg.fifo_wm_th = 32;
    fifo_cfg.hires_en   = INV_IMU_ENABLE;
    fifo_cfg.gyro_en    = INV_IMU_ENABLE;
    fifo_cfg.accel_en   = INV_IMU_ENABLE;
    // temp_en: not a field in this SDK version of inv_imu_fifo_config_t.
    // Temperature is automatically included in hires frames when accel+gyro are enabled.
    inv_imu_set_fifo_config(&_dev, &fifo_cfg);

    // inv_imu_set_fifo_config read-modify-writes FIFO_CONFIG4 (0x22) but only touches
    // fifo_tmst_fsync_en (bit1). If the hardware reset left fifo_comp_en (bit2) set,
    // that bit gets written back set — silently enabling delta-compression.
    // Compressed frames have ext_header=1 (0xF0 instead of 0x70) and a completely
    // different binary layout; parsing them as raw hi-res data produces garbage.
    // Clear bit2 (comp_en) only. Do NOT clear bit1 (fifo_tmst_fsync_en) — that
    // bit enables the 16-bit timestamp field in each FIFO frame. Without it,
    // all timestamp bytes are 0x0000.
    {
        uint8_t r22 = 0;
        spi_read(0x22, &r22, 1);
        printf("FIFO_CONFIG4 (0x22) BEFORE clear: 0x%02X  (bit2=comp_en bit1=tmst_fsync_en bit0=es0_6b_9b)\r\n", r22);
        r22 &= ~(uint8_t)0x04;  // clear fifo_comp_en (bit2) only; keep fifo_tmst_fsync_en (bit1) — it enables timestamps in FIFO frames
        spi_write(0x22, r22);
        uint8_t r22_rb = 0;
        spi_read(0x22, &r22_rb, 1);
        printf("FIFO_CONFIG4 (0x22) AFTER  clear: intended=0x%02X readback=0x%02X (bit1=tmst_fsync_en)\r\n", r22, r22_rb);
        // Verify bit1 is set — if not, force it
        if (!(r22_rb & 0x02)) {
            printf("WARNING: fifo_tmst_fsync_en NOT set, forcing...\r\n");
            r22_rb |= 0x02;
            spi_write(0x22, r22_rb);
            spi_read(0x22, &r22_rb, 1);
            printf("FIFO_CONFIG4 forced readback=0x%02X\r\n", r22_rb);
        }
    }

    // Enable the FIFO timestamp counter (TMST_EN in SMC_CONTROL_0).
    // Without this, the 16-bit timestamp field in FIFO frames is always 0,
    // causing all samples in a burst to get the same absolute timestamp.
    {
        smc_control_0_t smc;
        inv_imu_read_reg(&_dev, SMC_CONTROL_0, 1, (uint8_t *)&smc);
        smc.tmst_en = INV_IMU_ENABLE;
        inv_imu_write_reg(&_dev, SMC_CONTROL_0, 1, (uint8_t *)&smc);
        // Readback verify
        smc_control_0_t smc_rb;
        inv_imu_read_reg(&_dev, SMC_CONTROL_0, 1, (uint8_t *)&smc_rb);
        printf("SMC_CONTROL_0 tmst_en readback=%u (expect 1)\r\n", (unsigned)smc_rb.tmst_en);
    }

    // INT1 pin: push-pull, PULSE mode, active-high.
    // PULSE mode generates a ~100 µs rising edge per event and self-de-asserts.
    // LATCH mode was avoided because INT1 can fire during init (before g_imu_module
    // is non-null), leaving the pin stuck HIGH with no further rising edges for the
    // EXTI to detect — resulting in IMU=0 indefinitely.
    inv_imu_int_pin_config_t pin_cfg = {};
    pin_cfg.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
    pin_cfg.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
    pin_cfg.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
    inv_imu_set_pin_config_int(&_dev, INV_IMU_INT1, &pin_cfg);

    // Route FIFO threshold + DRDY to INT1
    inv_imu_int_state_t int_cfg = {};
    int_cfg.INV_FIFO_THS = INV_IMU_ENABLE;
    int_cfg.INV_UI_DRDY  = INV_IMU_ENABLE;
    inv_imu_set_config_int(&_dev, INV_IMU_INT1, &int_cfg);

    // ICM-45686 hires FIFO frame types:
    //   0x78 (ext_header=0): 20-bit hires, ALWAYS at ±32g / ±4000DPS (fixed by hardware).
    //   0xF0 (ext_header=1): 20-bit hires, at ACCEL_CONFIG0 / GYRO_CONFIG0 configured range.
    // _accel_scale/_gyro_scale hold the user-configured FSR (set by configure()).
    // _accel_scale_hires/_gyro_scale_hires hold the fixed 32g/4000DPS for 0x78 frames.
    _accel_scale_hires = (32.0f / 524288.0f) * 9.80665f;
    _gyro_scale_hires  = (4000.0f / 524288.0f) * (3.14159265f / 180.0f);

    // Sanity check: TDK driver sets _dev.fifo_frame_size = 20 when hires is enabled.
    // If it's anything else, the frame parser will silently produce garbage.
    if (_dev.fifo_frame_size != 20u) {
        _status_flags |= IMU_STATUS_FRAME_ALIGN_ERR;
    }

    // Flush FIFO to clear stale data accumulated during init/configure
    inv_imu_flush_fifo(&_dev);
    HAL_Delay(5);

    // Read INT1_STATUS0 to drain any pending status bits that accumulated during
    // init. INT1 is now in PULSE mode so any init-time DRDYs have already
    // self-de-asserted; this is a belt-and-suspenders clean-slate before the
    // first live interrupt after g_imu_module is set.
    uint8_t int1_status_clr = 0;
    spi_read(0x19, &int1_status_clr, 1);
}

void InvIMU_STM32::enableFsync() {
    inv_imu_adv_set_int2_pin_usage(&_dev, IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_FSYNC);
    inv_imu_adv_configure_fsync_ap_tag(&_dev, FSYNC_CONFIG0_AP_FSYNC_TEMP);

    // Enable FSYNC clock sync but keep regular FIFO timestamps.
    // inv_imu_adv_enable_fsync() sets tmst_delta_en=1 which replaces the FIFO
    // timestamp field with FSYNC delay values, breaking per-sample timestamps.
    // Instead, enable tmst_fsync_en + tmst_en WITHOUT tmst_delta_en.
    {
        smc_control_0_t smc;
        inv_imu_read_reg(&_dev, SMC_CONTROL_0, 1, (uint8_t *)&smc);
        smc.tmst_fsync_en = INV_IMU_ENABLE;
        smc.tmst_en       = INV_IMU_ENABLE;
        inv_imu_write_reg(&_dev, SMC_CONTROL_0, 1, (uint8_t *)&smc);
    }
    // Explicitly keep tmst_delta_en=0 (preserve regular FIFO timestamps)
    {
        tmst_wom_config_t twc;
        inv_imu_read_reg(&_dev, TMST_WOM_CONFIG, 1, (uint8_t *)&twc);
        twc.tmst_delta_en = INV_IMU_DISABLE;
        inv_imu_write_reg(&_dev, TMST_WOM_CONFIG, 1, (uint8_t *)&twc);
    }
}

bool InvIMU_STM32::parseFrameInto(IMUData& out, const uint8_t* p, uint8_t /*frame_size*/) {
    uint8_t header = p[0];
    bool hires = (header & 0x10u) != 0u;

    // Track frame header types for diagnostics
    // 0x78 = standard hires, 0x7C = hires+FSYNC_TAG, 0xF0 = ext_header hires
    if (header == 0x7Cu) ++_frame_count_0x7C;
    else if ((header & 0xF8u) == 0x78) ++_frame_count_0x78;
    else if (header == 0xF0) ++_frame_count_0xF0;
    else ++_frame_count_other;

    // Empty packet: no accel AND no gyro bits set.
    if (!(header & 0x60u)) {
        _status_flags |= IMU_STATUS_EMPTY_PACKET;
        return false;
    }

    // ICM-45686 FIFO frame layout (LITTLE-ENDIAN), 20 bytes, same for 0x78 and 0xF0:
    //   p[0]       = header
    //   p[1..6]    = accel XYZ (6 bytes, LE int16 each)
    //   p[7..12]   = gyro  XYZ (6 bytes, LE int16 each)
    //   p[13..14]  = temp  (2 bytes, LE int16)
    //   p[15..16]  = timestamp / padding (2 bytes)
    //   p[17..19]  = hires nibbles (3 bytes, present when hires bit is set)
    //
    // Despite ext_header=1 in 0xF0 frames, the ICM-45686 does NOT insert a second
    // header byte into the FIFO data stream.  Sensor data starts at byte[1] for
    // both frame types.

    const uint8_t* d = p + 1u;  // d[0..5]=accel LE, d[6..11]=gyro LE, d[12..13]=temp

    int16_t reg16_a[3], reg16_g[3];
    reg16_a[0] = (int16_t)((d[1] << 8) | d[0]);
    reg16_a[1] = (int16_t)((d[3] << 8) | d[2]);
    reg16_a[2] = (int16_t)((d[5] << 8) | d[4]);
    reg16_g[0] = (int16_t)((d[7] << 8) | d[6]);
    reg16_g[1] = (int16_t)((d[9] << 8) | d[8]);
    reg16_g[2] = (int16_t)((d[11] << 8) | d[10]);

    // Reject frames where any axis carries the ICM invalid-data sentinel (0x8000).
    static constexpr int16_t kInvalidFifo = (int16_t)0x8000;
    if (reg16_a[0] == kInvalidFifo || reg16_a[1] == kInvalidFifo || reg16_a[2] == kInvalidFifo ||
        reg16_g[0] == kInvalidFifo || reg16_g[1] == kInvalidFifo || reg16_g[2] == kInvalidFifo) {
        return false;
    }

    int32_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;

    if (hires) {
        // Hires frame (0x78 or 0xF0): merge 16-bit MSBs with 4-bit LSBs → 20-bit signed.
        // Hires nibbles at p[17..19] for both frame types (same 20-byte layout).
        const uint8_t* hr = p + 17;
        raw_ax = (int32_t)(((uint32_t)(uint16_t)reg16_a[0] << 4) | (hr[0] >> 4));
        raw_ay = (int32_t)(((uint32_t)(uint16_t)reg16_a[1] << 4) | (hr[1] >> 4));
        raw_az = (int32_t)(((uint32_t)(uint16_t)reg16_a[2] << 4) | (hr[2] >> 4));
        raw_gx = (int32_t)(((uint32_t)(uint16_t)reg16_g[0] << 4) | (hr[0] & 0x0F));
        raw_gy = (int32_t)(((uint32_t)(uint16_t)reg16_g[1] << 4) | (hr[1] & 0x0F));
        raw_gz = (int32_t)(((uint32_t)(uint16_t)reg16_g[2] << 4) | (hr[2] & 0x0F));
        // Sign-extend from 20 bits to 32 bits
        if (raw_ax & 0x80000) raw_ax |= (int32_t)0xFFF00000;
        if (raw_ay & 0x80000) raw_ay |= (int32_t)0xFFF00000;
        if (raw_az & 0x80000) raw_az |= (int32_t)0xFFF00000;
        if (raw_gx & 0x80000) raw_gx |= (int32_t)0xFFF00000;
        if (raw_gy & 0x80000) raw_gy |= (int32_t)0xFFF00000;
        if (raw_gz & 0x80000) raw_gz |= (int32_t)0xFFF00000;
    } else {
        // Non-hires frame: use 16-bit sensor words only.
        // Shift left 4 so the value lives in the same 20-bit scale as _accel/_gyro_scale.
        // int16_t sign propagates automatically through the cast+shift.
        raw_ax = (int32_t)reg16_a[0] << 4;
        raw_ay = (int32_t)reg16_a[1] << 4;
        raw_az = (int32_t)reg16_a[2] << 4;
        raw_gx = (int32_t)reg16_g[0] << 4;
        raw_gy = (int32_t)reg16_g[1] << 4;
        raw_gz = (int32_t)reg16_g[2] << 4;
    }

    // 0x78 (ext_header=0): 32g/4000DPS fixed hires FSR.
    // 0xF0 (ext_header=1): user-configured FSR (ACCEL_CONFIG0 / GYRO_CONFIG0).
    const bool ext_header = (header & 0x80u) != 0u;
    const float a_sc = ext_header ? _accel_scale       : _accel_scale_hires;
    const float g_sc = ext_header ? _gyro_scale        : _gyro_scale_hires;

    out.accel_x = raw_ax * a_sc;
    out.accel_y = raw_ay * a_sc;
    out.accel_z = raw_az * a_sc;
    out.gyro_x  = raw_gx * g_sc;
    out.gyro_y  = raw_gy * g_sc;
    out.gyro_z  = raw_gz * g_sc;

    // Temperature: 16-bit LE at d[12..13]  →  T_degC = raw/128 + 25
    int16_t raw_temp = (int16_t)((d[13] << 8) | d[12]);
    out.temperature = (raw_temp / 128.0f) + 25.0f + 273.15f;

    // Timestamp: for 0x78 frames p[15..16] is the dedicated 16-bit FIFO timestamp.
    // For 0xF0 frames (timestamp_bit=0) p[15..16] may be ES1 data or padding —
    // use it anyway as a monotonic proxy counter for relative unwrapping.
    uint16_t raw_ts = (uint16_t)((p[16] << 8) | p[15]);
    // Debug: dump first few raw timestamps to see if they're incrementing
    {
        static uint32_t ts_diag_count = 0;
        if (ts_diag_count < 10) {
            printf("[FIFO-TS] #%u  hdr=0x%02X  raw_ts=%u  p[14..16]=%02X %02X %02X  p[0..3]=%02X %02X %02X %02X\r\n",
                (unsigned)ts_diag_count, (unsigned)header,
                (unsigned)raw_ts,
                (unsigned)p[14], (unsigned)p[15], (unsigned)p[16],
                (unsigned)p[0], (unsigned)p[1], (unsigned)p[2], (unsigned)p[3]);
            ts_diag_count++;
        }
    }
    if (!_timestamp_initialized) {
        _last_fifo_ts          = raw_ts;
        _last_unwrapped_ts     = 0;
        _timestamp_initialized = true;
        out.timestamp_us       = 0;
        return true;
    }
    uint16_t delta = raw_ts - _last_fifo_ts;
    _last_fifo_ts = raw_ts;
    _last_unwrapped_ts += delta;
    out.timestamp_us = _last_unwrapped_ts;
    return true;
}

void InvIMU_STM32::processPendingRx() {
    if (!_rx_pending) return;
    _rx_pending = false;
    // D-Cache invalidation disabled: cache is not enabled (no SCB_EnableDCache() in main.c).
    // DMA writes are directly visible to CPU without invalidation.
    const uint16_t total_bytes = _last_dma_size;

    constexpr uint8_t frame_size = kFrameSize;
    const uint16_t n_frames = (uint16_t)(total_bytes / frame_size);
    if (n_frames == 0u) return;
    uint16_t parsed = 0u;
    for (uint16_t i = 0; i < n_frames && parsed < (uint16_t)kMaxFrames; ++i) {
        const uint16_t off = (uint16_t)(i * frame_size);
        IMUData d{};
        if (parseFrameInto(d, &_dma_rx_buffer[off], frame_size)) {
            _burst_buffer[parsed++] = d;
        }
    }
    if (parsed == 0u) return;
    const uint64_t last_fifo_us = _burst_buffer[parsed - 1u].timestamp_us;
    const uint64_t anchor_us = _dma_irq_time_us;
    if (!_fifo_to_abs_offset_initialized) {
        _fifo_to_abs_offset_us = (int64_t)anchor_us - (int64_t)last_fifo_us;
        _fifo_to_abs_offset_initialized = true;
    }

    // Snapshot the offset for this burst — all frames in THIS burst use
    // the same offset, preserving FIFO-relative spacing.
    const int64_t emit_offset_us = _fifo_to_abs_offset_us;

    for (uint16_t i = 0; i < parsed; ++i) {
        IMUData d = _burst_buffer[i];
        uint64_t abs_ts = (uint64_t)((int64_t)d.timestamp_us + emit_offset_us);
        // Last-resort monotonicity guard: use FIFO delta as repair spacing
        // instead of +1µs, preserving realistic sample intervals.
        if (_last_emitted_ts_us > 0 && abs_ts <= _last_emitted_ts_us) {
            uint64_t repair_dt_us = 156u; // default: one sample at 6400Hz
            if (i > 0) {
                const uint64_t fifo_prev = _burst_buffer[i - 1u].timestamp_us;
                const uint64_t fifo_curr = _burst_buffer[i].timestamp_us;
                if (fifo_curr > fifo_prev) {
                    repair_dt_us = fifo_curr - fifo_prev;
                }
            }
            if (repair_dt_us == 0u) repair_dt_us = 1u;
            abs_ts = _last_emitted_ts_us + repair_dt_us;
            _monotonic_repair_count++;
            _status_flags |= IMU_STATUS_TIMESTAMP_DESYNC;
        }
        _last_emitted_ts_us = abs_ts;
        d.timestamp_us = abs_ts;
        applyAlignment(d);
        const uint16_t next_head = (uint16_t)((_head + 1u) % kRingCapacity);
        if (next_head == _tail) {
            _tail = (uint16_t)((_tail + 1u) % kRingCapacity);
            _ring_overflow_count++;
            _status_flags |= IMU_STATUS_FIFO_OVERFLOW;
        }
        _ring_buffer[_head] = d;
        __DMB();
        _head = next_head;
        _ring_write_count++;
    }

    // Update offset estimate for FUTURE bursts only (slewed, not instant).
    // The current burst was emitted with the snapshot; this correction
    // applies to the next burst.
    {
        const int64_t predicted_irq = (int64_t)last_fifo_us + _fifo_to_abs_offset_us;
        const int64_t err = (int64_t)anchor_us - predicted_irq;
        _last_offset_err_us = (int32_t)err;

        // Convergence-aware outlier gate:
        // Phase 1 (boot): gate disabled, slew runs freely to converge from ~-43ms to 0.
        // Phase 2 (converged): once err stays below threshold for N consecutive bursts,
        //   gate activates. Any subsequent spike (e.g. BURN transition) with |err| > gate
        //   threshold is rejected, preventing offset contamination.
        constexpr int64_t kConvergedThresholdUs = 1000;
        constexpr int64_t kOutlierGateUs = 5000;
        constexpr uint32_t kConvergedStreakRequired = 50;  // ~1s
        const int64_t abs_err = (err >= 0) ? err : -err;

        const bool reject = _offset_converged && (abs_err > kOutlierGateUs);

        if (reject) {
            _offset_update_reject_count++;
            const int32_t abs_max = (_max_rejected_err_us >= 0) ? _max_rejected_err_us : -_max_rejected_err_us;
            if ((int32_t)abs_err > abs_max) {
                _max_rejected_err_us = (int32_t)err;
            }
        } else {
            // Bounded/smoothed correction: divide by 4, clamp to ±32µs per burst.
            constexpr int64_t kCorrectionDivisor = 4;
            constexpr int64_t kMaxCorrectionPerBurstUs = 32;
            constexpr int64_t kErrDeadbandUs = 10;

            if (err > kErrDeadbandUs || err < -kErrDeadbandUs) {
                int64_t correction = err / kCorrectionDivisor;
                if (correction > kMaxCorrectionPerBurstUs) {
                    correction = kMaxCorrectionPerBurstUs;
                } else if (correction < -kMaxCorrectionPerBurstUs) {
                    correction = -kMaxCorrectionPerBurstUs;
                }
                _fifo_to_abs_offset_us += correction;
            }

            // Convergence tracking (only when not rejecting)
            if (abs_err < kConvergedThresholdUs) {
                if (_converge_streak < UINT32_MAX) _converge_streak++;
                if (_converge_streak >= kConvergedStreakRequired) {
                    _offset_converged = true;
                }
            } else {
                _converge_streak = 0;
            }
        }

        if (err > 500 || err < -500) {
            _status_flags |= IMU_STATUS_TIMESTAMP_DESYNC;
        }
    }
}

void InvIMU_STM32::onInterrupt(uint64_t irq_us) {
    _irq_pending = true;
    _irq_time_us = (irq_us != 0u) ? irq_us : now_us(_hw.use_dwt_timestamps);
}
//POLLING HELPER FUNCTION
int InvIMU_STM32::spi_read_fifo(uint8_t reg, uint8_t* data, uint16_t len) {
    if (len == 0) return 0;

    // STM32H7 full-duplex SPI (2LINES) does not reliably support
    // HAL_SPI_Transmit + HAL_SPI_Receive as separate calls in one CS assertion:
    // the RX FIFO retains the byte received during the address phase, causing
    // HAL_SPI_Receive to fail. Use a single HAL_SPI_TransmitReceive instead.
    // Safety: max len = (kRawBufferSize/kFrameSize)*kFrameSize = 2040,
    // so len+1 = 2041 always fits inside _dma_rx_buffer[kRawBufferSize=2048].
    _tx_dummy_buf[0] = reg | 0x80u;
    // _tx_dummy_buf[1..len] are pre-zeroed dummy clock bytes.

    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_RESET);
    int rc = HAL_SPI_TransmitReceive(_hw.hspi, _tx_dummy_buf, data,
                                     (uint16_t)(len + 1u),
                                     INV_IMU_SPI_FIFO_RX_TIMEOUT_MS);
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);

    if (rc != HAL_OK) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return -1;
    }
    // data[0] is the dummy byte received during the address phase; shift it out.
    std::memmove(data, data + 1, len);
    return 0;
}

void InvIMU_STM32::tick() {
    _status_flags &= ~(uint32_t)IMU_STATUS_TIMESTAMP_DESYNC;
    if (_rx_pending) processPendingRx();
    if (!_irq_pending) return;
    // Guard BEFORE consuming _irq_pending.
    // If we consumed it while DMA was busy, the EXTI rising edge that fired during
    // spi_read(0x19) would be silently lost → streaming stops after one burst.
    if (_dma_busy || _rx_pending) return;
    _irq_pending = false;

    // Read INT1_STATUS0 to acknowledge the interrupt.  INT1 is in PULSE mode so the
    // pin already self-de-asserted; this read is diagnostic (tells us DRDY vs FIFO_THS)
    // and also clears the status bits so they don't accumulate across calls.
    uint8_t int1_status_clr = 0;
    spi_read(0x19, &int1_status_clr, 1);

    if (spi_write(REG_BANK_SEL, 0x00) != 0) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return;
    }

    // ICM-45686 FIFO_COUNT (0x12-0x13) is in FRAMES, not bytes.
    // Errata AN-000364 §2.2: read twice and use the second value.
    uint8_t counts[2];
    if (spi_read(REG_FIFO_COUNTH, counts, 2) != 0) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return;
    }
    if (spi_read(REG_FIFO_COUNTH, counts, 2) != 0) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return;
    }

    // ICM-45686 FIFO_COUNT register map (LE): 0x12 = FIFO_COUNT_0 (LSB), 0x13 = FIFO_COUNT_1 (MSB)
    uint16_t frame_count = (uint16_t)((counts[1] << 8) | counts[0]);
    if (frame_count == 0) return;

    // Errata AN-000364 §2.2: in STREAM mode read M-1 frames to avoid a torn frame.
    if (frame_count > 1u) frame_count--;

    // Convert frame count to byte count for the DMA transfer.
    uint16_t count = frame_count * kFrameSize;

    if (count > kRawBufferSize) {
        _status_flags |= IMU_STATUS_FIFO_OVERFLOW;
        count = (uint16_t)((kRawBufferSize / kFrameSize) * kFrameSize);
    }

    if (count == 0u) return;

    _last_dma_size = count;
    _dma_irq_time_us = _irq_time_us;

#ifndef UNIT_TEST_ENV
    const bool dma_hw_ready = (_hw.hspi != nullptr) && (_hw.hspi->hdmarx != nullptr);
#else
    const bool dma_hw_ready = false;
#endif
    const bool use_dma_path = _hw.use_dma && dma_hw_ready;

    if (!use_dma_path) {
        // Blocking SPI FIFO read.
        // If this returns != 0, the HAL likely returned HAL_BUSY because
        // CubeMX has DMA streams configured for this SPI peripheral.
        // Fix: remove DMA request rows for SPI_RX/TX in the .ioc file,
        // or call HAL_DMA_DeInit(hspi->hdmarx) after MX_SPIx_Init().
        int rc = spi_read_fifo(REG_FIFO_DATA, _dma_rx_buffer, count);
        if (rc != 0) {
            _status_flags |= IMU_STATUS_SPI_ERROR;
            static uint32_t _spi_fail_cnt = 0;
            if ((_spi_fail_cnt++ % 1000u) == 0u) {
                printf("[IMU] SPI blocking read FAIL #%lu  count=%u  "
                       "HAL_State=%u  HAL_ErrCode=0x%lX  hdmarx=%s\r\n",
                       (unsigned long)_spi_fail_cnt, (unsigned)count,
                       (unsigned)(_hw.hspi ? _hw.hspi->State : 0),
                       (unsigned long)(_hw.hspi ? _hw.hspi->ErrorCode : 0xDEAD),
                       dma_hw_ready ? "configured" : "null");
            }
            return;
        }
        _rx_pending = true;
        processPendingRx();
        return;
    }

    _dma_busy = true;

    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_RESET);
    uint8_t reg = REG_FIFO_DATA | 0x80;

    if (HAL_SPI_Transmit(_hw.hspi, &reg, 1, INV_IMU_SPI_TX_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
        _dma_busy = false;
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return;
    }

    if (HAL_SPI_Receive_DMA(_hw.hspi, _dma_rx_buffer, count) != HAL_OK) {
        HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
        _dma_busy = false;
        _status_flags |= IMU_STATUS_SPI_ERROR;
        static uint32_t _dma_fail_cnt = 0;
        printf("DMA_START_FAIL #%lu count=%u\r\n", (unsigned long)++_dma_fail_cnt, (unsigned)count);
        return;
    }
}

void InvIMU_STM32::onDmaComplete() {
    if (!_dma_busy) {
        return;
    }
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);
    const uint16_t total_bytes = _last_dma_size;
    _invalidate_size = (total_bytes > 0u) ? ((uint32_t)total_bytes + 31u) & ~31u : (uint32_t)kRawBufferSize;
    _dma_busy = false;
    _rx_pending = true;
}

bool InvIMU_STM32::ping() {
    uint8_t who_am_i = 0;
    if (spi_read(REG_WHO_AM_I, &who_am_i, 1) != 0) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return false;
    }
    printf("DEBUG: IMU WHO_AM_I Read = 0x%02X\r\n", who_am_i);
    const bool ok = (who_am_i == INV_IMU_WHO_AM_I_VAL);
    if (!ok) _status_flags |= IMU_STATUS_WHOAMI_MISMATCH;
    return ok;
}
int InvIMU_STM32::spi_write(uint8_t reg, uint8_t data) {
    uint8_t tx[2] = { reg, data };
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_RESET);
    int rc = HAL_SPI_Transmit(_hw.hspi, tx, 2, INV_IMU_SPI_TX_TIMEOUT_MS);
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
    // Build a TX buffer: [reg | 0x80] + [dummy bytes]
    uint8_t tx_buf[65] = {};
    tx_buf[0] = reg | 0x80;

    uint8_t rx_buf[65] = {};
    const uint32_t total = len + 1u;

    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_RESET);
    int rc = HAL_SPI_TransmitReceive(_hw.hspi, tx_buf, rx_buf, total, 50);
    HAL_GPIO_WritePin(_hw.cs_port, _hw.cs_pin, GPIO_PIN_SET);

    if (rc != HAL_OK) {
        _status_flags |= IMU_STATUS_SPI_ERROR;
        return -1;
    }
    // Data starts at rx_buf[1], skipping the dummy byte clocked during address phase
    std::memcpy(data, &rx_buf[1], len);
    return 0;
}

} // namespace InvIMU
} // namespace Drivers
