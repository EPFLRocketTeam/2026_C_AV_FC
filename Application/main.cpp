// Core/Inc/main.h includes stm32hal.h which pulls in C++ headers —
// include it outside extern "C" (it has its own C++ guards).
#include "Core/Inc/main.h"
#include "Application/app_timebase.h"
#include "Application/app_printf.h"
#include "Modules/baro_module.hpp"
#include "Modules/imu_modlue.hpp"
#include "Modules/gps_module.hpp"
#include "Application/Modules/battery_module.hpp"
#include "plume_driver.hpp"
#include "Modules/sd_logger.hpp"
#include "Modules/fc_temp_module.hpp"
#include "Drivers/Buzzer/buzzer.hpp"
#include "Application/Kalman/kalman_health.hpp"
#include "Application/Config/config.hpp"
#include "Application/FlightControl/fc_shell.hpp"
#include "Application/app_logger.hpp"
#include "Application/app_perf.h"
#include "Drivers/Camera/CameraPlatform.hpp"


// Forward-declared — defined in av_state.cpp to avoid BMP390 header clash.
void fsm_tick(void);

extern "C" {
#include "Application/main.h"
#include "Application/Kalman/kalman_process.h"
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"
#ifdef UNIT_TEST_ENV
#include "Drivers/BMP390/Impl/BMP390_mock.h"
#else
#include "Drivers/BMP390/BMP390.hpp"
#endif
#include  "Drivers/UBX_GPS/ubx_gps_interface.h"

// extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;
extern UART_HandleTypeDef huart6;
extern I2C_HandleTypeDef hi2c2;
extern SD_HandleTypeDef hsd1;

using Drivers::InvIMU::Config;
using Drivers::InvIMU::IMUData;
using Drivers::InvIMU::IMU_STATUS_OK;
using Drivers::InvIMU::InvIMU_Interface;
using Drivers::InvIMU::InvIMU_STM32;
using Drivers::BMP390::BaroData;

// ── FSYNC PWM on PD14 (TIM4_CH3, AF2) ────────────────────────────────────────
// Generates 6400 Hz FSYNC tags, NOT an external sampling/timestamp clock.
// The IMU counters remain free-running; FifoClock estimates their rate/phase
// against FIFO-count observations on the common application timebase.
static TIM_HandleTypeDef htim4_fsync;

static bool fsync_pwm_init(uint32_t freq_hz) {
    // Enable clocks
    __HAL_RCC_TIM4_CLK_ENABLE();
    // GPIOD clock already enabled by CubeMX MX_GPIO_Init

    // Configure PD14 as TIM4_CH3 (AF2)
    GPIO_InitTypeDef gpio{};
    gpio.Pin       = GPIO_PIN_14;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &gpio);

    // TIM4 clock = 2 × APB1 = 240 MHz (APB1 prescaler > 1)
    // ARR = 240 000 000 / freq_hz − 1
    const uint32_t tim_clk = 240000000u;
    const uint32_t arr     = (tim_clk / freq_hz) - 1u;

    htim4_fsync.Instance               = TIM4;
    htim4_fsync.Init.Prescaler         = 0;
    htim4_fsync.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4_fsync.Init.Period            = arr;
    htim4_fsync.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4_fsync.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4_fsync) != HAL_OK) return false;

    TIM_OC_InitTypeDef oc{};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = arr / 2u;   // 50 % duty
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim4_fsync, &oc, TIM_CHANNEL_3) != HAL_OK) return false;

    return HAL_TIM_PWM_Start(&htim4_fsync, TIM_CHANNEL_3) == HAL_OK;
}


AppImuRingBuffer imuData1;
AppImuRingBuffer imuData2;
AppImuRingBuffer imuData3;
AppImuRingBuffer imuData4;

RingBuffer<GpsBasicFixData, 100> gpsData;

FCTemperatureModule fcTemperatureModule;

// ── Fake GNSS injection for pipeline testing ──────────────────────────────────
// Enable with -DFAKE_GNSS_ENABLE=1.  Injects synthetic 3D-fix data at 16 Hz
// directly into the gpsData ring buffer, exercising the full GNSS→ESKF→rewind
// pipeline without a real GPS receiver.
// #ifndef FAKE_GNSS_ENABLE
// #define FAKE_GNSS_ENABLE 1
// #endif

#if FAKE_GNSS_ENABLE
namespace {

// Simulated launch site: 45.5088° N, -73.5878° W, 30m MSL (Montreal)
constexpr int32_t kFakeLat        = 455088000;   // 1e-7 deg
constexpr int32_t kFakeLon        = -735878000;  // 1e-7 deg
constexpr int32_t kFakeHMSL       = 30000;       // mm
constexpr int32_t kFakeHeight     = 30000;       // mm (ellipsoid ≈ MSL here)
constexpr uint32_t kFakeHAcc      = 1500;        // mm (1.5 m CEP)
constexpr uint32_t kFakeVAcc      = 3000;        // mm (3 m)
constexpr uint32_t kFakeSAcc      = 500;         // mm/s
constexpr uint8_t  kFakeNumSV     = 12;
constexpr uint16_t kFakePDOP      = 120;         // 1.2 (×100)

// Start with 1 Hz to stress-test the rewind pipeline safely.
// Each measurement triggers an individual 100ms rewind.
// Increase gradually: 1 → 4 → 8 → 16 Hz.
#ifndef FAKE_GNSS_RATE_HZ
#define FAKE_GNSS_RATE_HZ 16
#endif
constexpr uint32_t kFakeGpsPeriodUs = 1000000u / FAKE_GNSS_RATE_HZ;

struct FakeGnssState {
    uint64_t next_inject_us = 0;    // next injection time
    uint64_t last_pps_us    = 0;    // last simulated PPS pulse time
    uint32_t itow_ms        = 0;    // GPS time-of-week (ms), wraps at 604800000
    uint32_t sample_count   = 0;
};
FakeGnssState g_fake_gnss;

void fake_gnss_inject(uint64_t now_us) {
    // Don't inject until well after force-flight (15s) to avoid
    // preflight buffer accumulation and early-liftoff edge cases.
    constexpr uint64_t kFakeGnssStartDelayUs = 15000000ULL;  // 15 seconds
    if (now_us < kFakeGnssStartDelayUs) return;

    if (now_us < g_fake_gnss.next_inject_us) return;
    g_fake_gnss.next_inject_us = now_us + kFakeGpsPeriodUs;

    // PPS: set to 0 so each measurement uses its own reception timestamp.
    // This triggers individual ~100ms rewinds per measurement
    // (via ESKF_DEFAULT_GPS_DELAY_US = -100000).

    GpsBasicFixData fix{};
    fix.timestamp_us     = now_us;
    fix.pps_timestamp_us = 0;  // use reception time → unique rewinds

    // Time fields (synthetic)
    fix.iTOW  = g_fake_gnss.itow_ms;
    fix.year  = 2025; fix.month = 5; fix.day = 16;
    fix.hour  = 12; fix.min = 0;
    fix.sec   = static_cast<uint8_t>((g_fake_gnss.itow_ms / 1000) % 60);
    fix.nano  = static_cast<int32_t>((g_fake_gnss.itow_ms % 1000) * 1000000);

    // Validity
    fix.valid.validDate     = true;
    fix.valid.validTime     = true;
    fix.valid.fullyResolved = true;
    fix.valid.validMag      = false;

    // Fix info
    fix.fixType         = GpsFixType::FIX_3D;
    fix.flags.gnssFixOK = true;
    fix.flags.diffSoln  = false;
    fix.flags.headVehValid = false;
    fix.flags.psmState  = GpsPowerSaveMode::NOT_ACTIVE;
    fix.flags.carrSoln  = GpsCarrierPhaseStatus::NONE;
    fix.numSV           = kFakeNumSV;

    // Position (constant)
    fix.lat    = kFakeLat;
    fix.lon    = kFakeLon;
    fix.height = kFakeHeight;
    fix.hMSL   = kFakeHMSL;
    fix.hAcc   = kFakeHAcc;
    fix.vAcc   = kFakeVAcc;

    // Velocity (zero — static on ground)
    fix.velN = 0; fix.velE = 0; fix.velD = 0;
    fix.gSpeed  = 0;
    fix.headMot = 0;
    fix.sAcc    = kFakeSAcc;
    fix.headAcc = 1000000; // 10° (× 1e-5)

    fix.pDOP = kFakePDOP;

    gpsData.append(fix);
    g_fake_gnss.itow_ms += (kFakeGpsPeriodUs / 1000); // advance iTOW by ~62 ms
    if (g_fake_gnss.itow_ms >= 604800000u) g_fake_gnss.itow_ms = 0;
    g_fake_gnss.sample_count++;
}

} // namespace
#endif // FAKE_GNSS_ENABLE
void manual_test_buzzer_set_buzzer_2 (bool status) {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, status ? GPIO_PIN_SET : GPIO_PIN_RESET);

#ifdef DEBUG
    app_printf("[BUZZER] Set at time %d: %d\r\n", HAL_GetTick(), status);
#endif
}

RingBuffer<BaroData, 100> baroData1;
RingBuffer<BaroData, 100> baroData2;
RingBuffer<BaroData, 100> baroData3;
RingBuffer<BaroData, 100> baroData4;

#ifndef APP_IMU_USE_DMA
#define APP_IMU_USE_DMA 0u
#endif
#if APP_ENABLE_DCACHE && APP_IMU_USE_DMA
#error "SPI DMA needs a separately audited non-cacheable buffer policy"
#endif
#ifndef APP_IMU_FAST_FIFO
// The shared bus stays at 8 MHz for registers/BMP390. Only ICM FIFO bursts
// temporarily use 16 MHz (below its 24 MHz limit) to leave estimator CPU time.
#define APP_IMU_FAST_FIFO 1u
#endif

// Set to the matching GPIO pin number (e.g. GPIO_PIN_13) when EXTI is wired.
// Keep at 0 when no hardware interrupt line is available.
#ifndef APP_IMU1_INT_PIN
#define APP_IMU1_INT_PIN ICM_INT4_Pin
#endif

#ifndef APP_IMU2_INT_PIN
#define APP_IMU2_INT_PIN ICM_INT2_Pin
#endif

#ifndef APP_IMU3_INT_PIN
#define APP_IMU3_INT_PIN ICM_INT3_Pin
#endif

#ifndef APP_IMU4_INT_PIN
#define APP_IMU4_INT_PIN ICM_INT1_Pin
#endif

bool g_liftoff_detection_allowed = false;

namespace {

SDCardInterface g_sd_interface;
const size_t g_sd_arena_length = 256 * 1024;
/* CPU-owned arena in D2 SRAM1/2. Optional caching is safe because IDMA never
 * reads this memory: the driver copies it into the non-cacheable AXI bounce.
 * NOTE: SDMMC1 IDMA cannot access RAM_D2 directly.  The Plume driver
 * uses a 32KB bounce buffer in AXI SRAM (RAM_D1) for each DMA write.
 * Moved from RAM_D1 (nearly full) to absorb SD card GC pauses
 * of up to 250ms at 1 MB/s write rate without dropping records. */
uint8_t g_sd_arena_buffer[g_sd_arena_length] __attribute__((section(".ram_d2_bss"), aligned(32)));
bool g_sd_logging_active = false;  // Set after successful init+open
bool g_gps_init_ok       = false;
bool g_buzzer_finished   = false;
static uint32_t g_buzzer_finished_ms = 0;
static constexpr uint32_t kLiftoffArmDelayMs = 3000; // 3s margin after buzzer
SdLogger g_sd_logger;

// Full-rate raw sensor logging callbacks (forwarded to g_sd_logger)
void imu_raw_log_callback(size_t sensor_index, const Drivers::InvIMU::IMUData* samples, size_t count) {
    g_sd_logger.logImuRawBatch(sensor_index, samples, count);
}
void baro_raw_log_callback(size_t sensor_index, const Drivers::BMP390::BaroData& sample) {
    g_sd_logger.logBaroRaw(sensor_index, sample);
}
void ubx_raw_log_callback(const uint8_t* frame, uint16_t frame_len) {
    g_sd_logger.logUbxRaw(frame, frame_len);
}

// Logging decimation: write DataDump every N ms
constexpr uint32_t kLogIntervalMs = 10;  // ~100 Hz
constexpr uint32_t kMetricsIntervalMs = 1000;  // Health+metrics every 1s
uint32_t g_last_log_ms = 0;
uint32_t g_last_metrics_ms = 0;
flight_computer::State g_last_fsm_state = flight_computer::INIT;

// App metrics tracking
struct AppMetricsTracker {
    uint32_t loop_sum_us = 0;
    uint32_t loop_min_us = UINT32_MAX;
    uint32_t loop_max_us = 0;
    uint32_t loop_count = 0;
    uint32_t imu_batches = 0;
    uint32_t kalman_last_us = 0;
    uint32_t kalman_sum_us = 0;
    uint32_t kalman_max_us = 0;
    uint32_t kalman_count = 0;
    uint32_t sd_tick_last_us = 0;
    uint32_t sd_tick_max_us = 0;

    void recordLoop(uint32_t us) {
        loop_sum_us += us;
        if (us < loop_min_us) loop_min_us = us;
        if (us > loop_max_us) loop_max_us = us;
        loop_count++;
    }
    void recordKalman(uint32_t us) {
        kalman_last_us = us;
        kalman_sum_us += us;
        if (us > kalman_max_us) kalman_max_us = us;
        kalman_count++;
    }
    void recordSdTick(uint32_t us) {
        sd_tick_last_us = us;
        if (us > sd_tick_max_us) sd_tick_max_us = us;
    }
    void reset() {
        loop_sum_us = 0; loop_min_us = UINT32_MAX; loop_max_us = 0; loop_count = 0;
        imu_batches = 0;
        kalman_last_us = 0; kalman_sum_us = 0; kalman_max_us = 0; kalman_count = 0;
        sd_tick_last_us = 0; sd_tick_max_us = 0;
    }
};
AppMetricsTracker g_metrics_tracker;

ImuModule<4>* g_imu_module = nullptr;
uint8_t g_imu_healthy[4] = {0u, 0u, 0u, 0u};
uint32_t g_imu_status_flags[4] = {IMU_STATUS_OK, IMU_STATUS_OK, IMU_STATUS_OK, IMU_STATUS_OK};
uint8_t g_baro_healthy[4] = {0u, 0u, 0u, 0u};
uint32_t g_baro_status_flags[4] = {
    Drivers::BMP390::BMP390_STATUS_OK,
    Drivers::BMP390::BMP390_STATUS_OK,
    Drivers::BMP390::BMP390_STATUS_OK,
    Drivers::BMP390::BMP390_STATUS_OK,
};

#if APP_GPS_UPDATE_RATE_HZ > 0
constexpr uint16_t kGpsRateMs = static_cast<uint16_t>(
    (1000u + (APP_GPS_UPDATE_RATE_HZ / 2u)) / APP_GPS_UPDATE_RATE_HZ);
#else
constexpr uint16_t kGpsRateMs = 1000u;
#endif

constexpr uint16_t kImuIntPins[4] = {
    APP_IMU1_INT_PIN,
    APP_IMU2_INT_PIN,
    APP_IMU3_INT_PIN,
    APP_IMU4_INT_PIN,
};

Config makeImuConfig(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin) {
    Config cfg{};
    cfg.hspi = hspi;
    cfg.cs_port = cs_port;
    cfg.cs_pin = cs_pin;
    cfg.use_dwt_timestamps = true;
    cfg.use_dma = (APP_IMU_USE_DMA != 0u);
#if APP_IMU_FAST_FIFO
    // SPI4/5 kernel clocks are HSI64: FIFO=16MHz, registers/barometers=8MHz.
    // ICM-45686 DS-000577 section 3.5 permits up to 24MHz at this VDDIO.
    cfg.fifo_spi_prescaler = SPI_BAUDRATEPRESCALER_4;
#endif
    return cfg;
}

#ifndef UNIT_TEST_ENV
Drivers::BMP390::BMP390_SDK::Config makeBaroConfig(SPI_HandleTypeDef* hspi,
                                                   GPIO_TypeDef* cs_port,
                                                   uint16_t cs_pin) {
    Drivers::BMP390::BMP390_SDK::Config cfg{};
    cfg.hspi = hspi;
    cfg.cs_port = cs_port;
    cfg.cs_pin = cs_pin;
    return cfg;
}
#endif

struct SuperLoopContext {

    Config imu_cfg1 = makeImuConfig(&hspi4, ICM_CS4_GPIO_Port, ICM_CS4_Pin);
    Config imu_cfg2 = makeImuConfig(&hspi5, ICM_CS2_GPIO_Port, ICM_CS2_Pin);
    Config imu_cfg3 = makeImuConfig(&hspi4, ICM_CS3_GPIO_Port, ICM_CS3_Pin);
    Config imu_cfg4 = makeImuConfig(&hspi5, ICM_CS1_GPIO_Port, ICM_CS1_Pin);

    InvIMU_STM32 invImu1{imu_cfg1};
    InvIMU_STM32 invImu2{imu_cfg2};
    InvIMU_STM32 invImu3{imu_cfg3};
    InvIMU_STM32 invImu4{imu_cfg4};

    InvIMU_Interface* invArr[4] = {&invImu1, &invImu2, &invImu3, &invImu4};
    AppImuRingBuffer* ringArr[4] = {&imuData1, &imuData2, &imuData3, &imuData4};
    ImuModule<4> imuModule{invArr, ringArr};
    buzzer::Buzzer<13> buzzer;

#ifdef UNIT_TEST_ENV
    Drivers::BMP390::BMP390_Mock baro1{};
    Drivers::BMP390::BMP390_Mock baro2{};
    Drivers::BMP390::BMP390_Mock baro3{};
    Drivers::BMP390::BMP390_Mock baro4{};
#else
    Drivers::BMP390::BMP390_SDK::Config baro_cfg1 =
        makeBaroConfig(&hspi5, BMP_CS1_GPIO_Port, BMP_CS1_Pin);
    Drivers::BMP390::BMP390_SDK::Config baro_cfg2 =
        makeBaroConfig(&hspi5, BMP_CS2_GPIO_Port, BMP_CS2_Pin);
    Drivers::BMP390::BMP390_SDK::Config baro_cfg3 =
        makeBaroConfig(&hspi4, BMP_CS3_GPIO_Port, BMP_CS3_Pin);
    Drivers::BMP390::BMP390_SDK::Config baro_cfg4 =
        makeBaroConfig(&hspi4, BMP_CS4_GPIO_Port, BMP_CS4_Pin);

    Drivers::BMP390::BMP390_SDK baro1{baro_cfg1};
    Drivers::BMP390::BMP390_SDK baro2{baro_cfg2};
    Drivers::BMP390::BMP390_SDK baro3{baro_cfg3};
    Drivers::BMP390::BMP390_SDK baro4{baro_cfg4};
#endif
    Drivers::BMP390::BMP390_Interface* baroArr[4] = {
        &baro1, &baro2, &baro3, &baro4};
    RingBuffer<BaroData, 100>* baroRing[4] = {
        &baroData1, &baroData2, &baroData3, &baroData4};
//    Drivers::BMP390::BMP390_Interface* baroArr[1] = {
//            &baro1 };
//        RingBuffer<BaroData, 100>* baroRing[1] = {
//            &baroData1 };
    BaroModule<4> baroModule{baroArr, baroRing};

    UbxGpsInterface gps{&huart6, kGpsRateMs};
    UbxGpsInterface* gpsArr[1] = {&gps};
    RingBuffer<GpsBasicFixData, 100>* gpsRing[1] = {&gpsData};
    GpsModule gpsModule{gpsArr, gpsRing};

    BatteryModule batteryModule{&hi2c2};

    bool setup_done = false;
    bool ready = false;
};

SuperLoopContext g_superloop{};

} // namespace

extern "C" void app_on_imu_exti(uint16_t gpio_pin) {
    if (g_imu_module == nullptr || gpio_pin == 0u) {
        return;
    }

    const uint64_t irq_us = app_timebase_now_us();
    const uint32_t now_ms = HAL_GetTick();
    for (size_t i = 0; i < 4; ++i) {
        if (kImuIntPins[i] == 0u) {
            continue;
        }
        if (kImuIntPins[i] == gpio_pin) {
            g_imu_module->onImuInterrupt(i, now_ms, irq_us);
            return;
        }
    }
}

extern "C" void app_on_imu_spi_rx_complete(SPI_HandleTypeDef* hspi) {
    if (g_imu_module == nullptr || hspi == nullptr) {
        return;
    }

    // Current flight-test wiring has the enabled IMU on SPI4.
    if (hspi != &hspi4) {
        return;
    }

    g_imu_module->onSpiRxComplete(hspi);
}

extern "C" uint8_t app_imu_sensor_healthy(uint8_t sensor_index) {
    if (sensor_index >= 4u) {
        return 0u;
    }
    return g_imu_healthy[sensor_index];
}

extern "C" uint32_t app_imu_sensor_status_flags(uint8_t sensor_index) {
    if (sensor_index >= 4u) {
        return IMU_STATUS_OK;
    }
    return g_imu_status_flags[sensor_index];
}

extern "C" void app_imu_frame_counts(uint32_t* h78, uint32_t* hF0, uint32_t* other) {
    if (g_imu_module == nullptr) {
        *h78 = *hF0 = *other = 0;
        return;
    }
    // Read from the first (only active) IMU driver via the interface.
    auto& ctx = g_superloop;
    *h78 = ctx.invArr[0]->frameCount0x78();
    *hF0 = ctx.invArr[0]->frameCount0xF0();
    *other = ctx.invArr[0]->frameCountOther();
}

extern "C" void app_imu_ts_diagnostics(uint32_t* h7C, uint32_t* mono_repairs, int32_t* last_err,
                                       uint32_t* reject_count, int32_t* max_rejected_err,
                                       uint32_t* spi_fifo_fail, uint32_t* spi_not_ready,
                                       uint32_t* burst_count, uint8_t* gate_armed) {
    auto& imu = g_superloop.invImu1;
    *h7C = imu.frameCount0x7C();
    *mono_repairs = imu.monotonicRepairCount();
    *last_err = imu.lastOffsetErrUs();
    *reject_count = imu.offsetUpdateRejectCount();
    *max_rejected_err = imu.maxRejectedErrUs();
    *spi_fifo_fail = imu.spiFifoReadFailCount();
    *spi_not_ready = imu.spiStateNotReadyCount();
    *burst_count = imu.offsetBurstCount();
    *gate_armed = imu.offsetGateArmed() ? 1u : 0u;
}

extern "C" uint8_t app_baro_sensor_healthy(uint8_t sensor_index) {
    if (sensor_index >= 4u) {
        return 0u;
    }
    return g_baro_healthy[sensor_index];
}

extern "C" uint32_t app_baro_sensor_status_flags(uint8_t sensor_index) {
    if (sensor_index >= 4u) {
        return Drivers::BMP390::BMP390_STATUS_WHOAMI_MISMATCH;
    }
    return g_baro_status_flags[sensor_index];
}


// ── Standalone raw SPI baro test ──────────────────────────────────────────────
// Bypasses the Bosch SDK and baro module entirely. Just does a raw SPI chip ID
// read (reg 0x00) for each of the 4 BMP390 sensors.
// This tells us if the SPI bus and CS pins work at the lowest possible level.
static void baro_raw_spi_test() {
    struct BaroTestCfg {
        SPI_HandleTypeDef* hspi;
        GPIO_TypeDef* cs_port;
        uint16_t cs_pin;
        const char* name;
    };
    BaroTestCfg baros[4] = {
        {&hspi5, BMP_CS1_GPIO_Port, BMP_CS1_Pin, "BARO1(SPI5)"},
        {&hspi5, BMP_CS2_GPIO_Port, BMP_CS2_Pin, "BARO2(SPI5)"},
        {&hspi4, BMP_CS3_GPIO_Port, BMP_CS3_Pin, "BARO3(SPI4)"},
        {&hspi4, BMP_CS4_GPIO_Port, BMP_CS4_Pin, "BARO4(SPI4)"},
    };

    app_printf("[RAW-BARO-TEST] Starting raw SPI chip ID reads...\r\n");
    for (int i = 0; i < 4; i++) {
        auto& b = baros[i];
        // BMP390 SPI read: [reg|0x80] [dummy] [data]
        // For chip ID (reg 0x00): tx=[0x80, 0x00, 0x00], expect rx=[xx, xx, 0x60]
        uint8_t tx[3] = {0x80, 0x00, 0x00};
        uint8_t rx[3] = {0xAA, 0xBB, 0xCC}; // fill with known pattern

        // Ensure CS is HIGH before we start
        HAL_GPIO_WritePin(b.cs_port, b.cs_pin, GPIO_PIN_SET);
        HAL_Delay(1);

        HAL_GPIO_WritePin(b.cs_port, b.cs_pin, GPIO_PIN_RESET);
        HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(b.hspi, tx, rx, 3, 50);
        HAL_GPIO_WritePin(b.cs_port, b.cs_pin, GPIO_PIN_SET);

        app_printf("[RAW-BARO-TEST] %s: HAL=%d SPI_state=%u SPI_err=0x%lX rx=[%02X %02X %02X] chip_id=0x%02X %s\r\n",
               b.name, (int)st,
               (unsigned)b.hspi->State, (unsigned long)b.hspi->ErrorCode,
               rx[0], rx[1], rx[2], rx[2],
               (rx[2] == 0x60) ? "OK" : "MISMATCH!");
    }
    app_printf("[RAW-BARO-TEST] Done.\r\n");
}

SdLogger& app_get_sd_logger () {
    return g_sd_logger;   
}

// ── GPS UART reception via circular DMA ─────────────────────────────────
// Set up here rather than in CubeMX (.ioc / stm32h7xx_hal_msp.c) so that
// regenerating the CubeMX code neither drops nor duplicates it. DMA1_Stream0
// is otherwise unused. No DMA or USART6 interrupt is needed: the GPS driver
// reads the DMA counter from the super loop. The ring has a dedicated
// non-cacheable SRAM3 carve-out, reachable by DMA1 even with arena caching.
#ifndef APP_GPS_UART_DMA
#define APP_GPS_UART_DMA 1
#endif
#ifndef APP_GPS_DMA_RX_BUFFER_SIZE
#define APP_GPS_DMA_RX_BUFFER_SIZE 2048u  // ~0.7 s of UBX output at 16 Hz
#endif

#if APP_GPS_UART_DMA
static DMA_HandleTypeDef g_hdma_usart6_rx;
alignas(32) static uint8_t g_gps_dma_rx_buffer[APP_GPS_DMA_RX_BUFFER_SIZE]
    __attribute__((section(".gps_dma_rx")));

static bool app_gps_start_dma_rx(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    g_hdma_usart6_rx.Instance                 = DMA1_Stream0;
    g_hdma_usart6_rx.Init.Request             = DMA_REQUEST_USART6_RX;
    g_hdma_usart6_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    g_hdma_usart6_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    g_hdma_usart6_rx.Init.MemInc              = DMA_MINC_ENABLE;
    g_hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_usart6_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    g_hdma_usart6_rx.Init.Mode                = DMA_CIRCULAR;
    g_hdma_usart6_rx.Init.Priority            = DMA_PRIORITY_LOW;
    g_hdma_usart6_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&g_hdma_usart6_rx) != HAL_OK) {
        return false;
    }
    __HAL_LINKDMA(&huart6, hdmarx, g_hdma_usart6_rx);
    return g_superloop.gps.startDmaRx(g_gps_dma_rx_buffer,
                                      sizeof(g_gps_dma_rx_buffer)) == GpsStatus::OK;
}
#endif

extern "C" void app_super_loop_setup(void) {
    if (g_superloop.setup_done) {
        return;
    }
    g_superloop.setup_done = true;

    app_timebase_init();

    if (!g_superloop.imuModule.init()) {
        g_imu_module = nullptr;
        g_superloop.ready = false;
        return;
    }
    g_imu_module = &g_superloop.imuModule;

    cameraSetup();

    fcTemperatureModule.init();

    // ── FSYNC tags; the IMU oscillators remain free-running ─────────────────
    // Start 6400 Hz PWM on PD14 → ICM-45686 INT2 (FSYNC input), then tell
    // the IMU to use it. Order matters: clock must be running before the IMU
    // is told to listen to it, otherwise the IMU sees no edges.
    if (fsync_pwm_init(6400u)) {
        if (!g_superloop.imuModule.sensorFailed(0)) g_superloop.invImu1.enableFsync();
        if (!g_superloop.imuModule.sensorFailed(1)) g_superloop.invImu2.enableFsync();
        if (!g_superloop.imuModule.sensorFailed(2)) g_superloop.invImu3.enableFsync();
        if (!g_superloop.imuModule.sensorFailed(3)) g_superloop.invImu4.enableFsync();
        app_printf("[APP] FSYNC PWM started on PD14 @ 6400 Hz\r\n");
    } else {
        app_printf("[APP] WARNING: FSYNC PWM init failed — FSYNC tags unavailable\r\n");
    }
    app_printf("[APP] Initializing SD Card...\n");
    if (hsd1.Instance == NULL) {
        app_printf("[APP] SD Card not initialized (hsd1.Instance == NULL).\n");
    } else if (!g_sd_interface.init_sd_card(&hsd1, g_sd_arena_buffer, g_sd_arena_length)) {
        app_printf("[APP] Failure of init SD card (non-fatal, continuing).\n");
    } else {
        app_printf("Opening file...\n");
        if (!g_sd_interface.open_file()) {
            app_printf("[APP] Failure of open on SD card (non-fatal).\n");
        } else {
            app_printf("[APP] SD card file opened OK.\n");
            g_sd_logging_active = true;
            g_sd_logger.init(&g_sd_interface);
            eskf::setEskfLogger(&g_sd_logger);
            app_printf("[APP] SD logger active, ESKF logger connected.\n");

            /* Dump SDMMC bus config for diagnostics */
            uint32_t clkcr = hsd1.Instance->CLKCR;
            app_printf("[SD-CFG] CLKCR=0x%08lX  ClkDiv=%lu  BusWide=%lu  HwFlow=%lu\r\n",
                   (unsigned long)clkcr,
                   (unsigned long)(clkcr & 0x3FF),            /* CLKDIV bits[9:0] */
                   (unsigned long)((clkcr >> 14) & 0x3),      /* WIDBUS bits[15:14] */
                   (unsigned long)((clkcr >> 17) & 0x1));      /* HWFC_EN bit[17]   */
        }
    }

    // ── Baro init diagnostics ──────────────────────────────────────────────
    // Print SPI handle states after IMU init (IMU uses SPI4, baros use SPI4+SPI5)
    app_printf("[APP] SPI4 state=%u err=0x%lX  SPI5 state=%u err=0x%lX\r\n",
           (unsigned)hspi4.State, (unsigned long)hspi4.ErrorCode,
           (unsigned)hspi5.State, (unsigned long)hspi5.ErrorCode);

    // Blocking CPU SPI accesses are cache-coherent; no global cache flush.
    app_printf("[APP] Starting barometer initialization...\r\n");

    // Raw SPI test: bypasses SDK, directly reads chip IDs from all 4 baros
    baro_raw_spi_test();

    g_superloop.baroModule.setTriggerCallback(kalman_note_baro_trigger);
    if (!g_superloop.baroModule.init()) {
        // Non-fatal: system can operate with degraded baro (voting handles it)
        app_printf("WARNING: No barometers initialized\r\n");
    }

    if (!g_superloop.batteryModule.init()) {
        printf("WARNING: not all battery rails initialized\r\n");
    }

    bool gps_state = g_superloop.gpsModule.init();
#if FAKE_GNSS_ENABLE   /////
    app_printf("[APP] FAKE_GNSS_ENABLE=1: skipping real GPS init, using synthetic 16Hz GNSS\r\n");
    // Don't init real GPS — no hardware attached.
#else
    // GNSS is log-only for the estimator: a receiver failure must not stop
    // the super-loop (Kalman, FSM, SD). Its status is reported in the boot
    // marker.
    g_gps_init_ok = gps_state;
    if (!gps_state) {
        app_printf("[APP] WARNING: GPS init failed (non-fatal, GPS disabled)\r\n");
    }
#endif

#if APP_GPS_UART_DMA && !FAKE_GNSS_ENABLE
    if (gps_state) {
        if (app_gps_start_dma_rx()) {
            app_printf("[APP] GPS UART reception on circular DMA\r\n");
        } else {
            app_printf("[APP] WARNING: GPS DMA start failed, polling the UART\r\n");
        }
    }
#endif

    // ── Full-rate raw sensor logging callbacks ────────────────────────────
    if (g_sd_logging_active) {
        g_superloop.imuModule.setRawLogCallback(imu_raw_log_callback);
        g_superloop.baroModule.setRawLogCallback(baro_raw_log_callback);
        g_superloop.gps.setRawUbxCallback(ubx_raw_log_callback);
        app_printf("[APP] Full-rate raw sensor logging enabled.\r\n");

        // Log boot marker to delimit this session on SD
        uint8_t imu_ok = 0;
        for (size_t i = 0; i < 4; ++i) {
            if (!g_superloop.imuModule.sensorFailed(i)) imu_ok++;
        }
        uint8_t baro_ok = 0;
        for (size_t i = 0; i < 4; ++i) {
            if (g_superloop.baroModule.sensorInit(i)) baro_ok++;
        }
        g_sd_logger.logBootMarker(imu_ok, baro_ok, gps_state);
        app_printf("[APP] Boot marker logged to SD.\r\n");
    }

    g_superloop.ready = true;
#ifdef DEBUG
    app_printf("-------------------%d, %d, %d,%d,%d,%d,%d,%d,%d-------------------\r\n," , g_superloop.imuModule.sensorFailed(0), g_superloop.imuModule.sensorFailed(1),
            g_superloop.imuModule.sensorFailed(2), g_superloop.imuModule.sensorFailed(3),
            g_superloop.baroModule.sensorInit(0), g_superloop.baroModule.sensorInit(1),
            g_superloop.baroModule.sensorInit(2), g_superloop.baroModule.sensorInit(3),
            gps_state);
#endif
    /* g_superloop.buzzer.tick(HAL_GetTick());
    g_superloop.buzzer.start(
        HAL_GetTick(),
        manual_test_buzzer_set_buzzer_2,
        1, 1, !g_superloop.imuModule.sensorFailed(0), !g_superloop.imuModule.sensorFailed(1),
        !g_superloop.imuModule.sensorFailed(2), !g_superloop.imuModule.sensorFailed(3),
        g_superloop.baroModule.sensorInit(0), g_superloop.baroModule.sensorInit(1),
        g_superloop.baroModule.sensorInit(2), g_superloop.baroModule.sensorInit(3),
        gps_state , 1, 1
    );*/

}

static int nb_superloops = 0;
static int nb_consumed = 0;
static uint32_t lastRatioComputationTime = 0;
static int nb_consumed_since_last_poll = 0;
#if APP_PERF_TRACE
// One line per report period: per-IMU frames, timestamp gaps and samples lost
// in them, largest frame step, hardware FIFO fill (see ImuModule /
// InvIMU_Interface::FifoStats), and samples lost since boot (still meaningful
// after a period where the serial output was not read).
static void app_print_imu_acquisition(void) {
    static uint32_t total_lost[4] = {};
    static uint32_t total_app_drops[4] = {};
    char line[672];
    int n = snprintf(line, sizeof(line), "[IMU-ACQ]");
    for (size_t i = 0; i < 4; ++i) {
        const auto a = g_superloop.imuModule.takeAcqStats(i);
        const auto f = g_superloop.imuModule.takeFifoStats(i);
        total_lost[i] += a.lost;
        total_app_drops[i] += a.ring_overwrites;
        if (n > 0 && n < (int)sizeof(line)) {
            n += snprintf(line + n, sizeof(line) - n,
                          " %u:fr=%lu gap=%lu lost=%lu tot=%lu dt=%luus hwm=%u cap=%lu full=%lu app_drop=%lu app_tot=%lu",
                          (unsigned)i, (unsigned long)a.frames, (unsigned long)a.gaps,
                          (unsigned long)a.lost, (unsigned long)total_lost[i],
                          (unsigned long)a.max_dt_us,
                          (unsigned)f.count_hwm, (unsigned long)f.capped_reads,
                          (unsigned long)f.full_flags, (unsigned long)a.ring_overwrites,
                          (unsigned long)total_app_drops[i]);
        }
    }
    app_printf("%s\r\n", line);
}
#endif

extern "C" void app_super_loop_iterate(void) {
    app_perf_loop_mark();
    uint64_t perf_t0 = app_perf_begin();
	FC_Shell_Tick();
    app_perf_end(APP_PERF_SHELL, perf_t0);

    perf_t0 = app_perf_begin();
    RUN_EVERY(100)
        config::internal::tick();

    cameraTick();

    fcTemperatureModule.tick();

	//app_printf("Buzzer advancing ---------------------------------------------\r\n");
	g_superloop.buzzer.tick(HAL_GetTick());
	g_superloop.batteryModule.update(HAL_GetTick());
    // A buzzer that was never started (start() is commented out in setup)
    // produces no vibrations to wait for; without this, liftoff detection
    // would never be enabled.
    const bool buzzer_quiet =
        !g_superloop.buzzer.is_started() || g_superloop.buzzer.is_finished();
    if (buzzer_quiet && !g_buzzer_finished) {
        g_buzzer_finished = true;
        g_buzzer_finished_ms = HAL_GetTick();
    }

    // Allow liftoff detection only after buzzer vibrations have settled
    if (g_buzzer_finished && !g_liftoff_detection_allowed &&
        (HAL_GetTick() - g_buzzer_finished_ms >= kLiftoffArmDelayMs)) {
        g_liftoff_detection_allowed = true;
        app_printf("[LIFTOFF] Detection enabled (%lums after buzzer)\r\n",
               (unsigned long)kLiftoffArmDelayMs);
    }
    app_perf_end(APP_PERF_PERIPH, perf_t0);

    if (!g_superloop.ready) {
        return;
    }

    perf_t0 = app_perf_begin();

    // ── SD Card Logging ────────────────────────────────────────────────────
    // Write DataDump at ~62.5 Hz + on FSM transitions.
    // tick() is always called to drain the ring buffer via DMA.
    if (g_sd_logging_active) {
        RUN_EVERY(100) {
            app_printf("[SD] wr=%lu fail=%lu arena=%lu/%lu maxWr=%luus ticks=%lu disk=%lluKB imu=%lu/%lu(%luKB)\r\n",
                   (unsigned long)g_sd_logger.writeCount(),
                   (unsigned long)g_sd_logger.writeFailCount(),
                   (unsigned long)g_sd_interface.arena_used_bytes(),
                   (unsigned long)g_sd_interface.arena_total_bytes(),
                   (unsigned long)g_sd_logger.maxWriteTimeUs(),
                   (unsigned long)g_sd_logger.tickCount(),
                   (uint64_t)(g_sd_interface.disk_size_remaining() / 1024),
                   (unsigned long)g_sd_logger.imuBatchCount(),
                   (unsigned long)g_sd_logger.imuBatchFail(),
                   (unsigned long)(g_sd_logger.imuBytesOk() / 1024));
        }

        const uint32_t now_ms = HAL_GetTick();
        bool should_log = (now_ms - g_last_log_ms >= kLogIntervalMs);

        // Detect FSM state change (force immediate log on transition)
        const flight_computer::DataDump& dump = flight_computer::GOATStore::get_instance().get();
        if (dump.av_state != g_last_fsm_state) {
            g_sd_logger.logFsmTransition(g_last_fsm_state, dump.av_state);
            g_last_fsm_state = dump.av_state;
            should_log = true;  // Force DataDump on transition
        }

        if (should_log) {
            g_sd_logger.logDataDump(&dump, sizeof(dump));
            g_last_log_ms = now_ms;
        }

        // Periodic health + app metrics (1 Hz)
        if (now_ms - g_last_metrics_ms >= kMetricsIntervalMs) {
            g_last_metrics_ms = now_ms;

            // SD health record
            g_sd_logger.logSdHealth();

            // App metrics record
            const KalmanHealthSnapshot kh = KalmanHealthStore::instance().get();
            uint32_t fire_count, solo_flush, stale_flush;
            kalman_get_group_stats(&fire_count, &solo_flush, &stale_flush);

            // Compute per-interval deltas for cumulative counters
            static uint32_t prev_fire_count = 0;
            static uint32_t prev_solo_flush = 0;
            static uint32_t prev_stale_flush = 0;
            static uint32_t prev_total_events = 0;
            static uint32_t prev_catchup_yields = 0;
            static uint32_t prev_baro_corrections = 0;
            static uint32_t prev_reset_generation = 0;
            const uint32_t reset_generation = kalman_reset_generation();
            if (reset_generation != prev_reset_generation) {
                prev_fire_count = prev_solo_flush = prev_stale_flush = 0;
                prev_total_events = prev_catchup_yields = prev_baro_corrections = 0;
                prev_reset_generation = reset_generation;
            }
            const uint32_t delta_fire  = fire_count  - prev_fire_count;
            const uint32_t delta_solo  = solo_flush  - prev_solo_flush;
            const uint32_t delta_stale = stale_flush - prev_stale_flush;
            const uint32_t delta_events = kh.total_events_processed - prev_total_events;
            const uint32_t delta_budget_yields = kh.catchup_budget_yields - prev_catchup_yields;
            const uint32_t delta_baro = kh.baro_corrections - prev_baro_corrections;
            prev_fire_count  = fire_count;
            prev_solo_flush  = solo_flush;
            prev_stale_flush = stale_flush;
            prev_total_events = kh.total_events_processed;
            prev_catchup_yields = kh.catchup_budget_yields;
            prev_baro_corrections = kh.baro_corrections;

            SdLogAppMetrics m{};
            m.publish_us      = (uint32_t)app_timebase_now_us();
            m.loop_avg_us     = g_metrics_tracker.loop_count > 0
                                    ? g_metrics_tracker.loop_sum_us / g_metrics_tracker.loop_count
                                    : 0;
            m.loop_min_us     = (g_metrics_tracker.loop_min_us == UINT32_MAX) ? 0 : g_metrics_tracker.loop_min_us;
            m.loop_max_us     = g_metrics_tracker.loop_max_us;
            m.loop_count      = g_metrics_tracker.loop_count;
            m.imu_batches     = kh.imu_samples_consumed;
            m.imu_drops       = kh.yieldable_imu_drops;
            m.kalman_time_us  = kh.last_kalman_loop_us;
            m.kalman_avg_us   = g_metrics_tracker.kalman_count > 0
                                    ? g_metrics_tracker.kalman_sum_us / g_metrics_tracker.kalman_count
                                    : 0;
            m.kalman_max_us   = g_metrics_tracker.kalman_max_us;
            m.sd_tick_time_us = g_metrics_tracker.sd_tick_last_us;
            m.sd_tick_max_us  = g_metrics_tracker.sd_tick_max_us;
            m.catchup_events  = delta_events;
            m.stale_skip_count = delta_stale;
            m.group_fire_count = delta_fire;
            m.solo_flush_count = delta_solo;
            // New diagnostic fields
            m.catchup_budget_yields = delta_budget_yields;
            m.kalman_behind_us      = kh.kalman_behind_us;
            m.baro_corrections      = delta_baro;
            m.imu_fifo_max_samples  = 0;  // TODO: track from IMU driver
            for (size_t i = 0; i < 4; ++i) {
                if (kh.imu_ring_hwm[i] > m.imu_fifo_max_samples)
                    m.imu_fifo_max_samples = kh.imu_ring_hwm[i];
            }

            g_sd_logger.logAppMetrics(m);
            g_metrics_tracker.reset();

            // Debug print SD health + app metrics (1 Hz)
            {
                SdTimingStats st = sd_timing_snapshot();
                uint32_t avg_cycle = (st.dma_count > 0) ? (uint32_t)(st.sum_cycle_us / st.dma_count) : 0;
                app_printf("[SD-T] dma=%lu err=%lu(0x%lX) blk=%lu batch=%lu-%lu xfer=%luus prog=%luus cyc=%lu/%luus\r\n",
                       (unsigned long)st.dma_count,
                       (unsigned long)st.dma_error_count,
                       (unsigned long)st.last_error_code,
                       (unsigned long)st.total_blocks,
                       (unsigned long)st.min_batch,
                       (unsigned long)st.max_batch,
                       (unsigned long)st.max_xfer_us,
                       (unsigned long)st.max_prog_us,
                       (unsigned long)avg_cycle,
                       (unsigned long)st.max_cycle_us);
            }
            app_printf("[APP] loop=%lu/%lu/%luus(%lu) kal=%lu/%luus sd=%luus "
                   "grp=%lu solo=%lu stale=%lu\r\n",
                   (unsigned long)m.loop_min_us,
                   (unsigned long)m.loop_avg_us,
                   (unsigned long)m.loop_max_us,
                   (unsigned long)m.loop_count,
                   (unsigned long)m.kalman_avg_us,
                   (unsigned long)m.kalman_max_us,
                   (unsigned long)m.sd_tick_max_us,
                   (unsigned long)delta_fire,
                   (unsigned long)delta_solo,
                   (unsigned long)delta_stale);
        }
    }
    app_perf_end(APP_PERF_SD_LOG, perf_t0);
    {
        const uint64_t t0 = app_timebase_now_us();
        g_sd_interface.tick();
        const uint32_t sd_us = static_cast<uint32_t>(app_timebase_now_us() - t0);
        g_metrics_tracker.recordSdTick(sd_us);
        g_sd_logger.notifyTick();
        app_perf_end(APP_PERF_SD_TICK, t0);
    }

    const uint64_t iteration_start_us = app_timebase_now_us();
    const uint32_t iter_now_ms = HAL_GetTick();
    perf_t0 = app_perf_begin();
    g_superloop.imuModule.update(iter_now_ms);

    for (size_t i = 0; i < 4; ++i) {
        g_imu_healthy[i] = g_superloop.imuModule.sensorHealthy(i) ? 1u : 0u;
        g_imu_status_flags[i] = g_superloop.imuModule.sensorStatusFlags(i);
    }

    nb_superloops ++;
    RUN_EVERY(1000) {
    	app_printf("[IMU STATUS] \n");
    	for (size_t i = 0; i < 4; i ++) {
    		printf(" IMU %d: healthy=%d status=%d\n", (int) i, (int) g_imu_healthy[i], (int) g_imu_status_flags[i]);
    	}
    	app_printf("PRODUCED: %d\n", nb_consumed);
        app_printf("OVER TIME: %d\n", nb_superloops);
    	app_printf("[BARO STATUS] \n");
    	for (size_t i = 0; i < 4; i ++) {
    		printf(" BARO %d: healthy=%d status=%d\n", (int) i, (int) g_baro_healthy[i], (int) g_baro_status_flags[i]);
    	}
        nb_superloops = 0;
        nb_consumed = 0;
    }


    size_t producedCount = g_superloop.imuModule.takeProducedCount();
    nb_consumed += producedCount;
    app_perf_end(APP_PERF_IMU, perf_t0);

    perf_t0 = app_perf_begin();
    g_superloop.baroModule.update(iter_now_ms);
    for (size_t i = 0; i < 4; ++i) {
        g_baro_healthy[i] = g_superloop.baroModule.sensorHealthy(i) ? 1u : 0u;
        g_baro_status_flags[i] = g_superloop.baroModule.sensorStatusFlags(i);
    }
    (void)g_superloop.baroModule.takeProducedCount();
#if APP_GPS_UART_DMA && !FAKE_GNSS_ENABLE
    RUN_EVERY(1000) {
        const auto rx = g_superloop.gps.takeRxStats();
        app_printf("[GPS] bytes=%lu pvt=%lu ore=%lu\r\n", (unsigned long)rx.bytes,
                   (unsigned long)rx.pvt, (unsigned long)rx.overruns);
    }
#endif
#if !FAKE_GNSS_ENABLE
    if (g_gps_init_ok) {
        g_superloop.gpsModule.update(iter_now_ms);
    }
#endif

#if FAKE_GNSS_ENABLE
    fake_gnss_inject(iteration_start_us);
#endif
    app_perf_end(APP_PERF_BARO_GPS, perf_t0);

    const uint64_t kal_start_us = app_timebase_now_us();
    (void)kalman_loop();
    const uint64_t kal_end_us = app_timebase_now_us();
    g_metrics_tracker.recordKalman(static_cast<uint32_t>(kal_end_us - kal_start_us));
    app_perf_end(APP_PERF_KALMAN, kal_start_us);

    /* Second SD drain point: halves the max latency between DMA completion
     * checks (from one full loop iteration ~3-5ms down to ~1-2ms). */
    perf_t0 = app_perf_begin();
    g_sd_interface.tick();
    app_perf_end(APP_PERF_SD_TICK, perf_t0);

    // ── FSM tick ────────────────────────────────────────────────────────
    // Runs after kalman_loop so that imu_liftoff_detected is fresh.
    perf_t0 = app_perf_begin();
    fsm_tick();
    app_perf_end(APP_PERF_FSM, perf_t0);

    const uint64_t iteration_end_us = app_timebase_now_us();
    const uint64_t elapsed_us = iteration_end_us - iteration_start_us;
    kalman_note_main_loop_iteration_us(static_cast<uint32_t>(elapsed_us));
    g_metrics_tracker.recordLoop(static_cast<uint32_t>(elapsed_us));

#if APP_PERF_TRACE
    RUN_EVERY(APP_PERF_REPORT_MS) {
        perf_t0 = app_perf_begin();
        app_perf_print();
        app_print_imu_acquisition();
        simple_radio_print_stats();
        app_perf_end(APP_PERF_REPORT, perf_t0);
    }
#endif
}

extern "C" uint64_t app_get_remaining_disk_size (void) {
    return g_sd_interface.disk_size_remaining();
}
extern "C" uint64_t app_get_sd_fail_count (void) {
    return g_sd_logger.writeFailCount() + g_sd_logger.imuBatchFail();
}
extern "C" float app_get_current_imu_rate (void) {
    float ratio = 0;
    if (lastRatioComputationTime != 0) {
        uint32_t deltaTime = HAL_GetTick() - lastRatioComputationTime;

        ratio = ((float) nb_consumed_since_last_poll) / ((float) deltaTime);
    }

    flight_computer::GOATStore::get_instance()
        .sensStatusStore
        .set_imu_rate(ratio);

    nb_consumed_since_last_poll = 0;
    lastRatioComputationTime = HAL_GetTick();

    return ratio;
}
extern "C" void app_open_parachute () {
    app_set_pyro_status(1, true);
    app_set_pyro_status(2, true);
    app_set_pyro_status(3, true);
    app_set_pyro_status(4, true);
}
extern "C" void app_set_pyro_status (int pyro_id, bool enabled) {
    auto &store = flight_computer::GOATStore::get_instance().vehiculeOverviewStore;
    auto target = enabled ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if (pyro_id == 1) {
        store.set_pyro_ch1_on(enabled);
        HAL_GPIO_WritePin(PYROS_1_GPIO_Port, PYROS_1_Pin, target);
    } else if (pyro_id == 2) {
        store.set_pyro_ch2_on(enabled);
        HAL_GPIO_WritePin(PYROS_2_GPIO_Port, PYROS_2_Pin, target);
    } else if (pyro_id == 3) {
        store.set_pyro_ch3_on(enabled);
        HAL_GPIO_WritePin(PYROS_3_GPIO_Port, PYROS_3_Pin, target);
    } else if (pyro_id == 4) {
        store.set_pyro_ch4_on(enabled);
        HAL_GPIO_WritePin(PYROS_4_GPIO_Port, PYROS_4_Pin, target);
    }
}

extern "C" void app_on_state_becomes_init () {
    g_sd_logger.setLogRate(false);
}
extern "C" void app_on_state_becomes_armed () {
    g_sd_logger.setLogRate(true);
}
