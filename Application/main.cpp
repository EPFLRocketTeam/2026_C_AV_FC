// Core/Inc/main.h includes stm32hal.h which pulls in C++ headers —
// include it outside extern "C" (it has its own C++ guards).
#include "Core/Inc/main.h"
#include "Application/app_timebase.h"
#include "Modules/baro_module.hpp"
#include "Modules/imu_modlue.hpp"
#include "Modules/gps_module.hpp"
#include "plume_driver.hpp"
#include "Modules/sd_logger.hpp"
#include "Drivers/Buzzer/buzzer.hpp"
#include "Application/Kalman/kalman_health.hpp"


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
extern SD_HandleTypeDef hsd1;

using Drivers::InvIMU::Config;
using Drivers::InvIMU::IMUData;
using Drivers::InvIMU::IMU_STATUS_OK;
using Drivers::InvIMU::InvIMU_Interface;
using Drivers::InvIMU::InvIMU_STM32;
using Drivers::BMP390::BaroData;

// ── FSYNC PWM on PD14 (TIM4_CH3, AF2) ────────────────────────────────────────
// Generates a 6400 Hz square wave that the ICM-45686 uses as an external time
// reference (FSYNC). This locks IMU FIFO timestamps to the MCU crystal,
// eliminating the ~0.1 % clock drift between the IMU's internal RC oscillator
// and the MCU's PLL-derived DWT cycle counter.
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

// ── Fake GNSS injection for pipeline testing ──────────────────────────────────
// Enable with -DFAKE_GNSS_ENABLE=1.  Injects synthetic 3D-fix data at 16 Hz
// directly into the gpsData ring buffer, exercising the full GNSS→ESKF→rewind
// pipeline without a real GPS receiver.
#ifndef FAKE_GNSS_ENABLE
#define FAKE_GNSS_ENABLE 0
#endif

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
    printf("[BUZZER] Set at time %d: %d\r\n", HAL_GetTick(), status);
#endif
}

RingBuffer<BaroData, 100> baroData1;
RingBuffer<BaroData, 100> baroData2;
RingBuffer<BaroData, 100> baroData3;
RingBuffer<BaroData, 100> baroData4;

#ifndef APP_IMU_USE_DMA
#define APP_IMU_USE_DMA 0u
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

namespace {

SDCardInterface g_sd_interface;
const size_t g_sd_arena_length = 64 * 1024;
uint8_t g_sd_arena_buffer[g_sd_arena_length];
bool g_sd_logging_active = false;  // Set after successful init+open
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

    printf("[RAW-BARO-TEST] Starting raw SPI chip ID reads...\r\n");
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

        printf("[RAW-BARO-TEST] %s: HAL=%d SPI_state=%u SPI_err=0x%lX rx=[%02X %02X %02X] chip_id=0x%02X %s\r\n",
               b.name, (int)st,
               (unsigned)b.hspi->State, (unsigned long)b.hspi->ErrorCode,
               rx[0], rx[1], rx[2], rx[2],
               (rx[2] == 0x60) ? "OK" : "MISMATCH!");
    }
    printf("[RAW-BARO-TEST] Done.\r\n");
}

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

    // ── FSYNC: lock IMU timestamps to MCU crystal ──────────────────────────
    // Start 6400 Hz PWM on PD14 → ICM-45686 INT2 (FSYNC input), then tell
    // the IMU to use it. Order matters: clock must be running before the IMU
    // is told to listen to it, otherwise the IMU sees no edges.
    if (fsync_pwm_init(6400u)) {
        if (!g_superloop.imuModule.sensorFailed(0)) g_superloop.invImu1.enableFsync();
        if (!g_superloop.imuModule.sensorFailed(1)) g_superloop.invImu2.enableFsync();
        if (!g_superloop.imuModule.sensorFailed(2)) g_superloop.invImu3.enableFsync();
        if (!g_superloop.imuModule.sensorFailed(3)) g_superloop.invImu4.enableFsync();
        printf("[APP] FSYNC PWM started on PD14 @ 6400 Hz\r\n");
    } else {
        printf("[APP] WARNING: FSYNC PWM init failed — IMU timestamps may drift\r\n");
    }
    printf("[APP] Initializing SD Card...\n");
    if (hsd1.Instance == NULL) {
        printf("[APP] SD Card not initialized (hsd1.Instance == NULL).\n");
    } else if (!g_sd_interface.init_sd_card(&hsd1, g_sd_arena_buffer, g_sd_arena_length)) {
        printf("[APP] Failure of init SD card (non-fatal, continuing).\n");
    } else {
        printf("Opening file...\n");
        if (!g_sd_interface.open_file()) {
            printf("[APP] Failure of open on SD card (non-fatal).\n");
        } else {
            printf("[APP] SD card file opened OK.\n");
            g_sd_logging_active = true;
            g_sd_logger.init(&g_sd_interface);
            eskf::setEskfLogger(&g_sd_logger);
            printf("[APP] SD logger active, ESKF logger connected.\n");
        }
    }

    // ── Baro init diagnostics ──────────────────────────────────────────────
    // Print SPI handle states after IMU init (IMU uses SPI4, baros use SPI4+SPI5)
    printf("[APP] SPI4 state=%u err=0x%lX  SPI5 state=%u err=0x%lX\r\n",
           (unsigned)hspi4.State, (unsigned long)hspi4.ErrorCode,
           (unsigned)hspi5.State, (unsigned long)hspi5.ErrorCode);

    // D-cache clean+invalidate before baro init as a safety measure.
    // If D-cache holds stale data for the SPI handle structs (AXI SRAM),
    // this ensures a clean state. Costs ~microseconds, runs once.
    SCB_CleanInvalidateDCache();
    __DSB();
    __ISB();
    printf("[APP] D-cache clean+invalidate done, starting baro init...\r\n");

    // Raw SPI test: bypasses SDK, directly reads chip IDs from all 4 baros
    baro_raw_spi_test();

    g_superloop.baroModule.setTriggerCallback(kalman_note_baro_trigger);
    if (!g_superloop.baroModule.init()) {
        // Non-fatal: system can operate with degraded baro (voting handles it)
        printf("WARNING: No barometers initialized\r\n");
    }

    bool gps_state = g_superloop.gpsModule.init();
#if FAKE_GNSS_ENABLE
    printf("[APP] FAKE_GNSS_ENABLE=1: skipping real GPS init, using synthetic 16Hz GNSS\r\n");
    // Don't init real GPS — no hardware attached.
#else
    if (!gps_state) {
        g_superloop.ready = false;
        return;
    }
#endif

    // ── Full-rate raw sensor logging callbacks ────────────────────────────
    if (g_sd_logging_active) {
        g_superloop.imuModule.setRawLogCallback(imu_raw_log_callback);
        g_superloop.baroModule.setRawLogCallback(baro_raw_log_callback);
        g_superloop.gps.setRawUbxCallback(ubx_raw_log_callback);
        printf("[APP] Full-rate raw sensor logging enabled.\r\n");

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
        printf("[APP] Boot marker logged to SD.\r\n");
    }

    g_superloop.ready = true;
#ifdef DEBUG
    printf("-------------------%d, %d, %d,%d,%d,%d,%d,%d,%d-------------------\r\n," , g_superloop.imuModule.sensorFailed(0), g_superloop.imuModule.sensorFailed(1),
            g_superloop.imuModule.sensorFailed(2), g_superloop.imuModule.sensorFailed(3),
            g_superloop.baroModule.sensorInit(0), g_superloop.baroModule.sensorInit(1),
            g_superloop.baroModule.sensorInit(2), g_superloop.baroModule.sensorInit(3),
            gps_state);
#endif
    static const buzzer::ModuleId kBuzzerIds[13] = {
        {0, 1},                          // Buzzer Test  : ■
        {0, 2},                          // Superloop    : ■■
        {1, 1}, {1, 2}, {1, 3}, {1, 4}, // IMU 1-4      : ▬+N shorts
        {2, 1}, {2, 2}, {2, 3}, {2, 4}, // BMP 1-4      : ▬▬+N shorts
        {0, 3},                          // GPS          : ■■■
        {0, 4},                          // Kalman       : ■■■■
        {3, 0, true},                    // Ready        : shave and a haircut
    };
    g_superloop.buzzer.configure(kBuzzerIds);
    g_superloop.buzzer.tick(HAL_GetTick());
    g_superloop.buzzer.start(
        HAL_GetTick(),
        manual_test_buzzer_set_buzzer_2,
        1, 1, !g_superloop.imuModule.sensorFailed(0), !g_superloop.imuModule.sensorFailed(1),
        !g_superloop.imuModule.sensorFailed(2), !g_superloop.imuModule.sensorFailed(3),
        g_superloop.baroModule.sensorInit(0), g_superloop.baroModule.sensorInit(1),
        g_superloop.baroModule.sensorInit(2), g_superloop.baroModule.sensorInit(3),
        gps_state , 1, 1
    );

}

extern "C" void app_super_loop_iterate(void) {
	//printf("Buzzer advancing ---------------------------------------------\r\n");
	g_superloop.buzzer.tick(HAL_GetTick());
    if (!g_superloop.ready) {
        return;
    }

    // ── SD Card Logging ────────────────────────────────────────────────────
    // Write DataDump at ~62.5 Hz + on FSM transitions.
    // tick() is always called to drain the ring buffer via DMA.
    if (g_sd_logging_active) {
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

            SdLogAppMetrics m{};
            m.publish_ms      = now_ms;
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
            m.kalman_max_us   = kh.max_kalman_loop_us;
            m.sd_tick_time_us = g_metrics_tracker.sd_tick_last_us;
            m.sd_tick_max_us  = g_metrics_tracker.sd_tick_max_us;
            m.catchup_events  = 0;  // placeholder
            m.stale_skip_count = stale_flush;
            m.group_fire_count = fire_count;
            m.solo_flush_count = solo_flush;

            g_sd_logger.logAppMetrics(m);
            g_metrics_tracker.reset();
        }
    }
    {
        const uint64_t t0 = app_timebase_now_us();
        g_sd_interface.tick();
        const uint32_t sd_us = static_cast<uint32_t>(app_timebase_now_us() - t0);
        g_metrics_tracker.recordSdTick(sd_us);
    }

    const uint64_t iteration_start_us = app_timebase_now_us();
    const uint32_t iter_now_ms = HAL_GetTick();
    g_superloop.imuModule.update(iter_now_ms);

    for (size_t i = 0; i < 4; ++i) {
        g_imu_healthy[i] = g_superloop.imuModule.sensorHealthy(i) ? 1u : 0u;
        g_imu_status_flags[i] = g_superloop.imuModule.sensorStatusFlags(i);
    }

    (void)g_superloop.imuModule.takeProducedCount();
    g_superloop.baroModule.update(iter_now_ms);
    for (size_t i = 0; i < 4; ++i) {
        g_baro_healthy[i] = g_superloop.baroModule.sensorHealthy(i) ? 1u : 0u;
        g_baro_status_flags[i] = g_superloop.baroModule.sensorStatusFlags(i);
    }
    (void)g_superloop.baroModule.takeProducedCount();
#if !FAKE_GNSS_ENABLE
    g_superloop.gpsModule.update(iter_now_ms);
#endif

#if FAKE_GNSS_ENABLE
    fake_gnss_inject(iteration_start_us);
#endif

    (void)kalman_loop();

    // ── FSM tick ────────────────────────────────────────────────────────
    // Runs after kalman_loop so that imu_liftoff_detected is fresh.
    fsm_tick();

    const uint64_t iteration_end_us = app_timebase_now_us();
    const uint64_t elapsed_us = iteration_end_us - iteration_start_us;
    kalman_note_main_loop_iteration_us(static_cast<uint32_t>(elapsed_us));
    g_metrics_tracker.recordLoop(static_cast<uint32_t>(elapsed_us));
}
