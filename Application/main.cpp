// Core/Inc/main.h includes stm32hal.h which pulls in C++ headers —
// include it outside extern "C" (it has its own C++ guards).
#include "Core/Inc/main.h"
#include "Application/app_timebase.h"
#include "Modules/baro_module.hpp"
#include "Modules/imu_modlue.hpp"
#include "Modules/gps_module.hpp"

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

using Drivers::InvIMU::Config;
using Drivers::InvIMU::IMUData;
using Drivers::InvIMU::IMU_STATUS_OK;
using Drivers::InvIMU::InvIMU_Interface;
using Drivers::InvIMU::InvIMU_STM32;
using Drivers::BMP390::BaroData;


RingBuffer<IMUData, 100> imuData1;
RingBuffer<IMUData, 100> imuData2;
RingBuffer<IMUData, 100> imuData3;

RingBuffer<GpsBasicFixData, 100> gpsData;
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
#define APP_IMU2_INT_PIN 0u
#endif

#ifndef APP_IMU3_INT_PIN
#define APP_IMU3_INT_PIN 0u
#endif

namespace {

ImuModule* g_imu_module = nullptr;
uint8_t g_imu_healthy[3] = {0u, 0u, 0u};
uint32_t g_imu_status_flags[3] = {IMU_STATUS_OK, IMU_STATUS_OK, IMU_STATUS_OK};
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

constexpr uint16_t kImuIntPins[3] = {
    APP_IMU1_INT_PIN,
    APP_IMU2_INT_PIN,
    APP_IMU3_INT_PIN,
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
//    Config imu_cfg2 = makeImuConfig(&hspi4, BMI3_NSS_GPIO_Port, BMI3_NSS_Pin);
//    Config imu_cfg3 = makeImuConfig(&hspi5, BMI2_NSS_GPIO_Port, BMI2_NSS_Pin);

    InvIMU_STM32 invImu1{imu_cfg1};
//    InvIMU_STM32 invImu2{imu_cfg2};
//    InvIMU_STM32 invImu3{imu_cfg3};

    //InvIMU_Interface* invArr[3] = {&invImu1, &invImu2, &invImu3};
    InvIMU_Interface* invArr[1] = {&invImu1};
    // RingBuffer<IMUData, 100>* ringArr[3] = {&imuData1, &imuData2, &imuData3};
    RingBuffer<IMUData, 100>* ringArr[1] = {&imuData1};
    ImuModule imuModule{invArr, ringArr};

#ifdef UNIT_TEST_ENV
    Drivers::BMP390::BMP390_Mock baro1{};
    Drivers::BMP390::BMP390_Mock baro2{};
    Drivers::BMP390::BMP390_Mock baro3{};
    Drivers::BMP390::BMP390_Mock baro4{};
#else
    Drivers::BMP390::BMP390_SDK::Config baro_cfg1 =
        makeBaroConfig(&hspi5, BMP3_CS1_GPIO_Port, BMP3_CS1_Pin);
//    Drivers::BMP390::BMP390_SDK::Config baro_cfg2 =
//        makeBaroConfig(&hspi5, BMP2_NSS_GPIO_Port, BMP2_NSS_Pin);
//    Drivers::BMP390::BMP390_SDK::Config baro_cfg3 =
//        makeBaroConfig(&hspi4, BMP3_NSS_GPIO_Port, BMP3_NSS_Pin);
//    Drivers::BMP390::BMP390_SDK::Config baro_cfg4 =
//        makeBaroConfig(&hspi4, BMP4_NSS_GPIO_Port, BMP4_NSS_Pin);

    Drivers::BMP390::BMP390_SDK baro1{baro_cfg1};
    // Drivers::BMP390::BMP390_SDK baro2{baro_cfg2};
    // Drivers::BMP390::BMP390_SDK baro3{baro_cfg3};
    // Drivers::BMP390::BMP390_SDK baro4{baro_cfg4};
#endif
//    Drivers::BMP390::BMP390_Interface* baroArr[4] = {
//        &baro1, &baro2, &baro3, &baro4};
//    RingBuffer<BaroData, 100>* baroRing[4] = {
//        &baroData1, &baroData2, &baroData3, &baroData4};
    Drivers::BMP390::BMP390_Interface* baroArr[1] = {
            &baro1 };
        RingBuffer<BaroData, 100>* baroRing[1] = {
            &baroData1 };
    BaroModule baroModule{baroArr, baroRing};

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

    const uint32_t now_ms = HAL_GetTick();
    for (size_t i = 0; i < 3; ++i) {
        if (kImuIntPins[i] == 0u) {
            continue;
        }
        if (kImuIntPins[i] == gpio_pin) {
            g_imu_module->onImuInterrupt(i, now_ms);
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
    if (sensor_index >= 3u) {
        return 0u;
    }
    return g_imu_healthy[sensor_index];
}

extern "C" uint32_t app_imu_sensor_status_flags(uint8_t sensor_index) {
    if (sensor_index >= 3u) {
        return IMU_STATUS_OK;
    }
    return g_imu_status_flags[sensor_index];
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

    g_superloop.baroModule.setTriggerCallback(kalman_note_baro_trigger);
    if (!g_superloop.baroModule.init()) {
        // Non-fatal: system can operate with degraded baro (voting handles it)
        printf("WARNING: No barometers initialized\n");
    }

    if (!g_superloop.gpsModule.init()) {
        g_superloop.ready = false;
        return;
    }
    g_superloop.ready = true;
}

extern "C" void app_super_loop_iterate(void) {
    if (!g_superloop.ready) {
        return;
    }

    const uint64_t iteration_start_us = app_timebase_now_us();
    const uint32_t now_ms = HAL_GetTick();
    g_superloop.imuModule.update(now_ms);

    for (size_t i = 0; i < 3; ++i) {
        g_imu_healthy[i] = g_superloop.imuModule.sensorHealthy(i) ? 1u : 0u;
        g_imu_status_flags[i] = g_superloop.imuModule.sensorStatusFlags(i);
    }

    (void)g_superloop.imuModule.takeProducedCount();
    g_superloop.baroModule.update(now_ms);
    for (size_t i = 0; i < 4; ++i) {
        g_baro_healthy[i] = g_superloop.baroModule.sensorHealthy(i) ? 1u : 0u;
        g_baro_status_flags[i] = g_superloop.baroModule.sensorStatusFlags(i);
    }
    (void)g_superloop.baroModule.takeProducedCount();
    g_superloop.gpsModule.update(now_ms);

    (void)kalman_loop();

    const uint64_t iteration_end_us = app_timebase_now_us();
    const uint64_t elapsed_us = iteration_end_us - iteration_start_us;
    kalman_note_main_loop_iteration_us(static_cast<uint32_t>(elapsed_us));
}
