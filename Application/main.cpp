// Core/Inc/main.h includes stm32hal.h which pulls in C++ headers —
// include it outside extern "C" (it has its own C++ guards).
#include "Core/Inc/main.h"
#include "Application/app_timebase.h"
#include "Modules/imu_modlue.hpp"
#include "Modules/gps_module.hpp"

extern "C" {
#include "Application/main.h"
#include "Application/Kalman/kalman_process.h"
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"
#include  "Drivers/UBX_GPS/ubx_gps_interface.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart7;

using Drivers::InvIMU::Config;
using Drivers::InvIMU::IMUData;
using Drivers::InvIMU::IMU_STATUS_OK;
using Drivers::InvIMU::InvIMU_Interface;
using Drivers::InvIMU::InvIMU_STM32;


RingBuffer<IMUData, 100> imuData1;
RingBuffer<IMUData, 100> imuData2;
RingBuffer<IMUData, 100> imuData3;

RingBuffer<GpsBasicFixData, 100> gpsData;

#ifndef APP_IMU_USE_DMA
#define APP_IMU_USE_DMA 0u
#endif

// Set to the matching GPIO pin number (e.g. GPIO_PIN_13) when EXTI is wired.
// Keep at 0 when no hardware interrupt line is available.
#ifndef APP_IMU1_INT_PIN
#define APP_IMU1_INT_PIN 0u
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

Config makeImuConfig(GPIO_TypeDef* cs_port, uint16_t cs_pin) {
    Config cfg{};
    cfg.hspi = &hspi1;
    cfg.cs_port = cs_port;
    cfg.cs_pin = cs_pin;
    cfg.use_dwt_timestamps = true;
    cfg.use_dma = (APP_IMU_USE_DMA != 0u);
    return cfg;
}

struct SuperLoopContext {
    Config imu_cfg1 = makeImuConfig(BMI4_NSS_GPIO_Port, BMI4_NSS_Pin);
    Config imu_cfg2 = makeImuConfig(BMI3_NSS_GPIO_Port, BMI3_NSS_Pin);
    Config imu_cfg3 = makeImuConfig(BMI2_NSS_GPIO_Port, BMI2_NSS_Pin);

    InvIMU_STM32 invImu1{imu_cfg1};
    InvIMU_STM32 invImu2{imu_cfg2};
    InvIMU_STM32 invImu3{imu_cfg3};

    InvIMU_Interface* invArr[3] = {&invImu1, &invImu2, &invImu3};
    RingBuffer<IMUData, 100>* ringArr[3] = {&imuData1, &imuData2, &imuData3};
    ImuModule imuModule{invArr, ringArr};

    UbxGpsInterface gps{&huart7, kGpsRateMs};
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

    // Current ERT wiring uses SPI1 for all three IMUs.
    if (hspi != &hspi1) {
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
    g_superloop.gpsModule.update(now_ms);

    (void)kalman_loop();

    const uint64_t iteration_end_us = app_timebase_now_us();
    const uint64_t elapsed_us = iteration_end_us - iteration_start_us;
    kalman_note_main_loop_iteration_us(static_cast<uint32_t>(elapsed_us));
}
