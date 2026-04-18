// Core/Inc/main.h includes stm32hal.h which pulls in C++ headers —
// include it outside extern "C" (it has its own C++ guards).
#include "Core/Inc/main.h"
#include "cmsis_os.h"
#include "Modules/imu_modlue.hpp"
#include "Modules/gps_module.hpp"

extern "C" {
#include "Application/main.h"
#include "Drivers/InvIMU/InvIMU.h"
}
#include "Drivers/InvIMU/InvIMU.hpp"
#include  "Drivers/UBX_GPS/ubx_gps_interface.h"

extern SPI_HandleTypeDef hspi1;
extern osThreadId_t kalmanTaskHandle;
extern UART_HandleTypeDef huart7;


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
constexpr uint32_t kKalmanThreadWakeFlag = 0x0001U;

constexpr uint16_t kImuIntPins[3] = {
    APP_IMU1_INT_PIN,
    APP_IMU2_INT_PIN,
    APP_IMU3_INT_PIN,
};

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


void mainLoop() {
    Config imu_cfg1{};
    imu_cfg1.hspi = &hspi1;
    imu_cfg1.cs_port = BMI4_NSS_GPIO_Port;
    imu_cfg1.cs_pin = BMI4_NSS_Pin;
    imu_cfg1.use_dwt_timestamps = false;
    imu_cfg1.use_dma = (APP_IMU_USE_DMA != 0u);

    Config imu_cfg2 = imu_cfg1;
    imu_cfg2.cs_port = BMI3_NSS_GPIO_Port;
    imu_cfg2.cs_pin = BMI3_NSS_Pin;

    Config imu_cfg3 = imu_cfg1;
    imu_cfg3.cs_port = BMI2_NSS_GPIO_Port;
    imu_cfg3.cs_pin = BMI2_NSS_Pin;

    InvIMU_STM32 invImu1(imu_cfg1);
    InvIMU_STM32 invImu2(imu_cfg2);
    InvIMU_STM32 invImu3(imu_cfg3);

    InvIMU_Interface* invArr[] = {&invImu1, &invImu2, &invImu3};
    RingBuffer<IMUData, 100>* ringArr[] = {&imuData1, &imuData2, &imuData3};

    ImuModule imuModule(invArr, ringArr);

    if (!imuModule.init()) {
                g_imu_module = nullptr;
                return;
    }
        g_imu_module = &imuModule;

    UbxGpsInterface gps(&huart7, 1000);
    UbxGpsInterface* gpsArr[] = {&gps};
    RingBuffer<GpsBasicFixData, 100>* gpsRing[] = {&gpsData};

    GpsModule gpsModule(gpsArr, gpsRing);
    if (!gpsModule.init()) {
        g_imu_module = nullptr;
        return;
    }

    uint32_t currTick;
    while (1) {
        currTick = HAL_GetTick();
        imuModule.update(currTick);

        for (size_t i = 0; i < 3; ++i) {
            g_imu_healthy[i] = imuModule.sensorHealthy(i) ? 1u : 0u;
            g_imu_status_flags[i] = imuModule.sensorStatusFlags(i);
        }

        gpsModule.update(currTick);
        if (imuModule.takeProducedCount() > 0) {
            osThreadFlagsSet(kalmanTaskHandle, kKalmanThreadWakeFlag);
        }

        // Keep producer/consumer threads cooperative at equal RTOS priority.
        osThreadYield();
    }
}
