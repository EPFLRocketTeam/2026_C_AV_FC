#include "InvIMU.hpp"
#include "main.h"
#include "imu_manual_test.h"
#include <cstdio>

using namespace Drivers::InvIMU;

extern SPI_HandleTypeDef hspi3;

static Config imu_cfg;
static InvIMU_STM32* g_imu_instance = nullptr;

void imu_exti_callback() { if (g_imu_instance) g_imu_instance->onInterrupt(); }
void imu_dma_callback()  { if (g_imu_instance) g_imu_instance->onDmaComplete(); }

int manual_test_imu() {
    imu_cfg.hspi               = &hspi3;
    imu_cfg.cs_port            = GPIOE;
    imu_cfg.cs_pin             = GPIO_PIN_4;
    imu_cfg.use_dwt_timestamps = false;

    static InvIMU_STM32 imu_storage(imu_cfg);
    g_imu_instance = &imu_storage;

    printf("\r\n=== ICM-45686 Driver Test ===\r\n\r\n");

    // ----------------------------------------------------------------
    // Phase 2: Raw SPI sanity check
    printf("[Phase 2] Raw SPI WHO_AM_I ........ ");
    uint8_t tx_buf[2] = { 0x72u | 0x80u, 0x00u };
    uint8_t rx_buf[2] = {};
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, 2, 100);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
    if (spi_status != HAL_OK) {
        printf("FAIL (HAL err %d)\r\n", (int)spi_status);
        return -1;
    }
    printf("0x%02X  %s\r\n", rx_buf[1], rx_buf[1] == INV_IMU_WHO_AM_I_VAL ? "OK" : "UNEXPECTED");

    // ----------------------------------------------------------------
    // Phase 3: Driver ping
    printf("[Phase 3] Driver ping ............. ");
    if (!g_imu_instance->ping()) {
        printf("FAIL (0x%08lX)\r\n", g_imu_instance->getStatusFlags());
        return -1;
    }
    printf("OK\r\n");

    // ----------------------------------------------------------------
    // Phase 4: Driver init
    printf("[Phase 4] Driver init ............. ");
    if (!g_imu_instance->init()) {
        printf("FAIL (0x%08lX)\r\n", g_imu_instance->getStatusFlags());
        return -1;
    }
    printf("OK\r\n");

    // ----------------------------------------------------------------
    // Phase 5: Configure
    printf("[Phase 5] Configure + FIFO ........ ");
    g_imu_instance->configure(AccelRange::_16G, GyroRange::_2000DPS, ODR::_800Hz);
    g_imu_instance->configureFifo();
    g_imu_instance->clearStatusFlags();
    printf("16G | 2000 DPS | 800 Hz | hires 20B\r\n");

    printf("          Warmup drain (200ms) .... ");
    uint32_t drain_start = HAL_GetTick();
    while ((HAL_GetTick() - drain_start) < 200u) {
        g_imu_instance->tick();
        IMUData tmp;
        while (g_imu_instance->getFrame(tmp)) {}
    }
    g_imu_instance->clearStatusFlags();
    printf("OK\r\n");

    // ----------------------------------------------------------------
    // Phase 6: Streaming
    printf("\r\n[Phase 6] Streaming at 800 Hz\r\n");
    printf("          %8s  %8s  %8s    %8s  %8s  %8s    %s\r\n",
           "Ax(m/s²)", "Ay(m/s²)", "Az(m/s²)",
           "Gx(r/s)", "Gy(r/s)", "Gz(r/s)", "T(°C)");

    IMUData frame;
    uint32_t frame_count = 0;

    while (1) {
        g_imu_instance->tick();

        while (g_imu_instance->getFrame(frame)) {
            frame_count++;
            if (frame_count % 800 == 0) {
                printf("[%5lu]   %+7.3f   %+7.3f   %+7.3f     %+7.4f   %+7.4f   %+7.4f    %5.1f\r\n",
                    frame_count,
                    frame.accel_x, frame.accel_y, frame.accel_z,
                    frame.gyro_x,  frame.gyro_y,  frame.gyro_z,
                    frame.temperature - 273.15f);
            }
        }
    }
}
