#ifndef APPLICATION_MAIN_H
#define APPLICATION_MAIN_H

#include <stdint.h>
#include "Drivers/STM32HAL/stm32hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_super_loop_setup(void);
void app_super_loop_iterate(void);

// Bridge functions called from HAL callbacks in Core/Src/main.c.
void app_on_imu_exti(uint16_t gpio_pin);
void app_on_imu_spi_rx_complete(SPI_HandleTypeDef* hspi);

// Runtime IMU source observability for downstream consumers (e.g. Kalman).
uint8_t app_imu_sensor_healthy(uint8_t sensor_index);
uint32_t app_imu_sensor_status_flags(uint8_t sensor_index);
uint8_t app_baro_sensor_healthy(uint8_t sensor_index);
uint32_t app_baro_sensor_status_flags(uint8_t sensor_index);

#ifdef __cplusplus
}
#endif

#endif
