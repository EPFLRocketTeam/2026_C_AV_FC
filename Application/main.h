#ifndef APPLICATION_MAIN_H
#define APPLICATION_MAIN_H

#include <stdint.h>
#include "Drivers/STM32HAL/stm32hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void simple_radio_init(void);
void simple_radio_tick(void);
void simple_radio_print_stats(void);


void app_super_loop_setup(void);
void app_super_loop_iterate(void);

uint8_t app_get_current_baro_count (void);
void app_open_parachute ();
void app_set_pyro_status (int pyro_id, bool enabled);
uint64_t app_get_remaining_disk_size (void);
uint64_t app_get_sd_fail_count (void);
float app_get_current_imu_rate (void);
void app_on_state_becomes_init (void);
void app_on_state_becomes_armed (void);

// Bridge functions called from HAL callbacks in Core/Src/main.c.
void app_on_imu_exti(uint16_t gpio_pin);
void app_on_imu_spi_rx_complete(SPI_HandleTypeDef* hspi);

// Runtime IMU source observability for downstream consumers (e.g. Kalman).
uint8_t app_imu_sensor_healthy(uint8_t sensor_index);
uint32_t app_imu_sensor_status_flags(uint8_t sensor_index);
void app_imu_frame_counts(uint32_t* h78, uint32_t* hF0, uint32_t* other);
void app_imu_ts_diagnostics(uint32_t* h7C, uint32_t* mono_repairs, int32_t* last_err,
                            uint32_t* reject_count, int32_t* max_rejected_err,
                            uint32_t* spi_fifo_fail, uint32_t* spi_not_ready,
                            uint32_t* burst_count, uint8_t* gate_armed);
uint8_t app_baro_sensor_healthy(uint8_t sensor_index);
uint32_t app_baro_sensor_status_flags(uint8_t sensor_index);

#ifdef __cplusplus
}
#endif

#endif
