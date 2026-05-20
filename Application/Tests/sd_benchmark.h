#ifndef APPLICATION_TESTS_SD_BENCHMARK_H
#define APPLICATION_TESTS_SD_BENCHMARK_H

#include "Drivers/STM32HAL/stm32hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void sd_benchmark_run(SD_HandleTypeDef* hsd);

#ifdef __cplusplus
}
#endif

#endif
