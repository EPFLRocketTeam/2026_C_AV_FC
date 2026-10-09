// Host harness: minimal HAL surface used by the Kalman, data stores and drivers' headers.
#pragma once
#include <stdint.h>
#include "Drivers/STM32HAL/Simulations/stm32sim_def.hpp"
#include "Drivers/STM32HAL/Simulations/stm32sim_gpio.hpp"
#include "Drivers/STM32HAL/Simulations/stm32sim_spi.hpp"
#include "Drivers/STM32HAL/Simulations/stm32sim_uart.hpp"
#include "Drivers/STM32HAL/Simulations/stm32sim_ticks.hpp"
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
extern uint32_t SystemCoreClock;
