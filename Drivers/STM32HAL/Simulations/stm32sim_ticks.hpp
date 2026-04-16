#ifndef STM32SIM_TICKS_H
#define STM32SIM_TICKS_H

#include <stdint.h>

void stm32sim_ticks_init(void);
void stm32sim_ticks_deinit(void);

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

#endif
