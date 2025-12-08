
#ifndef STM32_HAL_MOCK_H
#define STM32_HAL_MOCK_H

#ifndef UNIT_TEST_ENV
#include "stm32h7xx_hal.h"
#else

/* Place your mocked types here */

/*
 * For example, define the UART_HandleTypeDef
 *  to be just an integer so that it is can be
 *  used in the interfaces of the drivers.
 */
// using UART_HandleTypeDef = int;

#endif

#endif
