
#ifndef STM32_SIM_GPIO_H
#define STM32_SIM_GPIO_H

#include <cstddef>
#include <cstdint>
#include <string>

#include "Drivers/STM32HAL/Simulations/stm32sim_def.hpp"

/** HAL Functions that are in the simulation */

typedef enum
{
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
} GPIO_PinState;

/** A single GPIO Port */
typedef struct {
    uint16_t port_uuid;
    std::string port_name;
} GPIO_TypeDef;

GPIO_PinState HAL_GPIO_ReadPin(const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/** Internal Interface to setup the GPIO simulator */
namespace SIMULATOR_NAMESPACE::gpio {
    uint32_t SIM_GpioUUID (const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

    uint32_t SIM_RegisterGPIO (const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    void SIM_UnregisterGPIO (uint32_t gpio);
};

#endif /* STM32_SIM_GPIO_H */
