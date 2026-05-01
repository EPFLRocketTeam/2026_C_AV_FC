
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

/** GPIO Pin bitmask constants (matches STM32 HAL values) */
#define GPIO_PIN_0   ((uint16_t)0x0001U)
#define GPIO_PIN_1   ((uint16_t)0x0002U)
#define GPIO_PIN_2   ((uint16_t)0x0004U)
#define GPIO_PIN_3   ((uint16_t)0x0008U)
#define GPIO_PIN_4   ((uint16_t)0x0010U)
#define GPIO_PIN_5   ((uint16_t)0x0020U)
#define GPIO_PIN_6   ((uint16_t)0x0040U)
#define GPIO_PIN_7   ((uint16_t)0x0080U)
#define GPIO_PIN_8   ((uint16_t)0x0100U)
#define GPIO_PIN_9   ((uint16_t)0x0200U)
#define GPIO_PIN_10  ((uint16_t)0x0400U)
#define GPIO_PIN_11  ((uint16_t)0x0800U)
#define GPIO_PIN_12  ((uint16_t)0x1000U)
#define GPIO_PIN_13  ((uint16_t)0x2000U)
#define GPIO_PIN_14  ((uint16_t)0x4000U)
#define GPIO_PIN_15  ((uint16_t)0x8000U)

/** Simulated GPIO port instances (extern declarations) */
extern GPIO_TypeDef SIM_GPIOA;
extern GPIO_TypeDef SIM_GPIOB;
extern GPIO_TypeDef SIM_GPIOC;
extern GPIO_TypeDef SIM_GPIOD;
extern GPIO_TypeDef SIM_GPIOE;
extern GPIO_TypeDef SIM_GPIOF;
extern GPIO_TypeDef SIM_GPIOG;
extern GPIO_TypeDef SIM_GPIOH;

#define GPIOA (&SIM_GPIOA)
#define GPIOB (&SIM_GPIOB)
#define GPIOC (&SIM_GPIOC)
#define GPIOD (&SIM_GPIOD)
#define GPIOE (&SIM_GPIOE)
#define GPIOF (&SIM_GPIOF)
#define GPIOG (&SIM_GPIOG)
#define GPIOH (&SIM_GPIOH)

/** Internal Interface to setup the GPIO simulator */
namespace SIMULATOR_NAMESPACE::gpio {
    uint32_t SIM_GpioUUID (const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

    uint32_t SIM_RegisterGPIO (const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
    void SIM_UnregisterGPIO (uint32_t gpio);
};

#endif /* STM32_SIM_GPIO_H */
