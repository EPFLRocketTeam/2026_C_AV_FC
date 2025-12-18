
#include "Drivers/STM32HAL/Simulations/stm32sim_gpio.hpp"

#include <map>
#include <stdexcept>

USING_SIMULATOR_NAMESPACE;

namespace SIMULATOR_NAMESPACE::gpio::extdata {
    std::map<uint32_t, GPIO_PinState> pinStates;
};
using namespace SIMULATOR_NAMESPACE::gpio::extdata;

constexpr int NUMBER_BITS_UINT16 = 8 * sizeof(uint16_t);

uint32_t gpio::SIM_GpioUUID (const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    return ((*GPIOx).port_uuid << NUMBER_BITS_UINT16) | GPIO_Pin;
}

uint32_t gpio::SIM_RegisterGPIO (const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint32_t gpioUUID = SIM_GpioUUID(GPIOx, GPIO_Pin);

    auto it = pinStates.find(gpioUUID);
    if (it == pinStates.end()) {
        pinStates[gpioUUID] = GPIO_PIN_RESET;
    } else {
        throw std::runtime_error(
            std::string("Can't register twice a GPIO (port=") 
          + GPIOx->port_name
          + std::string(", pin=")
          + std::to_string(GPIO_Pin)
          + std::string(")"));
    }

    return gpioUUID;
}
void gpio::SIM_UnregisterGPIO (uint32_t gpio) {
    auto it = pinStates.find(gpio);
    if (it == pinStates.end()) return ;

    pinStates.erase(it);
}

GPIO_PinState HAL_GPIO_ReadPin(const GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint32_t uuid = gpio::SIM_GpioUUID(GPIOx, GPIO_Pin);

    auto it = pinStates.find(uuid);
    if (it == pinStates.end()) {
        throw std::runtime_error(
            std::string("Could not read on Unitialized PIN (port=") 
          + GPIOx->port_name
          + std::string(", pin=")
          + std::to_string(GPIO_Pin)
          + std::string(")"));
    }

    return (*it).second;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
    uint32_t uuid = gpio::SIM_GpioUUID(GPIOx, GPIO_Pin);

    auto it = pinStates.find(uuid);
    if (it == pinStates.end()) {
        throw std::runtime_error(
            std::string("Could not write on Unitialized PIN (port=") 
          + GPIOx->port_name
          + std::string(", pin=")
          + std::to_string(GPIO_Pin)
          + std::string(")"));
    }

    (*it).second = PinState;
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    uint32_t uuid = gpio::SIM_GpioUUID(GPIOx, GPIO_Pin);

    auto it = pinStates.find(uuid);
    if (it == pinStates.end()) {
        throw std::runtime_error(
            std::string("Could not toggle on Unitialized PIN (port=") 
          + GPIOx->port_name
          + std::string(", pin=")
          + std::to_string(GPIO_Pin)
          + std::string(")"));
    }

    GPIO_PinState &state = (*it).second;
    state = (GPIO_PinState) (1 ^ ((int) state));
}

__attribute__((weak)) void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {}

