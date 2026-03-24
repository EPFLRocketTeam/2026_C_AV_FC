
#include "Drivers/STM32HAL/Simulations/stm32sim_uart.hpp"

#include <set>
#include <map>
#include <stdexcept>

USING_SIMULATOR_NAMESPACE;

/* Static data for the SPI simulator */
namespace SIMULATOR_NAMESPACE::uart::extdata {
    std::map<UART_HandleUUID, SIM_UARTDevice> devicePerHandle;
    std::set<SIM_UARTDevice> devicesUsed;
    std::map<SIM_UARTDevice, UART_HandleUUID> deviceHandles;
    std::map<SIM_UARTDevice, SIM_UARTDeviceParameters> deviceParameters;
};
using namespace SIMULATOR_NAMESPACE::uart::extdata;

UART_HandleUUID uart::SIM_GetUART_HandleUUID(
    UART_HandleTypeDef *huart) {
    return huart->huart_uuid;
}

static uart::SIM_UARTDevice findDeviceForHandle(
    UART_HandleTypeDef *huart) {
    UART_HandleUUID uuid = uart::SIM_GetUART_HandleUUID(huart);

    auto it = devicePerHandle.find(uuid);
    if (it == devicePerHandle.end()) {
        throw std::runtime_error(
            std::string("UART Handle (") + huart->huart_name +
            std::string(") has no registered device."));
    }

    return it->second;
}

uart::SIM_UARTDevice uart::SIM_RegisterUARTDevice(
    UART_HandleTypeDef *huart, SIM_UARTDeviceParameters params) {
    UART_HandleUUID huart_uuid = SIM_GetUART_HandleUUID(huart);

    /* Check if handle already has a device */
    if (devicePerHandle.find(huart_uuid) != devicePerHandle.end()) {
        throw std::runtime_error(
            std::string("UART device already exists (bus=") +
            huart->huart_name + std::string(")"));
    }

    /* Find a free device id */
    SIM_UARTDevice device = 0;
    for (; device >= 0 && device < 256; device++) {
        if (devicesUsed.find(device) == devicesUsed.end()) {
            break;
        }
    }

    if (device == 256) {
        throw std::runtime_error(
            std::string(
                "Too many UART devices, could not create it (bus=") +
            huart->huart_name + std::string(")"));
    }

    devicePerHandle[huart_uuid] = device;
    devicesUsed.insert(device);
    deviceHandles[device] = huart_uuid;
    deviceParameters[device] = params;

    return device;
}

void uart::SIM_UnregisterUARTDevice(SIM_UARTDevice device) {
    auto it = deviceHandles.find(device);
    if (it == deviceHandles.end()) return;

    UART_HandleUUID huart_uuid = it->second;

    devicePerHandle.erase(huart_uuid);
    devicesUsed.erase(device);
    deviceParameters.erase(device);
    deviceHandles.erase(it);
}

HAL_StatusTypeDef HAL_UART_Transmit(
    UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size,
    uint32_t Timeout) {
    auto device = findDeviceForHandle(huart);
    auto &params = deviceParameters[device];

    params.onTransmit(params.deviceData, pData, Size, Timeout);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(
    UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size,
    uint32_t Timeout) {
    auto device = findDeviceForHandle(huart);
    auto &params = deviceParameters[device];

    return params.onReceive(params.deviceData, pData, Size, Timeout);
}

