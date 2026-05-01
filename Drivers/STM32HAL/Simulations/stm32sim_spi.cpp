
#include "Drivers/STM32HAL/Simulations/stm32sim_spi.hpp"
#include "Drivers/STM32HAL/Simulations/stm32sim_gpio.hpp"

#include <set>
#include <map>
#include <stdexcept>

USING_SIMULATOR_NAMESPACE;

/* Static data for the SPI simulator */
namespace SIMULATOR_NAMESPACE::spi::extdata {
    std::map<SPI_HandleUUID, std::set<uint32_t>> ssListPerHandle;
    std::map<std::pair<SPI_HandleUUID, uint32_t>, SIM_SPIDevice> deviceList;
    std::map<uint32_t, std::tuple<GPIO_TypeDef*, uint16_t, GPIO_PinState>> ssGPIOInformation;
    std::set<SIM_SPIDevice> devicesUsed;
    std::map<SIM_SPIDevice, std::pair<SPI_HandleUUID, uint32_t>> deviceHandles;
    std::map<SIM_SPIDevice, SIM_SPIDeviceParameters> deviceParameters;
};
using namespace SIMULATOR_NAMESPACE::spi::extdata;

spi::SIM_SPIDevice spi::SIM_GetDeviceEnabled (SPI_HandleTypeDef* hspi) {
    SPI_HandleUUID hpsi_uuid = spi::SIM_GetSPI_HandleUUID(hspi);

    uint16_t nbGpios = 0;
    uint32_t gpioId  = 0;
    for (uint32_t GPIO_UUID : ssListPerHandle[hpsi_uuid]) {
        auto [GPIOx, GPIO_Pin, GPIO_ActiveState] = ssGPIOInformation[GPIO_UUID];

        GPIO_PinState state;
        
        try
        {
            state = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
        }
        catch(const std::runtime_error &error)
        {
            throw std::runtime_error(
                std::string("SPI Handle (")
                + hspi->hspi_name
                + std::string(") has an unregistered GPIO: ")
                + std::string(error.what())
            );
        }

        if (state == GPIO_ActiveState) {
            if (nbGpios == 1) {
                throw std::runtime_error(
                    std::string("SPI Handle (")
                  + hspi->hspi_name
                  + std::string(") has too many Slave Selects enabled (>= 2).")
                );
            }

            gpioId = GPIO_UUID;
            nbGpios ++;
        }
    }

    if (nbGpios == 0) {
        throw std::runtime_error(
            std::string("SPI Handle (")
            + hspi->hspi_name
            + std::string(") has no Slave Selects enabled.")
        );
    }

    return deviceList[{ hpsi_uuid, gpioId }];
}

HAL_StatusTypeDef HAL_SPI_Transmit(
    SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    auto  device = spi::SIM_GetDeviceEnabled(hspi);
    auto &params = deviceParameters[device];

    params.onTransmit(params.deviceData, pData, Size);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Receive(
    SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    auto  device = spi::SIM_GetDeviceEnabled(hspi);
    auto &params = deviceParameters[device];

    params.onReceive(params.deviceData, pData, Size);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    auto  device = spi::SIM_GetDeviceEnabled(hspi);
    auto &params = deviceParameters[device];

    params.onRxTx(params.deviceData, pTxData, pRxData, Size);
    return HAL_OK;
}

SPI_HandleUUID spi::SIM_GetSPI_HandleUUID (SPI_HandleTypeDef* hspi) {
    return hspi->hspi_uuid;
}
spi::SIM_SPIDevice spi::SIM_RegisterSPIDevice (
        SPI_HandleTypeDef* hspi,
        GPIO_TypeDef* GPIOx,
        uint16_t GPIO_Pin,
        SIM_SPIDeviceParameters params
    ) {
    SPI_HandleUUID hspi_uuid = spi::SIM_GetSPI_HandleUUID(hspi);
    
    SIM_SPIDevice device = 0;
    for (; device >= 0 && device < 256; device ++) {
        if (devicesUsed.find(device) == devicesUsed.end()) {
            break ;
        }
    }

    if (device == 256) {
        throw std::runtime_error(
            std::string("Too many SPI devices, could not create it (bus=")
          + std::string( hspi->hspi_name )
          + std::string(", ss=(port=")
          + std::string( GPIOx->port_name )
          + std::string(", pin=")
          + std::to_string( GPIO_Pin )
          + std::string("))")
        );
    }

    uint32_t GPIO_UUID = gpio::SIM_GpioUUID(GPIOx, GPIO_Pin);

    auto it = deviceList.find({ hspi_uuid, GPIO_UUID });
    if (it != deviceList.end()) {
        throw std::runtime_error(
            std::string("SPI device already exists (bus=")
          + std::string( hspi->hspi_name )
          + std::string(", ss=(port=")
          + std::string( GPIOx->port_name )
          + std::string(", pin=")
          + std::to_string( GPIO_Pin )
          + std::string("))")
        );
    }

    deviceList[{ hspi_uuid, GPIO_UUID }] = device;
    devicesUsed.insert(device);
    ssListPerHandle[hspi_uuid].insert(GPIO_UUID);
    deviceParameters[device] = params;
    deviceHandles[device] = { hspi_uuid, GPIO_UUID };
    ssGPIOInformation[GPIO_UUID] = { GPIOx, GPIO_Pin, params.activeState };

    return device;
}
void spi::SIM_UnregisterSPIDevice (SPI_HandleUUID uuid) {
    auto it = deviceHandles.find(uuid);
    if (it == deviceHandles.end()) return ;

    auto [_, hspi_gpio] = (*it);
    auto [hspi, gpio] = hspi_gpio;
    deviceList.erase({ hspi, gpio });
    ssListPerHandle[hspi].erase(gpio);
    devicesUsed.erase(uuid);
    deviceParameters.erase(uuid);
    deviceHandles.erase(it);
    ssGPIOInformation.erase(gpio);
}
