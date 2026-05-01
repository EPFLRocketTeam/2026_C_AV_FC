
#ifndef STM32_SIM_SPI_H
#define STM32_SIM_SPI_H

#include <cstddef>
#include <cstdint>
#include <string>

#include "Drivers/STM32HAL/Simulations/stm32sim_def.hpp"
#include "Drivers/STM32HAL/Simulations/stm32sim_gpio.hpp"

/** HAL Functions that are in the simulation */

/** Internal UUID for SPI bus */
typedef uint8_t SPI_HandleUUID;

/** Handle for SPI bus */
typedef struct {
    SPI_HandleUUID hspi_uuid;

    std::string hspi_name;
} SPI_HandleTypeDef;

HAL_StatusTypeDef HAL_SPI_Transmit(
    SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(
    SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(
    SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

/** Internal Interface to setup the SPI simulator */
namespace SIMULATOR_NAMESPACE::spi {
    typedef void (*SIM_SPIRxHandler)   (void* deviceData, uint8_t *pData, uint16_t Size);
    typedef void (*SIM_SPITxHandler)   (void* deviceData, const uint8_t *pData, uint16_t Size);
    typedef void (*SIM_SPIRxTxHandler) (void* deviceData, const uint8_t* pTxData, uint8_t* pRxData, uint16_t Size);

    /** Internal UUID for a SPI Device (Bus + Slave Select) */
    typedef int16_t SIM_SPIDevice;
    typedef enum {
        SPI_Handler
    } SIM_SPIDeviceMode;
    typedef struct {
        /* Device Data */
        void* deviceData;

        /* Device Mode */
        SIM_SPIDeviceMode mode;

        GPIO_PinState activeState;

        /* SPI_Handler mode handlers */
        SIM_SPIRxHandler   onReceive;
        SIM_SPITxHandler   onTransmit;
        SIM_SPIRxTxHandler onRxTx;
    } SIM_SPIDeviceParameters;

    /**
     * Return the handle uuid of a given SPI handle
     */
    SPI_HandleUUID SIM_GetSPI_HandleUUID (SPI_HandleTypeDef* hspi);
    /**
     * Find the SPI Device that is currently enabled.
     * 
     * Searchs through all Slave Select Pins to find one
     * that is enabled. It may throw an exception if no
     * such GPIO is disabled or if more than one GPIO
     * is enabled.
     */
    SIM_SPIDevice SIM_GetDeviceEnabled (SPI_HandleTypeDef* hspi);
    /**
     * Create a SPI device for the given handle and GPIO
     * 
     * @return either the spi device, or an integer strictly less than zero representing an error
     */
    SIM_SPIDevice SIM_RegisterSPIDevice (
        SPI_HandleTypeDef* hspi,
        GPIO_TypeDef* GPIOx,
        uint16_t GPIO_Pin,
        SIM_SPIDeviceParameters params
    );
    /**
     * Delete the SPI device
     */
    void SIM_UnregisterSPIDevice (SPI_HandleUUID uuid);
};

#endif /* STM32_SIM_SPI_H */