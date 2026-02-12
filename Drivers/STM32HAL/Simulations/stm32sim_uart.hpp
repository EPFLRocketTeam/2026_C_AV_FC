
#ifndef STM32_SIM_UART_H
#define STM32_SIM_UART_H

#include <cstddef>
#include <cstdint>
#include <string>

#include "Drivers/STM32HAL/Simulations/stm32sim_def.hpp"


/** HAL Functions that are in the simulation */

/** Internal UUID for UART bus */
typedef uint8_t UART_HandleUUID;

/** Handle for UART bus */
typedef struct {
    UART_HandleUUID huart_uuid;

    std::string huart_name;
} UART_HandleTypeDef;

// Mock HAL functions
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

/** Internal Interface to setup the SPI simulator */
namespace SIMULATOR_NAMESPACE::uart {

	typedef void (*SIM_UARTTxHandler)(
	        void *deviceData, const uint8_t *pData, uint16_t Size,
	        uint32_t Timeout);
	    typedef HAL_StatusTypeDef (*SIM_UARTRxHandler)(
	        void *deviceData, uint8_t *pData, uint16_t Size,
	        uint32_t Timeout);

	    /** Internal UUID for a UART Device */
	    typedef int16_t SIM_UARTDevice;

	    typedef struct {
	        /* Opaque device data passed to handlers */
	        void *deviceData;

	        /* Handlers */
	        SIM_UARTTxHandler onTransmit;
	        SIM_UARTRxHandler onReceive;
	    } SIM_UARTDeviceParameters;

	    /**
	     * Return the handle uuid of a given UART handle
	     */
	    UART_HandleUUID SIM_GetUART_HandleUUID(UART_HandleTypeDef *huart);

	    /**
	     * Register a UART device on the given handle.
	     * Each handle supports exactly one device (point-to-point).
	     *
	     * @return the device id, or a negative value on error
	     */
	    SIM_UARTDevice SIM_RegisterUARTDevice(
	        UART_HandleTypeDef *huart, SIM_UARTDeviceParameters params);

	    /**
	     * Unregister a UART device
	     */
	    void SIM_UnregisterUARTDevice(SIM_UARTDevice device);
};

#endif /* STM32_SIM_SPI_H */
