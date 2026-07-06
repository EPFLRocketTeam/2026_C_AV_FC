/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * Hardware layer for SX127X LoRa module
 */

#include "../SX127X_hw.h"
#include <string.h>
#include "Drivers/STM32HAL/stm32hal.h"

__weak void SX127X_hw_init(SX127X_hw_t *hw) {
	SX127X_hw_SetNSS(hw, 1);
	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);
}

__weak void SX127X_hw_SetNSS(SX127X_hw_t *hw, int value) {
	HAL_GPIO_WritePin(hw->nss.port, hw->nss.pin,
			(value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

__weak void SX127X_hw_Reset(SX127X_hw_t *hw) {
	SX127X_hw_SetNSS(hw, 1);
	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_RESET);

	SX127X_hw_DelayMs(1);

	HAL_GPIO_WritePin(hw->reset.port, hw->reset.pin, GPIO_PIN_SET);

	SX127X_hw_DelayMs(100);
}

__weak void SX127X_hw_SPICommand(SX127X_hw_t *hw, uint8_t cmd) {
	SX127X_hw_SetNSS(hw, 0);
	HAL_SPI_Transmit(hw->spi, &cmd, 1, 1000);
	while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
		;
}

__weak uint8_t SX127X_hw_SPIReadByte(SX127X_hw_t *hw) {
	uint8_t txByte = 0x00;
	uint8_t rxByte = 0x00;

	SX127X_hw_SetNSS(hw, 0);
	HAL_SPI_TransmitReceive(hw->spi, &txByte, &rxByte, 1, 1000);
	while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
		;
	return rxByte;
}

__weak void SX127X_hw_SPITransfer(SX127X_hw_t *hw, const uint8_t *txBuf,
		uint8_t *rxBuf, uint16_t length) {
	SX127X_hw_SetNSS(hw, 0);
	if (HAL_SPI_TransmitReceive(hw->spi, (uint8_t *) txBuf, rxBuf, length,
			1000) != HAL_OK) {
		/* A timed-out/failed transfer can leave unread bytes in the RX FIFO,
		 * which desynchronizes every following transfer (and can underflow the
		 * HAL's FIFO bookkeeping into an unbounded spin). Abort flushes the
		 * FIFOs and returns the peripheral to a clean state. */
		HAL_SPI_Abort(hw->spi);
	}
	while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
		;
	SX127X_hw_SetNSS(hw, 1);
}

__weak void SX127X_hw_DelayMs(uint32_t msec) {
	HAL_Delay(msec);
}

__weak int SX127X_hw_GetDIO0(SX127X_hw_t *hw) {
	return (HAL_GPIO_ReadPin(hw->dio0.port, hw->dio0.pin) == GPIO_PIN_SET);
}
