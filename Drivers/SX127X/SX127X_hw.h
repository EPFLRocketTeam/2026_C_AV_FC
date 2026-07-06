/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * Hardware layer for SX127X LoRa module
 */

#ifndef __SX127X_HW_HEADER
#define __SX127X_HW_HEADER

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
	int pin;
	void *port;
} SX127X_hw_dio_t;

typedef struct {
	SX127X_hw_dio_t reset;
	SX127X_hw_dio_t dio0;
	SX127X_hw_dio_t nss;
	void *spi;
} SX127X_hw_t;

/**
 * \brief Initialize hardware layer
 *
 * Clears NSS and resets LoRa module.
 *
 * \param[in]   hw 		Pointer to hardware structure
 */
void SX127X_hw_init(SX127X_hw_t *hw);

/**
 * \brief Control NSS
 *
 * Clears and sets NSS according to passed value.
 *
 * \param[in]   hw 		Pointer to hardware structure.
 * \param[in]   value   1 sets NSS high, other value sets NSS low.
 */
void SX127X_hw_SetNSS(SX127X_hw_t *hw, int value);

/**
 * \brief Resets LoRa module
 *
 * Resets LoRa module.
 *
 * \param[in]   hw 		Pointer to hardware structure
 */
void SX127X_hw_Reset(SX127X_hw_t *hw);

/**
 * \brief Send command via SPI.
 *
 * Send single byte via SPI interface.
 *
 * \param[in]   hw 		Pointer to hardware structure
 * \param[in]   cmd		Command
 */
void SX127X_hw_SPICommand(SX127X_hw_t *hw, uint8_t cmd);

/**
 * \brief Reads data via SPI
 *
 * Reads data via SPI interface.
 *
 * \param[in]   hw 		Pointer to hardware structure
 *
 * \return				Read value
 */
uint8_t SX127X_hw_SPIReadByte(SX127X_hw_t *hw);

/**
 * \brief Full-duplex SPI transfer in a single NSS-low frame.
 *
 * Sends \p length bytes from \p txBuf while capturing \p length bytes into
 * \p rxBuf, all within one continuous transfer (NSS asserted low for the whole
 * frame). This avoids toggling the SPI enable bit between bytes, which on the
 * STM32H7 injects a spurious clock and shifts the data by one bit.
 *
 * \param[in]   hw 		Pointer to hardware structure
 * \param[in]   txBuf	Bytes to transmit (length bytes)
 * \param[out]  rxBuf	Buffer receiving length bytes (must be non-NULL)
 * \param[in]   length	Number of bytes to transfer
 */
void SX127X_hw_SPITransfer(SX127X_hw_t *hw, const uint8_t *txBuf,
		uint8_t *rxBuf, uint16_t length);

/**
 * \brief ms delay
 *
 * Milisecond delay.
 *
 * \param[in]   msec 		Number of milliseconds to wait
 */
void SX127X_hw_DelayMs(uint32_t msec);

/**
 * \brief Reads DIO0 state
 *
 * Reads LoRa DIO0 state using GPIO.
 *
 * \param[in]   hw 		Pointer to hardware structure
 *
 * \return				0 if DIO0 low, 1 if DIO high
 */
int SX127X_hw_GetDIO0(SX127X_hw_t *hw);

#ifdef __cplusplus
}
#endif

#endif
