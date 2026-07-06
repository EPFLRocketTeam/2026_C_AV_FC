/*
 * sx127x_manual_test.cpp
 *
 *  Created on: Jun 30, 2026
 *      Author: maxime
 */

#include "sx127x_manual_test.h"
#include "../../SX127X.h"
#include "main.h"
#include "stm32h7xx_hal.h"
extern "C" {
#include <cstdio>
}

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi1;

int sx127x_manual_test(void) {
	SX127X_hw_t SX127X_TX_hw;
	SX127X_t SX127XTX;

	SX127X_hw_t SX127X_RX_hw;
	SX127X_t SX127XRX;


	//initialize LoRa module
	SX127X_RX_hw.dio0.port = GPIO_RFM_RX_INT0_GPIO_Port;
	SX127X_RX_hw.dio0.pin = GPIO_RFM_RX_INT0_Pin;
	SX127X_RX_hw.nss.port = SPI_RFM_RX_CS_GPIO_Port;
	SX127X_RX_hw.nss.pin = SPI_RFM_RX_CS_Pin;
	SX127X_RX_hw.reset.port = GPIO_RFM_RX_RST_GPIO_Port;
	SX127X_RX_hw.reset.pin = GPIO_RFM_RX_RST_Pin;
	SX127X_RX_hw.spi = &hspi2;

	//initialize LoRa module
	SX127X_TX_hw.dio0.port = GPIO_RFM_TX_INT0_GPIO_Port;
	SX127X_TX_hw.dio0.pin = GPIO_RFM_TX_INT0_Pin;
	SX127X_TX_hw.nss.port = SPI_RFM_TX_CS_GPIO_Port;
	SX127X_TX_hw.nss.pin = SPI_RFM_TX_CS_Pin;
	SX127X_TX_hw.reset.port = GPIO_RFM_TX_RST_GPIO_Port;
	SX127X_TX_hw.reset.pin = GPIO_RFM_TX_RST_Pin;
	SX127X_TX_hw.spi = &hspi1;

	SX127XTX.hw = &SX127X_TX_hw;
	SX127XRX.hw = &SX127X_RX_hw;

	/* SPI read sanity check: every SX127x returns 0x12 in RegVersion (0x42).
	 * 0x12 => MISO/read path OK. 0x00 or 0xFF => MISO line is dead (check PC2_C
	 * wiring/analog switch) and every status readback (Entry:, IRQ flags) will fail. */
	SX127X_hw_init(&SX127X_TX_hw);
	SX127X_hw_Reset(&SX127X_TX_hw); // active-low reset pulse + settle delay
	uint8_t version = SX127X_SPIRead(&SX127XTX, RegVersion);
	printf("RegVersion = 0x%02X (expect 0x12)\r\n", version);

	SX127X_hw_init(&SX127X_RX_hw);
	SX127X_hw_Reset(&SX127X_RX_hw);

	printf("Configuring LoRa module TX \r\n");
	SX127X_init(&SX127XTX, 868e6, SX127X_POWER_11DBM, SX127X_LORA_SF_7,
	SX127X_LORA_BW_125KHZ, SX127X_LORA_CR_4_5, SX127X_LORA_CRC_EN, 10);
	printf("Done configuring LoRaModule TX\r\n");

	printf("Configuring LoRa module RX\r\n");
	SX127X_init(&SX127XRX, 868e6, SX127X_POWER_11DBM, SX127X_LORA_SF_7,
	SX127X_LORA_BW_125KHZ, SX127X_LORA_CR_4_5, SX127X_LORA_CRC_EN, 10);
	printf("Done configuring LoRaModule RX\r\n");

	int ret = SX127X_LoRaEntryTx(&SX127XTX, 16, 2000);
	if (!ret) {
		printf("Failed to enter in transmission mode \r\n");
		return 1;
	}

	ret = SX127X_LoRaEntryRx(&SX127XRX, 16, 2000);
	if (!ret) {
		printf("Failed to enter in reception mode \r\n");
		return 1;
	}

	char buffer[512];
	char rx_buffer[512];

	int message = 0;
	int message_length = 0;
	ret = SX127X_LoRaEntryRx(&SX127XRX, message_length, 2000);

	while (1) {
		printf("Master ...\r\n");
		HAL_Delay(1000);
		printf("Sending package...\r\n");

//		message_length = sprintf(buffer, "Hello %d", message);
//		ret = SX127X_LoRaEntryTx(&SX127XTX, message_length, 2000);

//		printf("Entry: %d\r\n", ret);
//
//		printf("Sending %s\r\n", buffer);
//		ret = SX127X_LoRaTxPacket(&SX127XTX, (uint8_t*) buffer,
//				message_length, 2000);
		message_length = sprintf(buffer, "Hello %d", message);
		SX127X_transmit(&SX127XTX, (uint8_t*) buffer, message_length, 2000);
		message += 1;
		message_length = sprintf(buffer, "Hello %d", message);
		SX127X_transmit(&SX127XTX, (uint8_t*) buffer, message_length, 2000);
		message += 1;
		message_length = sprintf(buffer, "Hello %d", message);
		SX127X_transmit(&SX127XTX, (uint8_t*) buffer, message_length, 2000);
		message += 1;

		printf("Transmission: %d\r\n", ret);
		printf("Package sent...\r\n");
		HAL_Delay(150);

		int tries = 0;
		int available = 0;
		while (tries <= 4) {
			if (SX127X_available(&SX127XRX)) {
				available = 1;
				break;
			}
			HAL_Delay(100);
			tries++;
		}
//		int received_packet = SX127X_receive(&SX127XRX, message_length, 2000);
		if (!available) {
			printf("Failed to receive a packet\r\n");
			break;
		}
		SX127X_read(&SX127XRX, (uint8_t *)rx_buffer, message_length);
		for (int j = 0; j < message_length; j++) {
			printf("%d ", rx_buffer[j]);
		}
		printf("Received: %s \r\n", rx_buffer);


	}
}

