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
	SX1278_hw_t SX1278_TX_hw;
	SX1278_t SX1278TX;

	SX1278_hw_t SX1278_RX_hw;
	SX1278_t SX1278RX;


	//initialize LoRa module
	SX1278_RX_hw.dio0.port = GPIO_RFM_RX_INT0_GPIO_Port;
	SX1278_RX_hw.dio0.pin = GPIO_RFM_RX_INT0_Pin;
	SX1278_RX_hw.nss.port = SPI_RFM_RX_CS_GPIO_Port;
	SX1278_RX_hw.nss.pin = SPI_RFM_RX_CS_Pin;
	SX1278_RX_hw.reset.port = GPIO_RFM_RX_RST_GPIO_Port;
	SX1278_RX_hw.reset.pin = GPIO_RFM_RX_RST_Pin;
	SX1278_RX_hw.spi = &hspi2;

	//initialize LoRa module
	SX1278_TX_hw.dio0.port = GPIO_RFM_TX_INT0_GPIO_Port;
	SX1278_TX_hw.dio0.pin = GPIO_RFM_TX_INT0_Pin;
	SX1278_TX_hw.nss.port = SPI_RFM_TX_CS_GPIO_Port;
	SX1278_TX_hw.nss.pin = SPI_RFM_TX_CS_Pin;
	SX1278_TX_hw.reset.port = GPIO_RFM_TX_RST_GPIO_Port;
	SX1278_TX_hw.reset.pin = GPIO_RFM_TX_RST_Pin;
	SX1278_TX_hw.spi = &hspi1;

	SX1278TX.hw = &SX1278_TX_hw;
	SX1278RX.hw = &SX1278_RX_hw;

	/* SPI read sanity check: every SX127x returns 0x12 in RegVersion (0x42).
	 * 0x12 => MISO/read path OK. 0x00 or 0xFF => MISO line is dead (check PC2_C
	 * wiring/analog switch) and every status readback (Entry:, IRQ flags) will fail. */
	SX1278_hw_init(&SX1278_TX_hw);
	SX1278_hw_Reset(&SX1278_TX_hw); // active-low reset pulse + settle delay
	uint8_t version = SX1278_SPIRead(&SX1278TX, RegVersion);
	printf("RegVersion = 0x%02X (expect 0x12)\r\n", version);

	SX1278_hw_init(&SX1278_RX_hw);
	SX1278_hw_Reset(&SX1278_RX_hw);

	printf("Configuring LoRa module TX \r\n");
	SX1278_init(&SX1278TX, 868e6, SX1278_POWER_11DBM, SX1278_LORA_SF_7,
	SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);
	printf("Done configuring LoRaModule TX\r\n");

	printf("Configuring LoRa module RX\r\n");
	SX1278_init(&SX1278RX, 868e6, SX1278_POWER_11DBM, SX1278_LORA_SF_7,
	SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 10);
	printf("Done configuring LoRaModule RX\r\n");

	int ret = SX1278_LoRaEntryTx(&SX1278TX, 16, 2000);
	if (!ret) {
		printf("Failed to enter in transmission mode \r\n");
		return 1;
	}

	ret = SX1278_LoRaEntryRx(&SX1278RX, 16, 2000);
	if (!ret) {
		printf("Failed to enter in reception mode \r\n");
		return 1;
	}

	char buffer[512];
	char rx_buffer[512];

	int message = 0;
	int message_length = 0;
	ret = SX1278_LoRaEntryRx(&SX1278RX, message_length, 2000);

	while (1) {
		printf("Master ...\r\n");
		HAL_Delay(1000);
		printf("Sending package...\r\n");

//		message_length = sprintf(buffer, "Hello %d", message);
//		ret = SX1278_LoRaEntryTx(&SX1278TX, message_length, 2000);

//		printf("Entry: %d\r\n", ret);
//
//		printf("Sending %s\r\n", buffer);
//		ret = SX1278_LoRaTxPacket(&SX1278TX, (uint8_t*) buffer,
//				message_length, 2000);
		message_length = sprintf(buffer, "Hello %d", message);
		SX1278_transmit(&SX1278TX, (uint8_t*) buffer, message_length, 2000);
		message += 1;
		message_length = sprintf(buffer, "Hello %d", message);
		SX1278_transmit(&SX1278TX, (uint8_t*) buffer, message_length, 2000);
		message += 1;
		message_length = sprintf(buffer, "Hello %d", message);
		SX1278_transmit(&SX1278TX, (uint8_t*) buffer, message_length, 2000);
		message += 1;

		printf("Transmission: %d\r\n", ret);
		printf("Package sent...\r\n");
		HAL_Delay(150);

		int tries = 0;
		int available = 0;
		while (tries <= 4) {
			if (SX1278_available(&SX1278RX)) {
				available = 1;
				break;
			}
			HAL_Delay(100);
			tries++;
		}
//		int received_packet = SX1278_receive(&SX1278RX, message_length, 2000);
		if (!available) {
			printf("Failed to receive a packet\r\n");
			break;
		}
		SX1278_read(&SX1278RX, (uint8_t *)rx_buffer, message_length);
		for (int j = 0; j < message_length; j++) {
			printf("%d ", rx_buffer[j]);
		}
		printf("Received: %s \r\n", rx_buffer);


	}
}

