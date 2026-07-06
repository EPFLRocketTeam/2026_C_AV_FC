/*
 * sx127x_capsule_manual_test.cpp
 *
 * Manual hardware test for the SX127XCapsule wrapper: the TX module sends
 * Capsule-framed packets, the RX module drains its buffer through the
 * Capsule parser and the callback checks id + payload round-trip intact.
 *
 *  Created on: Jul 6, 2026
 *      Author: maxime
 */

#include "sx127x_capsule_manual_test.h"
#include "../../SX127X_capsule.hpp"
#include "main.h"
#include "stm32h7xx_hal.h"
extern "C" {
#include <cstdio>
}
#include <cstring>

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi1;

#define TEST_PAYLOAD_LEN	10
#define TEST_PACKET_ID		0x2A

static uint8_t expectedPayload[TEST_PAYLOAD_LEN];
static int packetsDecoded = 0;
static int packetsCorrupted = 0;

static void onPacket(uint8_t packetId, uint8_t *payload, uint32_t length) {
	printf("Decoded packet: id=0x%02X len=%lu payload=[%s]\r\n", packetId,
			(unsigned long) length, (char*) payload);

	if (packetId != TEST_PACKET_ID || length != TEST_PAYLOAD_LEN
			|| memcmp(payload, expectedPayload, TEST_PAYLOAD_LEN) != 0) {
		printf("  MISMATCH: expected id=0x%02X len=%d payload=[%s]\r\n",
				TEST_PACKET_ID, TEST_PAYLOAD_LEN, (char*) expectedPayload);
		packetsCorrupted++;
	} else {
		packetsDecoded++;
	}
}

int sx127x_capsule_manual_test(void) {
	SX127X_hw_t SX127X_TX_hw;
	SX127X_hw_t SX127X_RX_hw;

	SX127X_RX_hw.dio0.port = GPIO_RFM_RX_INT0_GPIO_Port;
	SX127X_RX_hw.dio0.pin = GPIO_RFM_RX_INT0_Pin;
	SX127X_RX_hw.nss.port = SPI_RFM_RX_CS_GPIO_Port;
	SX127X_RX_hw.nss.pin = SPI_RFM_RX_CS_Pin;
	SX127X_RX_hw.reset.port = GPIO_RFM_RX_RST_GPIO_Port;
	SX127X_RX_hw.reset.pin = GPIO_RFM_RX_RST_Pin;
	SX127X_RX_hw.spi = &hspi2;

	SX127X_TX_hw.dio0.port = GPIO_RFM_TX_INT0_GPIO_Port;
	SX127X_TX_hw.dio0.pin = GPIO_RFM_TX_INT0_Pin;
	SX127X_TX_hw.nss.port = SPI_RFM_TX_CS_GPIO_Port;
	SX127X_TX_hw.nss.pin = SPI_RFM_TX_CS_Pin;
	SX127X_TX_hw.reset.port = GPIO_RFM_TX_RST_GPIO_Port;
	SX127X_TX_hw.reset.pin = GPIO_RFM_TX_RST_Pin;
	SX127X_TX_hw.spi = &hspi1;

	SX127XCapsule tx(&SX127X_TX_hw, nullptr);
	SX127XCapsule rx(&SX127X_RX_hw, onPacket);

	/* SPI read sanity check: every SX127x returns 0x12 in RegVersion (0x42). */
	SX127X_hw_init(&SX127X_TX_hw);
	SX127X_hw_Reset(&SX127X_TX_hw);
	uint8_t version = SX127X_SPIRead(tx.raw(), RegVersion);
	printf("TX RegVersion = 0x%02X (expect 0x12)\r\n", version);

	SX127X_hw_init(&SX127X_RX_hw);
	SX127X_hw_Reset(&SX127X_RX_hw);
	version = SX127X_SPIRead(rx.raw(), RegVersion);
	printf("RX RegVersion = 0x%02X (expect 0x12)\r\n", version);

	printf("Configuring LoRa module TX\r\n");
	tx.init(868e6, SX127X_POWER_11DBM, SX127X_LORA_SF_7,
	SX127X_LORA_BW_125KHZ, SX127X_LORA_CR_4_5, SX127X_LORA_CRC_EN,
	TEST_PAYLOAD_LEN);

//	int ret = SX127X_LoRaEntryTx(tx.raw(), TEST_PAYLOAD_LEN, 2000);
//	if (!ret) {
//		printf("Failed to enter in transmission mode \r\n");
//		return 1;
//	}

	printf("Configuring LoRa module RX\r\n");
	rx.init(868e6, SX127X_POWER_11DBM, SX127X_LORA_SF_7,
	SX127X_LORA_BW_125KHZ, SX127X_LORA_CR_4_5, SX127X_LORA_CRC_EN,
	TEST_PAYLOAD_LEN);

	if (!rx.receive(2000)) {
		printf("Failed to enter in reception mode\r\n");
		return 1;
	}

	int message = 0;

	while (1) {
		printf("Master ...\r\n");
		HAL_Delay(1000);

		/* Fixed-size payload: the radios are configured for exactly
		 * TEST_PAYLOAD_LEN payload bytes per frame. */
		memset(expectedPayload, 0, sizeof(expectedPayload));
		snprintf((char*) expectedPayload, sizeof(expectedPayload), "Hello %02d",
				message % 100);

		printf("Sending [%s]...\r\n", (char*) expectedPayload);
		if (!tx.transmit(TEST_PACKET_ID, expectedPayload, TEST_PAYLOAD_LEN,
				2000)) {
			printf("Transmission failed\r\n");
			break;
		}
		message++;

		/* Wait for the frame to land, then run it through the decoder. */
		int available = 0;
		for (int tries = 0; tries <= 4; tries++) {
			if (rx.available()) {
				available = 1;
				break;
			}
			HAL_Delay(100);
		}

		if (!available) {
			printf("Failed to receive a packet\r\n");
			break;
		}

		uint8_t consumed = rx.read(); // fires onPacket on each valid frame
		printf("Consumed %d raw bytes, decoded=%d corrupted=%d\r\n", consumed,
				packetsDecoded, packetsCorrupted);

		/* Re-arm reception for the next frame. */
		if (!rx.receive(2000)) {
			printf("Failed to re-enter reception mode\r\n");
			break;
		}
	}

	printf("Test stopped: decoded=%d corrupted=%d sent=%d\r\n", packetsDecoded,
			packetsCorrupted, message);
	return 1;
}
