/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner/SX127X_LoRa
 */

#include "../SX127X.h"
#include <string.h>

/* All register accesses are issued as a single continuous SPI frame (one
 * HAL transfer with NSS held low for the whole exchange). Splitting the
 * address and data into separate HAL calls toggles the SPI enable bit between
 * bytes, which on the STM32H7 injects a spurious clock and shifts every byte by
 * one bit (e.g. RegVersion reads 0x09 instead of 0x12). */

uint8_t SX127X_SPIRead(SX127X_t *module, uint8_t addr) {
	uint8_t tx[2] = { (uint8_t) (addr & 0x7F), 0x00 };
	uint8_t rx[2] = { 0, 0 };
	SX127X_hw_SPITransfer(module->hw, tx, rx, 2);
	return rx[1];
}

void SX127X_SPIWrite(SX127X_t *module, uint8_t addr, uint8_t cmd) {
	uint8_t tx[2] = { (uint8_t) (addr | 0x80), cmd };
	uint8_t rx[2];
	SX127X_hw_SPITransfer(module->hw, tx, rx, 2);
}

void SX127X_SPIBurstRead(SX127X_t *module, uint8_t addr, uint8_t *rxBuf,
		uint8_t length) {
	if (length <= 1) {
		return;
	} else {
		uint8_t tx[SX127X_MAX_PACKET + 1] = { 0 };
		uint8_t rx[SX127X_MAX_PACKET + 1] = { 0 };
		tx[0] = (uint8_t) (addr & 0x7F);
		SX127X_hw_SPITransfer(module->hw, tx, rx, (uint16_t) length + 1);
		memcpy(rxBuf, &rx[1], length);
	}
}

void SX127X_SPIBurstWrite(SX127X_t *module, uint8_t addr, uint8_t *txBuf,
		uint8_t length) {
	if (length <= 1) {
		return;
	} else {
		uint8_t tx[SX127X_MAX_PACKET + 1];
		uint8_t rx[SX127X_MAX_PACKET + 1];
		tx[0] = (uint8_t) (addr | 0x80);
		memcpy(&tx[1], txBuf, length);
		SX127X_hw_SPITransfer(module->hw, tx, rx, (uint16_t) length + 1);
	}
}

void SX127X_config(SX127X_t *module) {
	SX127X_sleep(module); //Change modem mode Must in Sleep mode
	SX127X_hw_DelayMs(15);

	SX127X_entryLoRa(module);
	//SX127X_SPIWrite(module, 0x5904); //?? Change digital regulator form 1.6V to 1.47V: see errata note

#if SX127X_USE_TCXO
	/* Enable external TCXO clock input (RegTcxo.TcxoInputOn = bit 4). Required
	 * for TCXO-fed modules so the PLL has a reference and can lock for TX/RX.
	 * Reset value of RegTcxo is 0x09; 0x09 | 0x10 = 0x19. */
	SX127X_SPIWrite(module, REG_LR_TCXO, 0x19);
#endif

	uint64_t freq = ((uint64_t) module->frequency << 19) / 32000000;
	uint8_t freq_reg[3];
	freq_reg[0] = (uint8_t) (freq >> 16);
	freq_reg[1] = (uint8_t) (freq >> 8);
	freq_reg[2] = (uint8_t) (freq >> 0);
	SX127X_SPIBurstWrite(module, LR_RegFrMsb, (uint8_t*) freq_reg, 3); //setting  frequency parameter

	SX127X_SPIWrite(module, RegSyncWord, 0x34);

	//setting base parameter
	SX127X_SPIWrite(module, LR_RegPaConfig, SX127X_Power[module->power]); //Setting output power parameter

	SX127X_SPIWrite(module, LR_RegOcp, 0x0B);			//RegOcp,Close Ocp
	SX127X_SPIWrite(module, LR_RegLna, 0x23);		//RegLNA,High & LNA Enable
	if (SX127X_SpreadFactor[module->LoRa_SF] == 6) {	//SFactor=6
		uint8_t tmp;
		SX127X_SPIWrite(module,
		LR_RegModemConfig1,
				((SX127X_LoRaBandwidth[module->LoRa_BW] << 4)
						+ (SX127X_CodingRate[module->LoRa_CR] << 1) + 0x01)); //Implicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX127X_SPIWrite(module,
		LR_RegModemConfig2,
				((SX127X_SpreadFactor[module->LoRa_SF] << 4)
						+ (SX127X_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x03));

		tmp = SX127X_SPIRead(module, 0x31);
		tmp &= 0xF8;
		tmp |= 0x05;
		SX127X_SPIWrite(module, 0x31, tmp);
		SX127X_SPIWrite(module, 0x37, 0x0C);
	} else {
		SX127X_SPIWrite(module,
		LR_RegModemConfig1,
				((SX127X_LoRaBandwidth[module->LoRa_BW] << 4)
						+ (SX127X_CodingRate[module->LoRa_CR] << 1) + 0x00)); //Explicit Enable CRC Enable(0x02) & Error Coding rate 4/5(0x01), 4/6(0x02), 4/7(0x03), 4/8(0x04)

		SX127X_SPIWrite(module,
		LR_RegModemConfig2,
				((SX127X_SpreadFactor[module->LoRa_SF] << 4)
						+ (SX127X_CRC_Sum[module->LoRa_CRC_sum] << 2) + 0x00)); //SFactor &  LNA gain set by the internal AGC loop
	}

	SX127X_SPIWrite(module, LR_RegModemConfig3, 0x04);
	SX127X_SPIWrite(module, LR_RegSymbTimeoutLsb, 0x08); //RegSymbTimeoutLsb Timeout = 0x3FF(Max)
	SX127X_SPIWrite(module, LR_RegPreambleMsb, 0x00); //RegPreambleMsb
	SX127X_SPIWrite(module, LR_RegPreambleLsb, 8); //RegPreambleLsb 8+4=12byte Preamble
	SX127X_SPIWrite(module, REG_LR_DIOMAPPING2, 0x01); //RegDioMapping2 DIO5=00, DIO4=01
	module->readBytes = 0;
	SX127X_standby(module); //Entry standby mode
}

void SX127X_standby(SX127X_t *module) {
	SX127X_SPIWrite(module, LR_RegOpMode, 0x09);
	module->status = STANDBY;
}

void SX127X_sleep(SX127X_t *module) {
	SX127X_SPIWrite(module, LR_RegOpMode, 0x08);
	module->status = SLEEP;
}

void SX127X_entryLoRa(SX127X_t *module) {
	SX127X_SPIWrite(module, LR_RegOpMode, 0x88);
}

void SX127X_clearLoRaIrq(SX127X_t *module) {
	SX127X_SPIWrite(module, LR_RegIrqFlags, 0xFF);
}

int SX127X_LoRaEntryRx(SX127X_t *module, uint8_t length, uint32_t timeout) {
	uint8_t addr;

	module->packetLength = length;

	SX127X_config(module);		//Setting base parameter
	SX127X_SPIWrite(module, REG_LR_PADAC, 0x84);	//Normal and RX
	SX127X_SPIWrite(module, LR_RegHopPeriod, 0xFF);	//No FHSS
	SX127X_SPIWrite(module, REG_LR_DIOMAPPING1, 0x01);//DIO=00,DIO1=00,DIO2=00, DIO3=01
	SX127X_SPIWrite(module, LR_RegIrqFlagsMask, 0x3F);//Open RxDone interrupt & Timeout
	SX127X_clearLoRaIrq(module);
	SX127X_SPIWrite(module, LR_RegPayloadLength, length);//Payload Length 21byte(this register must difine when the data long of one byte in SF is 6)
	addr = SX127X_SPIRead(module, LR_RegFifoRxBaseAddr); //Read RxBaseAddr
	SX127X_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr->FiFoAddrPtr
	SX127X_SPIWrite(module, LR_RegOpMode, 0x8d);	//Mode//Low Frequency Mode
	//SX127X_SPIWrite(module, LR_RegOpMode,0x05);	//Continuous Rx Mode //High Frequency Mode
	module->readBytes = 0;

	while (1) {
		if ((SX127X_SPIRead(module, LR_RegModemStat) & 0x04) == 0x04) {	//Rx-on going RegModemStat
			module->status = RX;
			return 1;
		}
		if (--timeout == 0) {
			SX127X_hw_Reset(module->hw);
			SX127X_config(module);
			return 0;
		}
		SX127X_hw_DelayMs(1);
	}
}

uint8_t SX127X_LoRaRxPacket(SX127X_t *module) {
	unsigned char addr;
	unsigned char packet_size;

	/* Only touch the FIFO once a whole packet has landed. RegRxNbBytes and
	 * RegFifoRxCurrentaddr are not valid while a reception is in progress, so
	 * polling this function right after (or during) the on-air time would read
	 * a garbage length and drain the FIFO mid-update. Poll the RxDone flag
	 * (bit 6) over SPI so this also works when DIO0 is not wired. */
	uint8_t irqFlags = SX127X_SPIRead(module, LR_RegIrqFlags);
	if ((irqFlags & 0x40) == 0) {
		return module->readBytes;
	}
	if (irqFlags & 0x20) { /* PayloadCrcError: drop the corrupted packet */
		SX127X_clearLoRaIrq(module);
		return module->readBytes;
	}

		memset(module->rxBuffer, 0x00, SX127X_MAX_PACKET);

		addr = SX127X_SPIRead(module, LR_RegFifoRxCurrentaddr); //last packet addr
		SX127X_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RxBaseAddr -> FiFoAddrPtr

		if (module->LoRa_SF == SX127X_LORA_SF_6) { //When SpreadFactor is six,will used Implicit Header mode(Excluding internal packet length)
			packet_size = module->packetLength;
		} else {
			packet_size = SX127X_SPIRead(module, LR_RegRxNbBytes); //Number for received bytes
		}

		SX127X_SPIBurstRead(module, 0x00, module->rxBuffer, packet_size);
		module->readBytes = packet_size;
		SX127X_clearLoRaIrq(module);

	return module->readBytes;
}

int SX127X_LoRaEntryTx(SX127X_t *module, uint8_t length, uint32_t timeout) {
	uint8_t addr;
	uint8_t temp;

	module->packetLength = length;

	SX127X_config(module); //setting base parameter
	SX127X_SPIWrite(module, REG_LR_PADAC, 0x87);	//Tx for 20dBm
	SX127X_SPIWrite(module, LR_RegHopPeriod, 0x00); //RegHopPeriod NO FHSS
	SX127X_SPIWrite(module, REG_LR_DIOMAPPING1, 0x41); //DIO0=01, DIO1=00,DIO2=00, DIO3=01
	SX127X_clearLoRaIrq(module);
	SX127X_SPIWrite(module, LR_RegIrqFlagsMask, 0xF7); //Open TxDone interrupt
	SX127X_SPIWrite(module, LR_RegPayloadLength, length); //RegPayloadLength 21byte
	addr = SX127X_SPIRead(module, LR_RegFifoTxBaseAddr); //RegFiFoTxBaseAddr
	SX127X_SPIWrite(module, LR_RegFifoAddrPtr, addr); //RegFifoAddrPtr

	while (1) {
		temp = SX127X_SPIRead(module, LR_RegPayloadLength);
		if (temp == length) {
			module->status = TX;
			return 1;
		}

		if (--timeout == 0) {
			SX127X_hw_Reset(module->hw);
			SX127X_config(module);
			return 0;
		}
	}
}

int SX127X_LoRaTxPacket(SX127X_t *module, uint8_t *txBuffer, uint8_t length,
		uint32_t timeout) {
	SX127X_SPIBurstWrite(module, 0x00, txBuffer, length);
	SX127X_SPIWrite(module, LR_RegOpMode, 0x8b);	//Tx Mode
	while (1) {
		/* Detect TxDone either on the DIO0 pin or via the TxDone flag (bit 3 of
		 * RegIrqFlags) read over SPI, so transmission completes even when DIO0
		 * is not wired to the MCU. */
		if (SX127X_hw_GetDIO0(module->hw)
				|| (SX127X_SPIRead(module, LR_RegIrqFlags) & 0x08)) {
			SX127X_SPIRead(module, LR_RegIrqFlags);
			SX127X_clearLoRaIrq(module); //Clear irq
			SX127X_standby(module); //Entry Standby mode
			return 1;
		}

		if (--timeout == 0) {
			SX127X_hw_Reset(module->hw);
			SX127X_config(module);
			return 0;
		}
		SX127X_hw_DelayMs(1);
	}
}

void SX127X_init(SX127X_t *module, uint64_t frequency, uint8_t power,
		uint8_t LoRa_SF, uint8_t LoRa_BW, uint8_t LoRa_CR,
		uint8_t LoRa_CRC_sum, uint8_t packetLength) {
	SX127X_hw_init(module->hw);
	module->frequency = frequency;
	module->power = power;
	module->LoRa_SF = LoRa_SF;
	module->LoRa_BW = LoRa_BW;
	module->LoRa_CR = LoRa_CR;
	module->LoRa_CRC_sum = LoRa_CRC_sum;
	module->packetLength = packetLength;
	SX127X_config(module);
}

int SX127X_transmit(SX127X_t *module, uint8_t *txBuf, uint8_t length,
		uint32_t timeout) {
	if (SX127X_LoRaEntryTx(module, length, timeout)) {
		return SX127X_LoRaTxPacket(module, txBuf, length, timeout);
	}
	return 0;
}

int SX127X_receive(SX127X_t *module, uint8_t length, uint32_t timeout) {
	return SX127X_LoRaEntryRx(module, length, timeout);
}

uint8_t SX127X_available(SX127X_t *module) {
	return SX127X_LoRaRxPacket(module);
}

uint8_t SX127X_read(SX127X_t *module, uint8_t *rxBuf, uint8_t length) {
	if (length != module->readBytes)
		length = module->readBytes;
	memcpy(rxBuf, module->rxBuffer, length);
	rxBuf[length] = '\0';
	module->readBytes = 0;
	return length;
}

uint8_t SX127X_RSSI_LoRa(SX127X_t *module) {
	uint32_t temp = 10;
	temp = SX127X_SPIRead(module, LR_RegRssiValue); //Read RegRssiValue, Rssi value
	temp = temp + 127 - 137; //127:Max RSSI, 137:RSSI offset
	return (uint8_t) temp;
}

uint8_t SX127X_RSSI(SX127X_t *module) {
	uint8_t temp = 0xff;
	temp = SX127X_SPIRead(module, RegRssiValue);
	temp = 127 - (temp >> 1);	//127:Max RSSI
	return temp;
}
