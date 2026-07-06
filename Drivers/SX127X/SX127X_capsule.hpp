/**
 * C++ convenience wrapper combining the SX127X LoRa driver with the
 * Capsule framing protocol.
 *
 * TX path: payload -> Capsule frame (preambles, id, length, checksum)
 *          -> SX127X_transmit
 * RX path: SX127X_read -> Capsule parser -> user callback fired once a
 *          full valid frame has been decoded
 *
 * C++ only: include this from .cpp files, not from the C driver.
 */

#ifndef __SX127X_CAPSULE_HPP__
#define __SX127X_CAPSULE_HPP__

#include "SX127X.h"
#include "capsule.h"

class SX127XCapsule {
public:
	/** Called with (packetId, payload, payloadLength) for every frame whose
	 *  Capsule checksum verified. */
	using PacketCallback = void (*)(uint8_t packetId, uint8_t *payload,
			uint32_t length);

	/**
	 * \param[in] hw       Hardware description (pins/SPI), must outlive this
	 *                     object. Caller runs SX127X_hw_init/Reset as usual.
	 * \param[in] onPacket Callback invoked from read() on each decoded frame
	 */
	SX127XCapsule(SX127X_hw_t *hw, PacketCallback onPacket) :
			capsule(onPacket) {
		module.hw = hw;
	}

	/**
	 * Configure the radio. Same parameters as SX127X_init(), except
	 * payloadLength is the *user* payload size: the Capsule framing overhead
	 * is added internally so TX and RX sides agree on the on-air length.
	 */
	void init(uint64_t frequency, uint8_t power, uint8_t LoRa_SF,
			uint8_t LoRa_BW, uint8_t LoRa_CR, uint8_t LoRa_CRC_sum,
			uint8_t payloadLength) {
		SX127X_init(&module, frequency, power, LoRa_SF, LoRa_BW, LoRa_CR,
				LoRa_CRC_sum, capsule.getCodedLen(payloadLength));
	}

	/**
	 * Encode payload into a Capsule frame and transmit it.
	 *
	 * \return true if the frame was sent, false on timeout or if the encoded
	 *         frame would exceed SX127X_MAX_PACKET
	 */
	bool transmit(uint8_t packetId, const uint8_t *payload, uint8_t length,
			uint32_t timeout = SX127X_DEFAULT_TIMEOUT) {
		uint32_t codedLen = capsule.getCodedLen(length);
		if (codedLen > SX127X_MAX_PACKET) {
			return false;
		}
		uint8_t *frame = capsule.encode(packetId,
				const_cast<uint8_t*>(payload), length);
		int sent = SX127X_transmit(&module, frame,
				static_cast<uint8_t>(codedLen), timeout);
		delete[] frame; // encode() allocates on the heap
		return sent == 1;
	}

	/**
	 * Enter reception mode, listening for frames of the length configured in
	 * init().
	 *
	 * \return true if RX mode was entered, false on timeout
	 */
	bool receive(uint32_t timeout = SX127X_DEFAULT_TIMEOUT) {
		return SX127X_receive(&module, module.packetLength, timeout) == 1;
	}

	/**
	 * \return number of received bytes pending in the radio buffer
	 *         (0 if nothing arrived yet)
	 */
	uint8_t available() {
		return SX127X_available(&module);
	}

	/**
	 * Drain the radio buffer through the Capsule parser. The callback fires
	 * once per complete frame with a valid checksum; corrupted frames are
	 * dropped silently and the parser resynchronizes on the next preamble.
	 *
	 * \return number of raw bytes consumed from the radio
	 */
	uint8_t read() {
		uint8_t buffer[SX127X_MAX_PACKET + 1]; // +1: SX127X_read appends '\0'
		uint8_t bytes = SX127X_read(&module, buffer, UINT8_MAX);
		for (uint8_t i = 0; i < bytes; i++) {
			capsule.decode(buffer[i]);
		}
		return bytes;
	}

	/** Escape hatch to the underlying driver (RSSI, standby, sleep, ...). */
	SX127X_t* raw() {
		return &module;
	}

private:
	SX127X_t module;
	CapsuleStatic capsule;
};

#endif
