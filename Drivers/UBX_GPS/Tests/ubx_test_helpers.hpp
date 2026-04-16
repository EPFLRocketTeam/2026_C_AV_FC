#ifndef UBX_TEST_HELPERS_H
#define UBX_TEST_HELPERS_H

#include <cstdint>
#include <vector>
#include <cstring>

// Assuming GpsPvtData is defined in your interface header
// #include "Drivers/UBX_GPS/ubx_gps_interface.h"

namespace UBXTestHelpers {

inline void setU4(uint8_t *p, uint32_t val) {
    p[0] = val & 0xFF;
    p[1] = (val >> 8) & 0xFF;
    p[2] = (val >> 16) & 0xFF;
    p[3] = (val >> 24) & 0xFF;
}

inline void setI4(uint8_t *p, int32_t val) {
    setU4(p, static_cast<uint32_t>(val));
}

inline void setU2(uint8_t *p, uint16_t val) {
    p[0] = val & 0xFF;
    p[1] = (val >> 8) & 0xFF;
}

inline void ubxChecksum(const uint8_t *data, size_t len, uint8_t &ckA, uint8_t &ckB) {
    ckA = 0;
    ckB = 0;
    for (size_t i = 0; i < len; ++i) {
        ckA += data[i];
        ckB += ckA;
    }
}

/**
 * Creates a valid UBX NAV-PVT packet with hardcoded test values.
 * Matches the data previously in MockHAL::AddFakePvtMessage.
 */
inline std::vector<uint8_t> createDefaultPvtPacket() {
    std::vector<uint8_t> message;

    // UBX header
    message.push_back(0xB5);  // Sync char 1
    message.push_back(0x62);  // Sync char 2
    message.push_back(0x01);  // Class: NAV
    message.push_back(0x07);  // ID: PVT
    message.push_back(0x5C);  // Length LSB (92 bytes)
    message.push_back(0x00);  // Length MSB

    // Payload (92 bytes)
    const uint8_t payload[92] = {
        // iTOW (4 bytes)
        0x10, 0x27, 0x05, 0x00,
        // Year (2 bytes)
        0xE6, 0x07,
        // Month (1 byte)
        0x0C,
        // Day (1 byte)
        0x0B,
        // Hour (1 byte)
        0x0A,
        // Minute (1 byte)
        0x29,
        // Second (1 byte)
        0x15,
        // Valid (1 byte) - all flags set
        0x0F,
        // tAcc (4 bytes)
        0x20, 0x00, 0x00, 0x00,
        // nano (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // fixType (1 byte) - 3D fix
        0x03,
        // Flags (1 byte) - GNSS fix OK
        0x01,
        // Flags2 (1 byte)
        0x00,
        // numSV (1 byte)
        0x08,
        // lon (4 bytes) - 8.542278 degrees
        0x2C, 0x72, 0x09, 0x05,
        // lat (4 bytes) - 47.376888 degrees
        0x60, 0xE6, 0x3A, 0x02,
        // height (4 bytes)
        0x80, 0xF2, 0x04, 0x00,
        // hMSL (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // hAcc (4 bytes)
        0x50, 0x00, 0x00, 0x00,
        // vAcc (4 bytes)
        0x60, 0x00, 0x00, 0x00,
        // velN (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // velE (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // velD (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // gSpeed (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // headMot (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // sAcc (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // headAcc (4 bytes)
        0x00, 0x00, 0x00, 0x00,
        // pDOP (2 bytes)
        0x80, 0x00,
        // Remaining padding to reach 92 bytes
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    // Add payload to message
    for (int i = 0; i < 92; i++) {
        message.push_back(payload[i]);
    }

    // Calculate checksum over header (class/id/len) + payload
    uint8_t ck_a = 0, ck_b = 0;
    // Start checksum from byte 2 (Class) to end of payload
    ubxChecksum(message.data() + 2, message.size() - 2, ck_a, ck_b);

    message.push_back(ck_a);
    message.push_back(ck_b);

    return message;
}

} // namespace UBXTestHelpers

#endif // UBX_TEST_HELPERS_H
