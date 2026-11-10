#include <gtest/gtest.h>

#include <cstring>

#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"
#include "Drivers/ERT_RF_Protocol_Interface/DownlinkCompression_Firehorn2.h"

namespace {

av_downlink_unpacked_t SampleUnpacked(uint8_t prc_alive) {
    av_downlink_unpacked_t u;
    std::memset(&u, 0, sizeof(u));
    u.packet_nbr        = 12345;
    u.av_timestamp      = 678901;
    u.gnss_lat          = 46.5f;
    u.gnss_lon          = 6.6f;
    u.gnss_alt          = 1200.0f;
    u.HPO_pressure      = 300.0f;
    u.LOX_fls_temp_6    = -150.0f;
    u.chamber_pressure  = 30.0f;
    u.valves_state      = 0x2A;
    u.hpb_main_voltage  = 24.0f;
    u.av_state          = 5;
    u.cam_rec           = 0x5;
    u.pyro_status       = 0x9;
    u.average_imu_rate  = 40.0f;
    u.baro_count        = 3;
    u.prc_alive         = prc_alive;
    return u;
}

av_downlink_t Encode(uint8_t prc_alive) {
    av_downlink_t p;
    std::memset(&p, 0, sizeof(p));
    encode_downlink(&p, SampleUnpacked(prc_alive));
    return p;
}

} // namespace

TEST(DownlinkPrcAlive, SizeIsUnchanged) {
    EXPECT_EQ(sizeof(av_downlink_t), 55u);
}

// prc_alive sits at bits 433 to 435 (byte 54, bits 1 to 3, GCC packs
// bitfields LSB first), so changing it must flip nothing else.
TEST(DownlinkPrcAlive, RoundTripsEveryValueAndTouchesNothingElse) {
    // The encoder keeps sd_fail_count state across calls: its first packet
    // carries the reset code, the next ones a zero delta. Warm it up so the
    // base and every compared packet share the same sd field.
    (void) Encode(0);
    const av_downlink_t base = Encode(0);
    const uint8_t* base_bytes = reinterpret_cast<const uint8_t*>(&base);

    for (uint8_t m = 0; m < 8; m++) {
        const av_downlink_t p = Encode(m);
        const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&p);
        for (size_t i = 0; i < sizeof(av_downlink_t); i++) {
            const uint8_t diff = bytes[i] ^ base_bytes[i];
            const uint8_t expected = (i == 54) ? static_cast<uint8_t>(m << 1) : 0;
            EXPECT_EQ(diff, expected) << "byte " << i << " value " << int(m);
        }

        const av_downlink_unpacked_t d = decode_downlink(p);
        EXPECT_EQ(d.prc_alive, m);
        EXPECT_EQ(d.packet_nbr, 12345u);
        EXPECT_EQ(d.av_timestamp, 678901u);
        EXPECT_EQ(d.valves_state, 0x2A);
        EXPECT_EQ(d.av_state, 5);
        EXPECT_EQ(d.cam_rec, 0x5);
        EXPECT_EQ(d.pyro_status, 0x9);
        EXPECT_EQ(d.baro_count, 3);
    }
}

TEST(DownlinkPrcAlive, EncodeMasksToThreeBits) {
    const av_downlink_unpacked_t d = decode_downlink(Encode(0xFF));
    EXPECT_EQ(d.prc_alive, 0x7);
}
