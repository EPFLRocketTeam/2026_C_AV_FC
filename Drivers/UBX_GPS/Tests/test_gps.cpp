/*
 * test_gps.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: maxime
 */

#include <gtest/gtest.h>

#include "Drivers/UBX_GPS/ubx_gps_interface.h"
#include "Drivers/STM32HAL/Interfaces/mock_uart.hpp" // New Interface
#include "ubx_test_helpers.hpp"                      // New Helper
//
class UbxGPSTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        stm32sim_ticks_init();
    }

    void TearDown() override
    {
        stm32sim_ticks_deinit();
    }
};

// Define a handle for the GPS UART
static UART_HandleTypeDef huart_gps = {10, "GPS_UART"};

TEST_F(UbxGPSTest, TestInit) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);

    UbxGpsInterface gps(mockGps.getHandle(), 1000); // 1Hz update rate

    GpsStatus status = gps.init();

    EXPECT_EQ(status, GpsStatus::OK);
}

TEST_F(UbxGPSTest, TestGetPVT) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);

    UbxGpsInterface gps(mockGps.getHandle(), 1000); // 1Hz update rate
    GpsStatus status = gps.init();
    EXPECT_EQ(status, GpsStatus::OK);

    std::vector<uint8_t> fakePacket = UBXTestHelpers::createDefaultPvtPacket();

    mockGps.feedRx(fakePacket);

    GpsBasicFixData fixData;
    status = gps.getPvt(&fixData, 2000);
    EXPECT_EQ(status, GpsStatus::OK);

    uint8_t buf_lon[] = {0x2C, 0x72, 0x09, 0x05};
    uint8_t buf_lat[] = {0x60, 0xE6, 0x3A, 0x02};


    double expected_lon = static_cast<double>(*(int*)buf_lon * 1e-7);
    double expected_lat = static_cast<double>(*(int*)buf_lat * 1e-7);

    double lat_deg = fixData.lat * UBX_SCALE_LAT_LON;
    double lon_deg = fixData.lon * UBX_SCALE_LAT_LON;

    EXPECT_NEAR(expected_lon, lon_deg, 0.000001);
    EXPECT_NEAR(expected_lat, lat_deg, 0.000001);
}

// ---------------------------------------------------------------------------
// Additional field-level checks on the default packet
// ---------------------------------------------------------------------------

TEST_F(UbxGPSTest, TestGetPVT_ParsesFixTypeAndNumSV) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    mockGps.feedRx(UBXTestHelpers::createDefaultPvtPacket());

    GpsBasicFixData fix{};
    ASSERT_EQ(gps.getPvt(&fix, 2000), GpsStatus::OK);

    EXPECT_EQ(fix.fixType, GpsFixType::FIX_3D);
    EXPECT_EQ(fix.numSV,   8);
}

TEST_F(UbxGPSTest, TestGetPVT_ParsesValidityFlags) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    // Default packet has valid byte = 0x0F → all four flags set
    mockGps.feedRx(UBXTestHelpers::createDefaultPvtPacket());

    GpsBasicFixData fix{};
    ASSERT_EQ(gps.getPvt(&fix, 2000), GpsStatus::OK);

    EXPECT_TRUE(fix.valid.validDate);
    EXPECT_TRUE(fix.valid.validTime);
    EXPECT_TRUE(fix.valid.fullyResolved);
    EXPECT_TRUE(fix.valid.validMag);
    EXPECT_TRUE(fix.flags.gnssFixOK);
}

TEST_F(UbxGPSTest, TestGetPVT_ParsesHAcc) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    // Default packet hAcc bytes = {0x50, 0x00, 0x00, 0x00} → 80 mm
    mockGps.feedRx(UBXTestHelpers::createDefaultPvtPacket());

    GpsBasicFixData fix{};
    ASSERT_EQ(gps.getPvt(&fix, 2000), GpsStatus::OK);

    EXPECT_EQ(fix.hAcc, 80u);
}

// ---------------------------------------------------------------------------
// Custom packet builder verification
// ---------------------------------------------------------------------------

TEST_F(UbxGPSTest, TestGetPVT_CustomPacketFields) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    UBXTestHelpers::PvtPacketParams p;
    p.lat         = 473768880;   // 47.376888°
    p.lon         = 85052780;    // 8.505278°
    p.hAcc        = 1500;        // 1500 mm
    p.fixType     = 2;           // 2-D fix
    p.numSV       = 5;
    p.validFlags  = 0x03;        // only date+time valid
    p.statusFlags = 0x00;        // gnssFixOK not set

    mockGps.feedRx(UBXTestHelpers::createCustomPvtPacket(p));

    GpsBasicFixData fix{};
    ASSERT_EQ(gps.getPvt(&fix, 2000), GpsStatus::OK);

    EXPECT_EQ(fix.lat,    473768880);
    EXPECT_EQ(fix.lon,    85052780);
    EXPECT_EQ(fix.hAcc,   1500u);
    EXPECT_EQ(fix.fixType, GpsFixType::FIX_2D);
    EXPECT_EQ(fix.numSV,  5);
    EXPECT_TRUE(fix.valid.validDate);
    EXPECT_TRUE(fix.valid.validTime);
    EXPECT_FALSE(fix.valid.fullyResolved);
    EXPECT_FALSE(fix.flags.gnssFixOK);
}

// ---------------------------------------------------------------------------
// Error / edge-case scenarios
// ---------------------------------------------------------------------------

TEST_F(UbxGPSTest, TestGetPVT_TimesOutWithNoData) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    // Feed nothing – getPvt must give up after the timeout
    GpsBasicFixData fix{};
    GpsStatus status = gps.getPvt(&fix, 150);
    EXPECT_EQ(status, GpsStatus::ERROR_TIMEOUT);
}

TEST_F(UbxGPSTest, TestGetPVT_BadChecksumCausesTimeout) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    // Packet with corrupted CK_B: parser resyncs and never finds a valid
    // frame, so getPvt() falls through to ERROR_TIMEOUT.
    mockGps.feedRx(UBXTestHelpers::createPvtPacketBadChecksum());

    GpsBasicFixData fix{};
    GpsStatus status = gps.getPvt(&fix, 150);
    EXPECT_EQ(status, GpsStatus::ERROR_TIMEOUT);
}

TEST_F(UbxGPSTest, TestGetPVT_MultipleConsecutiveCalls) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    // First packet: default values
    mockGps.feedRx(UBXTestHelpers::createDefaultPvtPacket());

    // Second packet: distinct lat/lon so we can tell the calls apart
    UBXTestHelpers::PvtPacketParams p2;
    p2.lat = 100000000;
    p2.lon = 200000000;
    mockGps.feedRx(UBXTestHelpers::createCustomPvtPacket(p2));

    GpsBasicFixData fix1{}, fix2{};
    ASSERT_EQ(gps.getPvt(&fix1, 2000), GpsStatus::OK);
    ASSERT_EQ(gps.getPvt(&fix2, 2000), GpsStatus::OK);

    // First call must have consumed the first packet
    EXPECT_EQ(fix1.lat, 37414496);
    // Second call must have consumed the second packet
    EXPECT_EQ(fix2.lat, 100000000);
    EXPECT_EQ(fix2.lon, 200000000);
}

// ---------------------------------------------------------------------------
// setRate / getRate
// ---------------------------------------------------------------------------

TEST_F(UbxGPSTest, TestSetRate_ValidRate) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    EXPECT_EQ(gps.setRate(500), GpsStatus::OK);
    EXPECT_EQ(gps.getRate(), 500);
}

TEST_F(UbxGPSTest, TestSetRate_TooFastIsRejected) {
    SIMULATOR_NAMESPACE::interfaces::MockUARTDevice mockGps(&huart_gps);
    UbxGpsInterface gps(mockGps.getHandle(), 1000);
    ASSERT_EQ(gps.init(), GpsStatus::OK);

    // Anything below 25 ms should be rejected
    EXPECT_EQ(gps.setRate(10), GpsStatus::ERROR_CONFIG);
    // Rate must remain unchanged after a rejected call
    EXPECT_EQ(gps.getRate(), 1000);
}
