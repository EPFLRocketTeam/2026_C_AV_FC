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
