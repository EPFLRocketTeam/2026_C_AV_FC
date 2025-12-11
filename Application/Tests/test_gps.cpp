/*
 * test_gps.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: maxime
 */


#include <gtest/gtest.h>

#include "Drivers/UBX_GPS/ubx_gps_interface.h"
#include "Drivers/STM32HAL/stm32hal.h"



TEST(UbxGPS, TestInit) {


    UbxGpsInterface gps(&huart2, 1000); // 1Hz update rate


      GpsStatus status = gps.init();

      EXPECT_EQ(status, GpsStatus::OK);


      // Main loop: read data every second
//      GpsBasicFixData fixData;
//
//      while (1) {
//        status = gps.getPvt(&fixData, 2000); // timeout 2s
//
//        if (status == GpsStatus::OK) {
//          double lat_deg = fixData.lat * UBX_SCALE_LAT_LON;
//          double lon_deg = fixData.lon * UBX_SCALE_LAT_LON;
//
//          printf("Fix OK | Type: %d | Sats: %2d | Lat: | Lon: | hAcc: %lu cm\r\n",
//                 static_cast<int>(fixData.fixType),
//                 fixData.numSV,
//
//                 (int)fixData.hAcc);
//        } else if (status == GpsStatus::ERROR_TIMEOUT) {
//          printf("No GPS fix received (timeout)\r\n");
//        } else {
//          printf("GPS read error: %d\r\n", static_cast<int>(status));
//        }
//
//        HAL_Delay(1000);
//      }

}

