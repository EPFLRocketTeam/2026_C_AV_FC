/*
 * test_gps.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: maxime
 */

#include <gtest/gtest.h>

#include "Drivers/STM32HAL/stm32hal.h"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"

static inline void setU4(uint8_t *p, uint32_t val) {
  p[0] = val & 0xFF;
  p[1] = (val >> 8) & 0xFF;
  p[2] = (val >> 16) & 0xFF;
  p[3] = (val >> 24) & 0xFF;
}

static inline void setI4(uint8_t *p, int32_t val) {
  setU4(p, static_cast<uint32_t>(val));
}

static inline void setU2(uint8_t *p, uint16_t val) {
  p[0] = val & 0xFF;
  p[1] = (val >> 8) & 0xFF;
}

static void ubxChecksum(const uint8_t *data, size_t len, uint8_t &ckA,
                        uint8_t &ckB) {
  ckA = 0;
  ckB = 0;
  for (size_t i = 0; i < len; ++i) {
    ckA += data[i];
    ckB += ckA;
  }
}

int buildUbxPvtPacket(uint8_t **buffer, uint16_t *len) {
  if (!buffer || !len)
    return -1; // Invalid arguments

  // Prefill dummy GpsPvtData
  GpsPvtData data = {};
  data.iTOW = 450000;
  data.year = 2025;
  data.month = 12;
  data.day = 11;
  data.hour = 17;
  data.min = 59;
  data.sec = 30;
  data.valid = {true, true, true, false};
  data.tAcc = 200;
  data.nano = 500000;
  data.fixType = GpsFixType::FIX_3D;
  data.flags = {true, true, GpsPowerSaveMode::NOT_ACTIVE, true,
                GpsCarrierPhaseStatus::FLOAT};
  data.numSV = 15;
  data.lon = static_cast<int32_t>(8.541694 * 1e7);
  data.lat = static_cast<int32_t>(47.376888 * 1e7);
  data.height = 408000;
  data.hMSL = 406000;
  data.hAcc = 300;
  data.vAcc = 500;
  data.velN = 100;
  data.velE = 50;
  data.velD = -10;
  data.gSpeed = 112;
  data.headMot = static_cast<int32_t>(90.0 * 1e5);
  data.sAcc = 20;
  data.headAcc = 100000;
  data.pDOP = 150;

  // Prepare buffer (allocate dynamically)
  const uint16_t maxPacketSize = 100;
  *buffer = (uint8_t *)malloc(maxPacketSize);
  if (!*buffer)
    return -2; // Memory allocation failure

  uint8_t *packet = *buffer;
  uint8_t *payload = packet + 6; // skip header

  // Build payload
  uint8_t *p = payload;
  setU4(p, data.iTOW);
  p += 4;
  setU2(p, data.year);
  p += 2;
  *p++ = data.month;
  *p++ = data.day;
  *p++ = data.hour;
  *p++ = data.min;
  *p++ = data.sec;
  uint8_t validByte = 0;
  validByte |= (data.valid.validDate ? 0x01 : 0);
  validByte |= (data.valid.validTime ? 0x02 : 0);
  validByte |= (data.valid.fullyResolved ? 0x04 : 0);
  validByte |= (data.valid.validMag ? 0x08 : 0);
  *p++ = validByte;
  setU4(p, data.tAcc);
  p += 4;
  setI4(p, data.nano);
  p += 4;
  *p++ = static_cast<uint8_t>(data.fixType);
  uint8_t flags = 0;
  flags |= (data.flags.gnssFixOK ? 0x01 : 0);
  flags |= (data.flags.diffSoln ? 0x02 : 0);
  flags |= ((static_cast<uint8_t>(data.flags.psmState) & 0x07) << 2);
  flags |= (data.flags.headVehValid ? 0x20 : 0);
  flags |= ((static_cast<uint8_t>(data.flags.carrSoln) & 0x03) << 6);
  *p++ = flags;
  *p++ = 0; // flags2 reserved
  *p++ = data.numSV;
  setI4(p, data.lon);
  p += 4;
  setI4(p, data.lat);
  p += 4;
  setI4(p, data.height);
  p += 4;
  setI4(p, data.hMSL);
  p += 4;
  setU4(p, data.hAcc);
  p += 4;
  setU4(p, data.vAcc);
  p += 4;
  setI4(p, data.velN);
  p += 4;
  setI4(p, data.velE);
  p += 4;
  setI4(p, data.velD);
  p += 4;
  setI4(p, data.gSpeed);
  p += 4;
  setI4(p, data.headMot);
  p += 4;
  setU4(p, data.sAcc);
  p += 4;
  setU4(p, data.headAcc);
  p += 4;
  setU2(p, data.pDOP);
  p += 2;

  uint16_t payloadLen = p - payload;

  // Add UBX header
  packet[0] = 0xB5; // SYNC1
  packet[1] = 0x62; // SYNC2
  packet[2] = 0x01; // NAV
  packet[3] = 0x07; // PVT
  setU2(&packet[4], payloadLen);

  // Compute checksum
  uint8_t ckA, ckB;
  ubxChecksum(&packet[2], payloadLen + 4, ckA, ckB);
  *p++ = ckA;
  *p++ = ckB;

  *len = p - packet;
  return 0; // Success
}

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
  //          printf("Fix OK | Type: %d | Sats: %2d | Lat: | Lon: | hAcc: %lu
  //          cm\r\n",
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

TEST(UbxGPS, TestGetPVT) {
	fprintf(stdout, "We enter here-----------------\n");
  UbxGpsInterface gps(&huart2, 1000); // 1Hz update rate

  GpsStatus status = gps.init();

  EXPECT_EQ(status, GpsStatus::OK);

  MockHAL::Init();
  // uint8_t* buf;
  // uint16_t len;
  // int ret = buildUbxPvtPacket(&buf, &len);
  // fprintf(stdout, "The size is %d\n", len);
  // std::vector<uint8_t> ubxVec(buf, buf + len);
  // MockHAL::AddFakeUbxMessage(ubxVec);
  MockHAL::AddFakePvtMessage();
  // fprintf(stdout, "Added ubx to the buf\n");
  // EXPECT_EQ(ret, 0);
  // Main loop: read data every second
  GpsBasicFixData fixData;
  uint8_t buf_lon[] = {0x2C, 0x72, 0x09, 0x05};
  uint8_t buf_lat[] = {0x60, 0xE6, 0x3A, 0x02};

  status = gps.getPvt(&fixData, 2000); // timeout 2s
  EXPECT_EQ(status, GpsStatus::OK);

  double expected_lon = static_cast<double>(*(int*)buf_lon * 1e-7);
  double expected_lat = static_cast<double>(*(int*)buf_lat * 1e-7);

  double lat_deg = fixData.lat * UBX_SCALE_LAT_LON;
  double lon_deg = fixData.lon * UBX_SCALE_LAT_LON;

  EXPECT_EQ(expected_lon, lon_deg);
  EXPECT_EQ(expected_lat, lat_deg);
}
