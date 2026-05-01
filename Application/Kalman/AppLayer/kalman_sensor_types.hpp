#pragma once

#include <stddef.h>
#include <stdint.h>

namespace app {

struct ImuSample {
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
  int8_t temperature = 0;
  uint16_t internal_timestamp = 0;
};

struct ImuBatch {
  const ImuSample* data = nullptr;
  size_t count = 0;
  uint64_t t0_us = 0;
  uint32_t dt_us = 0;
  uint8_t source = 0;
  uint8_t slot = 0xFF;
};

struct BaroSample {
  float pressurePa = 0.0f;
  float temperatureC = 0.0f;
  uint64_t timestamp_us = 0;
  uint8_t source = 0;
};

struct BaroBatch {
  const BaroSample* data = nullptr;
  size_t count = 0;
  uint64_t t0_us = 0;
  uint32_t dt_us = 0;
};

namespace sensors {
namespace mmc5983ma {
struct MmcSample {
  uint32_t x = 0;
  uint32_t y = 0;
  uint32_t z = 0;
  uint64_t t_us = 0;
};
}  // namespace mmc5983ma

namespace gnss {
struct GnssSample {
  uint64_t timestamp_us = 0;
  uint64_t pps_timestamp_us = 0;
  int32_t lat_deg7 = 0;
  int32_t lon_deg7 = 0;
  int32_t alt_msl_mm = 0;
  int32_t alt_ellipsoid_mm = 0;
  int32_t vel_n_mms = 0;
  int32_t vel_e_mms = 0;
  int32_t vel_d_mms = 0;
  int32_t ground_speed_mms = 0;
  int32_t heading_deg5 = 0;
  uint32_t h_acc_mm = 0;
  uint32_t v_acc_mm = 0;
  uint32_t s_acc_mms = 0;
  uint32_t head_acc_deg5 = 0;
  uint16_t pdop = 0;
  uint16_t hdop = 0;
  uint16_t vdop = 0;
  uint16_t gdop = 0;
  uint16_t tdop = 0;
  uint8_t fix_type = 0;
  uint8_t num_sv = 0;
  uint8_t flags = 0;
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  uint8_t hour = 0;
  uint8_t min = 0;
  uint8_t sec = 0;
  int32_t nano = 0;
  uint8_t time_valid = 0;
  uint32_t itow_ms = 0;
  bool valid = false;
};
}  // namespace gnss
}  // namespace sensors

}  // namespace app
