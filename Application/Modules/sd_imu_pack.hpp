#pragma once
// Compact raw IMU batches (SD_LOG_IMU_RAW_PACKED), shared by the SD logger and
// the post-flight decoder so both always agree on the layout.
//
// The ICM-45686 high-resolution FIFO gives 20-bit counts that the driver
// scales to floats (count * LSB). Logging the counts instead of the 40-byte
// IMUData struct needs ~17 bytes per sample, which halves the raw-IMU SD rate
// and lets the arena absorb the card's ~170 ms write stalls (the SD spec
// allows up to 250 ms). Packing is exact: a batch is packed only if every
// value is rebuilt bit for bit, otherwise the caller logs SD_LOG_IMU_RAW.
//
// Sample layout (kImuPackedSampleBytes = 17, little endian):
//   bytes 0..14  accel x,y,z then gyro x,y,z as six signed 20-bit counts,
//                packed back to back from bit 0
//   bytes 15..16 timestamp - t0_us, in microseconds
// Temperature is stored once per batch (first sample).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "Application/Modules/sd_logger_types.hpp"

namespace sdlog {

// Same float expressions as the driver's high-resolution FIFO scales.
constexpr float kImuAccelLsb = (32.0f / 524288.0f) * 9.80665f;
constexpr float kImuGyroLsb = (4000.0f / 524288.0f) * (3.14159265f / 180.0f);
constexpr size_t kImuPackedSampleBytes = 17;
constexpr size_t kImuPackedMaxSamples = 255;

inline bool imuToCount(float value, float lsb, int32_t &count) {
  const float q = std::nearbyint(value / lsb);
  if (!(q >= -524288.0f && q <= 524287.0f)) {
    return false;
  }
  count = static_cast<int32_t>(q);
  // Exact only if the decoder's count * lsb gives back the same float.
  return static_cast<float>(count) * lsb == value;
}

inline void imuPut20(uint8_t *p, unsigned index, int32_t value) {
  const uint32_t v = static_cast<uint32_t>(value) & 0xFFFFFu;
  const unsigned bit = index * 20u;
  for (unsigned b = 0; b < 20u; ++b) {
    const unsigned pos = bit + b;
    if (v & (1u << b)) {
      p[pos >> 3] |= static_cast<uint8_t>(1u << (pos & 7u));
    }
  }
}

inline int32_t imuGet20(const uint8_t *p, unsigned index) {
  const unsigned bit = index * 20u;
  uint32_t v = 0;
  for (unsigned b = 0; b < 20u; ++b) {
    const unsigned pos = bit + b;
    if (p[pos >> 3] & (1u << (pos & 7u))) {
      v |= 1u << b;
    }
  }
  if (v & 0x80000u) {
    v |= 0xFFF00000u;
  }
  return static_cast<int32_t>(v);
}

inline float imuTemperatureFromRaw(int16_t raw) {
  return (raw / 128.0f) + 25.0f + 273.15f;  // same as the driver
}

// Writes header + samples to out (capacity: sizeof header + n * 17 bytes).
// Returns the payload length, or 0 if the batch cannot be packed exactly.
inline size_t packImuBatch(uint8_t sensor_index,
                           const Drivers::InvIMU::IMUData *samples, size_t n,
                           uint8_t *out) {
  if (n == 0 || n > kImuPackedMaxSamples) {
    return 0;
  }
  SdLogImuPackedBatchHeader h{};
  h.sensor_index = sensor_index;
  h.sample_count = static_cast<uint8_t>(n);
  h.temperature_raw = static_cast<int16_t>(
      std::lround((samples[0].temperature - (25.0f + 273.15f)) * 128.0f));
  h.accel_lsb = kImuAccelLsb;
  h.gyro_lsb = kImuGyroLsb;
  h.t0_us = samples[0].timestamp_us;
  std::memcpy(out, &h, sizeof(h));

  uint8_t *p = out + sizeof(h);
  for (size_t i = 0; i < n; ++i, p += kImuPackedSampleBytes) {
    const Drivers::InvIMU::IMUData &s = samples[i];
    if (s.timestamp_us < h.t0_us || s.timestamp_us - h.t0_us > 0xFFFFu) {
      return 0;
    }
    const float values[6] = {s.accel_x, s.accel_y, s.accel_z,
                             s.gyro_x,  s.gyro_y,  s.gyro_z};
    std::memset(p, 0, kImuPackedSampleBytes);
    for (unsigned k = 0; k < 6; ++k) {
      int32_t count = 0;
      if (!imuToCount(values[k], k < 3 ? kImuAccelLsb : kImuGyroLsb, count)) {
        return 0;
      }
      imuPut20(p, k, count);
    }
    const uint16_t dt = static_cast<uint16_t>(s.timestamp_us - h.t0_us);
    p[15] = static_cast<uint8_t>(dt & 0xFFu);
    p[16] = static_cast<uint8_t>(dt >> 8);
  }
  return sizeof(h) + n * kImuPackedSampleBytes;
}

// Rebuilds sample i of a packed batch. payload points at the header.
inline Drivers::InvIMU::IMUData unpackImuSample(const uint8_t *payload,
                                               size_t i) {
  SdLogImuPackedBatchHeader h;
  std::memcpy(&h, payload, sizeof(h));
  const uint8_t *p =
      payload + sizeof(h) + i * kImuPackedSampleBytes;
  Drivers::InvIMU::IMUData d{};
  d.accel_x = static_cast<float>(imuGet20(p, 0)) * h.accel_lsb;
  d.accel_y = static_cast<float>(imuGet20(p, 1)) * h.accel_lsb;
  d.accel_z = static_cast<float>(imuGet20(p, 2)) * h.accel_lsb;
  d.gyro_x = static_cast<float>(imuGet20(p, 3)) * h.gyro_lsb;
  d.gyro_y = static_cast<float>(imuGet20(p, 4)) * h.gyro_lsb;
  d.gyro_z = static_cast<float>(imuGet20(p, 5)) * h.gyro_lsb;
  d.temperature = imuTemperatureFromRaw(h.temperature_raw);
  d.timestamp_us =
      h.t0_us + (static_cast<uint64_t>(p[15]) | (static_cast<uint64_t>(p[16]) << 8));
  return d;
}

}  // namespace sdlog
