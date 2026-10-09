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

inline bool imuToCount(float value, float inv_lsb, float lsb, int32_t &count) {
  const float q = value * inv_lsb;
  if (!(q > -524288.5f && q < 524287.5f)) {
    return false;
  }
  count = static_cast<int32_t>(q + (q >= 0.0f ? 0.5f : -0.5f));
  // Exact only if the decoder's count * lsb gives back the same float.
  return static_cast<float>(count) * lsb == value;
}

// Two 20-bit values per 5 bytes: value 2k in bits 0..19, 2k+1 in bits 20..39
// of bytes 5k..5k+4 (little endian), i.e. one continuous bit stream.
inline void imuPutPair(uint8_t *p, int32_t a, int32_t b) {
  const uint64_t w = (static_cast<uint64_t>(static_cast<uint32_t>(a) & 0xFFFFFu)) |
                     (static_cast<uint64_t>(static_cast<uint32_t>(b) & 0xFFFFFu) << 20);
  for (unsigned i = 0; i < 5; ++i) {
    p[i] = static_cast<uint8_t>(w >> (8u * i));
  }
}

inline int32_t imuSignExtend20(uint32_t v) {
  return static_cast<int32_t>((v & 0x80000u) ? (v | 0xFFF00000u) : v);
}

inline int32_t imuGet20(const uint8_t *p, unsigned index) {
  const uint8_t *q = p + 5u * (index / 2u);
  uint64_t w = 0;
  for (unsigned i = 0; i < 5; ++i) {
    w |= static_cast<uint64_t>(q[i]) << (8u * i);
  }
  return imuSignExtend20(static_cast<uint32_t>(w >> ((index & 1u) ? 20u : 0u)) & 0xFFFFFu);
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
    constexpr float kInvAccel = 1.0f / kImuAccelLsb;
    constexpr float kInvGyro = 1.0f / kImuGyroLsb;
    int32_t c[6];
    if (!imuToCount(s.accel_x, kInvAccel, kImuAccelLsb, c[0]) ||
        !imuToCount(s.accel_y, kInvAccel, kImuAccelLsb, c[1]) ||
        !imuToCount(s.accel_z, kInvAccel, kImuAccelLsb, c[2]) ||
        !imuToCount(s.gyro_x, kInvGyro, kImuGyroLsb, c[3]) ||
        !imuToCount(s.gyro_y, kInvGyro, kImuGyroLsb, c[4]) ||
        !imuToCount(s.gyro_z, kInvGyro, kImuGyroLsb, c[5])) {
      return 0;
    }
    imuPutPair(p, c[0], c[1]);
    imuPutPair(p + 5, c[2], c[3]);
    imuPutPair(p + 10, c[4], c[5]);
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
