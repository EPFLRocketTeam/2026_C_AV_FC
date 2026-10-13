// Packed raw IMU SD records: exact round trip and fallback cases.
#include <gtest/gtest.h>

#include <cstring>
#include <random>

#include "Application/Modules/sd_imu_pack.hpp"

using Drivers::InvIMU::IMUData;

namespace {

// Same arithmetic as the driver: 20-bit count (int32) times the float LSB.
IMUData driverSample(std::mt19937 &rng, uint64_t ts) {
  std::uniform_int_distribution<int32_t> count(-524288, 524287);
  std::uniform_int_distribution<int32_t> temp(-3000, 6000);
  IMUData d{};
  d.accel_x = count(rng) * sdlog::kImuAccelLsb;
  d.accel_y = count(rng) * sdlog::kImuAccelLsb;
  d.accel_z = count(rng) * sdlog::kImuAccelLsb;
  d.gyro_x = count(rng) * sdlog::kImuGyroLsb;
  d.gyro_y = count(rng) * sdlog::kImuGyroLsb;
  d.gyro_z = count(rng) * sdlog::kImuGyroLsb;
  d.temperature = (temp(rng) / 128.0f) + 25.0f + 273.15f;
  d.timestamp_us = ts;
  return d;
}

bool sameBits(float a, float b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

}  // namespace

TEST(SdImuPack, RoundTripIsBitExact) {
  std::mt19937 rng(12345);
  for (int batch = 0; batch < 2000; ++batch) {
    const size_t n = 1 + batch % 32;
    IMUData in[32];
    uint64_t ts = 1000000000ull + batch * 7000ull;
    for (size_t i = 0; i < n; ++i, ts += (i % 4) ? 156 : 157) {
      in[i] = driverSample(rng, ts);
    }
    uint8_t buf[sizeof(SdLogImuPackedBatchHeader) + 32 * sdlog::kImuPackedSampleBytes];
    const size_t len = sdlog::packImuBatch(2, in, n, buf);
    ASSERT_EQ(len, sizeof(SdLogImuPackedBatchHeader) + n * sdlog::kImuPackedSampleBytes);
    for (size_t i = 0; i < n; ++i) {
      const IMUData out = sdlog::unpackImuSample(buf, i);
      EXPECT_TRUE(sameBits(out.accel_x, in[i].accel_x));
      EXPECT_TRUE(sameBits(out.accel_y, in[i].accel_y));
      EXPECT_TRUE(sameBits(out.accel_z, in[i].accel_z));
      EXPECT_TRUE(sameBits(out.gyro_x, in[i].gyro_x));
      EXPECT_TRUE(sameBits(out.gyro_y, in[i].gyro_y));
      EXPECT_TRUE(sameBits(out.gyro_z, in[i].gyro_z));
      EXPECT_EQ(out.timestamp_us, in[i].timestamp_us);
    }
    // Temperature is stored once per batch: exact for the first sample.
    EXPECT_TRUE(sameBits(sdlog::unpackImuSample(buf, 0).temperature, in[0].temperature));
  }
}

TEST(SdImuPack, RefusesValuesThatAreNotCounts) {
  IMUData d{};
  d.accel_x = 1.2345f;  // not a multiple of the LSB
  d.timestamp_us = 10;
  uint8_t buf[64];
  EXPECT_EQ(sdlog::packImuBatch(0, &d, 1, buf), 0u);
}

TEST(SdImuPack, RefusesOutOfRangeTimeSpan) {
  std::mt19937 rng(1);
  IMUData d[2] = {driverSample(rng, 100), driverSample(rng, 100 + 70000)};
  uint8_t buf[128];
  EXPECT_EQ(sdlog::packImuBatch(0, d, 2, buf), 0u);
}

TEST(SdImuPack, HalvesTheRawRate) {
  // Typical batch on the bench: ~5-6 samples per sensor per loop.
  const size_t n = 6;
  const size_t full = 8 + 4 + n * sizeof(IMUData);  // SdLogHeader + batch header
  const size_t packed = 8 + sizeof(SdLogImuPackedBatchHeader) + n * sdlog::kImuPackedSampleBytes;
  EXPECT_LT(packed * 10, full * 6);  // at least 40 % smaller
}
