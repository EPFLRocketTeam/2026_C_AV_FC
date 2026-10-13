#include <gtest/gtest.h>
#include "Application/Data/ring_buffer.hpp"
#include "Application/Kalman/imu_group_alignment.hpp"
#include "Drivers/InvIMU/InvIMU.h"

using Drivers::InvIMU::IMUData;
using Ring = RingBuffer<IMUData, 128>;

TEST(ImuAlignment, RechecksInternalGapsOnEveryGroup) {
  Ring a, b;
  Ring *rings[] = {&a, &b};
  const bool healthy[] = {true, true};
  for (auto ts : {1000u, 1200u, 100000u}) { IMUData s{}; s.timestamp_us = ts; a.append(s); }
  for (auto ts : {1010u, 1210u, 1410u, 100010u}) { IMUData s{}; s.timestamp_us = ts; b.append(s); }
  uint32_t discarded = 0;
  for (unsigned n = 0; n < 3; ++n) {
    ASSERT_TRUE(app::alignImuFronts(rings, healthy, discarded));
    IMUData x{}, y{}; a.pop(x); b.pop(y);
    EXPECT_LE(std::abs(static_cast<int64_t>(x.timestamp_us) -
                       static_cast<int64_t>(y.timestamp_us)), 200);
  }
  EXPECT_EQ(discarded, 1u);
}

TEST(ImuAlignment, RecomputesTargetWhenDiscardJumpsPastIt) {
  Ring a, b;
  Ring *rings[] = {&a, &b};
  const bool healthy[] = {true, true};
  for (auto ts : {1000u, 100000u}) { IMUData s{}; s.timestamp_us = ts; a.append(s); }
  for (auto ts : {2000u, 100050u}) { IMUData s{}; s.timestamp_us = ts; b.append(s); }
  uint32_t discarded = 0;
  ASSERT_TRUE(app::alignImuFronts(rings, healthy, discarded));
  EXPECT_EQ(a.get(0)->timestamp_us, 100000u);
  EXPECT_EQ(b.get(0)->timestamp_us, 100050u);
  EXPECT_EQ(discarded, 2u);
}

TEST(ImuAlignment, EmptyHealthySourceWaitsWithoutEatingOtherRings) {
  Ring a, b;
  Ring *rings[] = {&a, &b};
  const bool healthy[] = {true, true};
  IMUData s{}; s.timestamp_us = 1000; a.append(s);
  uint32_t discarded = 0;
  EXPECT_FALSE(app::alignImuFronts(rings, healthy, discarded));
  EXPECT_EQ(a.size(), 1u);
  EXPECT_EQ(discarded, 0u);
}

TEST(ImuAlignment, UnhealthySourceDoesNotBlockRemainingInput) {
  Ring a, b;
  Ring *rings[] = {&a, &b};
  const bool healthy[] = {true, false};
  IMUData s{}; s.timestamp_us = 1000; a.append(s);
  uint32_t discarded = 0;
  EXPECT_TRUE(app::alignImuFronts(rings, healthy, discarded));
}
