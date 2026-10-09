#include <gtest/gtest.h>
#include "Application/Modules/imu_modlue.hpp"

namespace {
class BurstImu final : public Drivers::InvIMU::InvIMU_Interface {
 public:
  unsigned remaining = 32;
  unsigned sample = 0;
  unsigned ticks = 0;
  unsigned timestamp_offset = 0;
  bool init() override { return true; }
  bool ping() override { return true; }
  void configure(AccelRange, GyroRange, ODR) override {}
  void configureFifo() override {}
  void enableFsync() override {}
  void onInterrupt(uint64_t = 0) override {}
  void onDmaComplete() override {}
  void tick() override { ++ticks; }
  bool getFrame(IMUData& out) override {
    if (!remaining) return false;
    --remaining;
    out = {};
    out.accel_x = 9.80665f;
    out.timestamp_us = 1000000 + sample++ * 156 - timestamp_offset;
    return true;
  }
};

struct BenchImuModuleTest : testing::Test {
  BurstImu devices[4];
  InvIMU_Interface* drivers[4] = {&devices[0], &devices[1], &devices[2], &devices[3]};
  AppImuRingBuffer rings[4];
  AppImuRingBuffer* buffers[4] = {&rings[0], &rings[1], &rings[2], &rings[3]};
  ImuModule<4> module{drivers, buffers};
};

TEST_F(BenchImuModuleTest, PublishesEverySampleAndAllFourHealthySources) {
  ASSERT_TRUE(module.init());
  module.update(1);
  EXPECT_EQ(module.takeProducedCount(), 128u);
  for (unsigned i = 0; i < 4; ++i) {
    EXPECT_EQ(rings[i].size(), 32u);
    EXPECT_TRUE(module.sensorHealthy(i));
    EXPECT_EQ(devices[i].ticks, 1u);
    const auto stats = module.takeAcqStats(i);
    EXPECT_EQ(stats.frames, 32u);
    EXPECT_EQ(stats.gaps, 0u);
    EXPECT_EQ(stats.ring_overwrites, 0u);
  }
}

TEST_F(BenchImuModuleTest, ReportsSoftwareOverwriteSeparatelyFromHardwareLoss) {
  ASSERT_TRUE(module.init());
  IMUData older{};
  for (unsigned j = 0; j < 100; ++j) rings[0].append(older);
  module.update(1);
  EXPECT_EQ(rings[0].size(), 128u);
  const auto stats = module.takeAcqStats(0);
  EXPECT_EQ(stats.frames, 32u);
  EXPECT_EQ(stats.gaps, 0u);
  EXPECT_EQ(stats.ring_overwrites, 4u);
  EXPECT_EQ(module.takeAcqStats(0).ring_overwrites, 0u);
}

TEST_F(BenchImuModuleTest, DoesNotRepollLaggingSecondariesBeforeConsumption) {
  ASSERT_TRUE(module.init());
  devices[1].timestamp_offset = 1000;
  module.update(1);
  EXPECT_EQ(module.takeProducedCount(), 128u);
  for (auto& device : devices) EXPECT_EQ(device.ticks, 1u);
  EXPECT_EQ(module.sensorAlignmentMismatches(1), 1u);
}
}  // namespace
