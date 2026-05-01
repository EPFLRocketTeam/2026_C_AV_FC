#include <gtest/gtest.h>

#include "Application/Modules/imu_modlue.hpp"
#include "Drivers/InvIMU/Impl/InvIMU_mock.h"

using Drivers::InvIMU::IMUData;
using Drivers::InvIMU::IMU_STATUS_FIFO_OVERFLOW;
using Drivers::InvIMU::AccelRange;
using Drivers::InvIMU::GyroRange;
using Drivers::InvIMU::ODR;
using Drivers::InvIMU::InvIMU_Mock;
using Drivers::InvIMU::InvIMU_Interface;

class ImuModuleTest : public ::testing::Test {
protected:
  InvIMU_Mock imu0;
  InvIMU_Mock imu1;
  InvIMU_Mock imu2;

  RingBuffer<IMUData, 100> rb0;
  RingBuffer<IMUData, 100> rb1;
  RingBuffer<IMUData, 100> rb2;

  InvIMU_Interface* drivers[3] = {&imu0, &imu1, &imu2};
  RingBuffer<IMUData, 100>* buffers[3] = {&rb0, &rb1, &rb2};

  ImuModule module{drivers, buffers};

  static IMUData makeSample(uint64_t ts) {
    IMUData d{};
    d.timestamp_us = ts;
    d.accel_x = 1.0f;
    d.accel_y = 2.0f;
    d.accel_z = 3.0f;
    d.gyro_x = 4.0f;
    d.gyro_y = 5.0f;
    d.gyro_z = 6.0f;
    d.temperature = 298.15f;
    return d;
  }

  void setAllWatermarks(size_t wm) {
    imu0.setFifoWatermark(wm);
    imu1.setFifoWatermark(wm);
    imu2.setFifoWatermark(wm);
  }
};

TEST_F(ImuModuleTest, InitConfiguresAllImusToTargetProfile) {
  ASSERT_TRUE(module.init());

  EXPECT_EQ(imu0.getAccelRange(), AccelRange::_32G);
  EXPECT_EQ(imu0.getGyroRange(), GyroRange::_4000DPS);
  EXPECT_EQ(imu0.getOdr(), ODR::_6_4kHz);
  EXPECT_TRUE(imu0.isFifoConfigured());
  EXPECT_TRUE(imu0.isFsyncEnabled());

  EXPECT_EQ(imu1.getAccelRange(), AccelRange::_32G);
  EXPECT_EQ(imu1.getGyroRange(), GyroRange::_4000DPS);
  EXPECT_EQ(imu1.getOdr(), ODR::_6_4kHz);
  EXPECT_TRUE(imu1.isFifoConfigured());
  EXPECT_TRUE(imu1.isFsyncEnabled());

  EXPECT_EQ(imu2.getAccelRange(), AccelRange::_32G);
  EXPECT_EQ(imu2.getGyroRange(), GyroRange::_4000DPS);
  EXPECT_EQ(imu2.getOdr(), ODR::_6_4kHz);
  EXPECT_TRUE(imu2.isFifoConfigured());
  EXPECT_TRUE(imu2.isFsyncEnabled());
}

TEST_F(ImuModuleTest, UpdateDrainsAllThreeImusAndReportsProducedCount) {
  ASSERT_TRUE(module.init());
  setAllWatermarks(1);

  imu0.addMockSample(makeSample(100));
  imu1.addMockSample(makeSample(110));
  imu2.addMockSample(makeSample(120));

  module.update(1000);

  EXPECT_EQ(module.takeProducedCount(), 3u);
  EXPECT_EQ(rb0.size(), 1u);
  EXPECT_EQ(rb1.size(), 1u);
  EXPECT_EQ(rb2.size(), 1u);

  EXPECT_TRUE(module.sensorHealthy(0));
  EXPECT_TRUE(module.sensorHealthy(1));
  EXPECT_TRUE(module.sensorHealthy(2));
}

TEST_F(ImuModuleTest, ReportsDriverOverflowThroughModuleHealthAPI) {
  ASSERT_TRUE(module.init());
  setAllWatermarks(1);

  imu1.setMaxFifoSize(2);
  imu0.addMockSample(makeSample(100));
  imu2.addMockSample(makeSample(120));
  imu1.addMockSample(makeSample(200));
  imu1.addMockSample(makeSample(201));
  imu1.addMockSample(makeSample(202));
  imu1.addMockSample(makeSample(203));

  module.update(2000);

  EXPECT_GT(module.sensorDropCount(1), 0u);
  EXPECT_NE(module.sensorStatusFlags(1) & IMU_STATUS_FIFO_OVERFLOW, 0u);
}

TEST_F(ImuModuleTest, AlignmentMismatchCounterIncrementsWhenSecondaryLags) {
  ASSERT_TRUE(module.init());
  setAllWatermarks(1);

  // Master (imu0) sample is newer than imu1 sample.
  imu0.addMockSample(makeSample(1000));
  imu1.addMockSample(makeSample(900));
  imu2.addMockSample(makeSample(1005));

  module.update(3000);

  EXPECT_EQ(module.sensorAlignmentMismatches(1), 1u);
}
