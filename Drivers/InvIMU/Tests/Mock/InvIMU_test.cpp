#include <gtest/gtest.h>
#include "../../Impl/InvIMU_mock.h"

using namespace Drivers::InvIMU;

class InvIMUTest : public ::testing::Test {
protected:
    InvIMU_Mock imu;

    void SetUp() override {
        imu.init();
        imu.configureFifo();
    }
};

// ==========================================================
// CONFIGURATION
// ==========================================================

TEST_F(InvIMUTest, ConfigurationStoredCorrectly) {
    imu.configure(AccelRange::_16G, GyroRange::_1000DPS, ODR::_800Hz);

    EXPECT_EQ(imu.getAccelRange(), AccelRange::_16G);
    EXPECT_EQ(imu.getGyroRange(), GyroRange::_1000DPS);
    EXPECT_EQ(imu.getOdr(), ODR::_800Hz);
}

// ==========================================================
// FIFO ORDER
// ==========================================================

TEST_F(InvIMUTest, FifoPreservesOrder) {
    IMUData d1{}, d2{}, d3{};
    d1.timestamp_us = 1;
    d2.timestamp_us = 2;
    d3.timestamp_us = 3;

    imu.addMockSample(d1);
    imu.addMockSample(d2);
    imu.addMockSample(d3);

    IMUData out{};
    EXPECT_TRUE(imu.getFrame(out));
    EXPECT_EQ(out.timestamp_us, 1u);
    EXPECT_TRUE(imu.getFrame(out));
    EXPECT_EQ(out.timestamp_us, 2u);
    EXPECT_TRUE(imu.getFrame(out));
    EXPECT_EQ(out.timestamp_us, 3u);
}

// ==========================================================
// INTERRUPT → DMA → DATA
// ==========================================================

TEST_F(InvIMUTest, InterruptTriggersDmaFlow) {
    imu.setFifoWatermark(2);

    IMUData d{};
    imu.addMockSample(d);
    imu.addMockSample(d);

    imu.onInterrupt();
    imu.tick();
    imu.onDmaComplete();

    IMUData out{};
    EXPECT_TRUE(imu.getFrame(out));
    EXPECT_TRUE(imu.getFrame(out));
}

// ==========================================================
// BURST HANDLING
// ==========================================================

TEST_F(InvIMUTest, HandlesLargeBurst) {
    constexpr int N = 100;

    for (int i = 0; i < N; ++i) {
        IMUData d{};
        d.timestamp_us = i;
        imu.addMockSample(d);
    }

    IMUData out{};
    int count = 0;
    while (imu.getFrame(out)) {
        EXPECT_EQ(out.timestamp_us, static_cast<uint64_t>(count));
        count++;
    }

    EXPECT_EQ(count, N);
}

// ==========================================================
// TIMESTAMP MONOTONICITY
// ==========================================================

TEST_F(InvIMUTest, TimestampMonotonic) {
    imu.setBaseTimestamp(1'000'000);

    for (int i = 0; i < 10; ++i) {
        IMUData d{};
        imu.addMockSample(d);
    }

    IMUData out{};
    uint64_t last = 0;

    while (imu.getFrame(out)) {
        EXPECT_GT(out.timestamp_us, last);
        last = out.timestamp_us;
    }
}

// ==========================================================
// FAILURE MODES
// ==========================================================

TEST_F(InvIMUTest, SpiErrorBlocksDma) {
    imu.injectSpiError();

    IMUData d{};
    imu.addMockSample(d);

    imu.onInterrupt();
    imu.tick();
    imu.onDmaComplete();

    IMUData out{};
    EXPECT_FALSE(imu.getFrame(out));
}

TEST_F(InvIMUTest, FifoOverflowDetected) {
    imu.setMaxFifoSize(4);

    IMUData d{};
    for (int i = 0; i < 10; ++i) {
        imu.addMockSample(d);
    }

    EXPECT_TRUE(imu.hasFifoOverflow());
}
