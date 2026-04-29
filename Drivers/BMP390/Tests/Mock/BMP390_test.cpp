#include <gtest/gtest.h>
#include "../../Impl/BMP390_mock.h"

using namespace Drivers::BMP390;

class BMP390MockTest : public ::testing::Test {
protected:
    BMP390_Mock baro;

    void SetUp() override {
        baro.inject_data = { 101325.0f, 25.0f, 1000ULL };
    }
};

// ==========================================================
// INIT
// ==========================================================

TEST_F(BMP390MockTest, InitSuccess) {
    EXPECT_TRUE(baro.init());
    EXPECT_EQ(baro.call_init, 1);
}

TEST_F(BMP390MockTest, InitFailure) {
    baro.init_returns = false;
    EXPECT_FALSE(baro.init());
    EXPECT_EQ(baro.call_init, 1);
}

// ==========================================================
// PING
// ==========================================================

TEST_F(BMP390MockTest, PingSuccess) {
    EXPECT_TRUE(baro.ping());
    EXPECT_EQ(baro.call_ping, 1);
}

TEST_F(BMP390MockTest, PingFailure) {
    baro.ping_returns = false;
    EXPECT_FALSE(baro.ping());
}

// ==========================================================
// CONFIGURE
// ==========================================================

TEST_F(BMP390MockTest, ConfigureArgsCaptured) {
    baro.configure(OsrPressure::x8, OsrTemp::x4, IIRFilter::c16);
    EXPECT_EQ(baro.call_configure, 1);
    EXPECT_EQ(baro.last_osr_p,  OsrPressure::x8);
    EXPECT_EQ(baro.last_osr_t,  OsrTemp::x4);
    EXPECT_EQ(baro.last_filter, IIRFilter::c16);
}

TEST_F(BMP390MockTest, ConfigureMultipleTimes) {
    baro.configure(OsrPressure::x1,  OsrTemp::x1,  IIRFilter::OFF);
    baro.configure(OsrPressure::x32, OsrTemp::x32, IIRFilter::c64);
    EXPECT_EQ(baro.call_configure, 2);
    EXPECT_EQ(baro.last_osr_p,  OsrPressure::x32);
    EXPECT_EQ(baro.last_filter, IIRFilter::c64);
}

// ==========================================================
// N/N+1 TRIGGER → GETFRAME PATTERN
// ==========================================================

TEST_F(BMP390MockTest, TriggerThenGetFrameImmediate) {
    baro.inject_data = { 101325.0f, 25.0f, 5000ULL };

    baro.triggerMeasurement();
    EXPECT_EQ(baro.call_trigger, 1);
    EXPECT_TRUE(baro.isPending());

    BaroData d{};
    EXPECT_TRUE(baro.getFrame(d));
    EXPECT_NEAR(d.pressure_pa,   101325.0f, 0.01f);
    EXPECT_NEAR(d.temperature_c, 25.0f,     0.01f);
    EXPECT_EQ(d.timestamp_us, 5000ULL);
    EXPECT_FALSE(baro.isPending());
}

TEST_F(BMP390MockTest, GetFrameWithoutTriggerReturnsFalse) {
    BaroData d{};
    EXPECT_FALSE(baro.getFrame(d));
    EXPECT_EQ(baro.call_getFrame, 1);
    EXPECT_FALSE(baro.isPending());
}

TEST_F(BMP390MockTest, GetFrameClearsPending) {
    baro.triggerMeasurement();
    BaroData d{};
    baro.getFrame(d);
    EXPECT_FALSE(baro.getFrame(d));  // second call without re-trigger
}

// ==========================================================
// DRDY NOT-READY DELAY
// ==========================================================

TEST_F(BMP390MockTest, NotReadyOneCycle) {
    baro.not_ready_cycles = 1;
    baro.triggerMeasurement();

    BaroData d{};
    EXPECT_FALSE(baro.getFrame(d));  // DRDY not ready
    EXPECT_TRUE(baro.getFrame(d));   // now ready
    EXPECT_NEAR(d.pressure_pa, 101325.0f, 0.01f);
}

TEST_F(BMP390MockTest, NotReadyThreeCycles) {
    baro.not_ready_cycles = 3;
    baro.triggerMeasurement();

    BaroData d{};
    int false_count = 0;
    bool result = false;
    for (int i = 0; i < 5; ++i) {
        result = baro.getFrame(d);
        if (!result) ++false_count;
        else break;
    }
    EXPECT_EQ(false_count, 3);
    EXPECT_TRUE(result);
}

// ==========================================================
// FAILURE MODES
// ==========================================================

TEST_F(BMP390MockTest, GetFrameReadError) {
    baro.getFrame_returns = false;
    baro.triggerMeasurement();

    BaroData d{};
    EXPECT_FALSE(baro.getFrame(d));
    EXPECT_NE(baro.getStatus() & BMP390_STATUS_SPI_ERROR, 0u);
    EXPECT_FALSE(baro.isPending());
}

// ==========================================================
// STATUS FLAGS
// ==========================================================

TEST_F(BMP390MockTest, StatusOkByDefault) {
    EXPECT_EQ(baro.getStatus(), static_cast<uint32_t>(BMP390_STATUS_OK));
}

TEST_F(BMP390MockTest, StatusInjectWhoamiMismatch) {
    baro.inject_status = BMP390_STATUS_WHOAMI_MISMATCH;
    EXPECT_NE(baro.getStatus() & BMP390_STATUS_WHOAMI_MISMATCH, 0u);
}

TEST_F(BMP390MockTest, StatusInjectCalibError) {
    baro.inject_status = BMP390_STATUS_CALIB_ERROR;
    EXPECT_NE(baro.getStatus() & BMP390_STATUS_CALIB_ERROR, 0u);
}

// ==========================================================
// MULTIPLE CONSECUTIVE CYCLES
// ==========================================================

TEST_F(BMP390MockTest, MultipleCycles) {
    const float pressures[] = { 101325.0f, 100800.0f, 99500.0f };
    const float temps[]     = { 22.1f,     22.3f,     22.5f    };
    constexpr int N = 3;

    for (int i = 0; i < N; ++i) {
        baro.inject_data = { pressures[i], temps[i], static_cast<uint64_t>(i * 1000) };
        baro.triggerMeasurement();
        BaroData d{};
        EXPECT_TRUE(baro.getFrame(d));
        EXPECT_NEAR(d.pressure_pa,   pressures[i], 0.01f);
        EXPECT_NEAR(d.temperature_c, temps[i],     0.01f);
    }

    EXPECT_EQ(baro.call_trigger,  N);
    EXPECT_EQ(baro.call_getFrame, N);
}

// ==========================================================
// RESET COUNTERS
// ==========================================================

TEST_F(BMP390MockTest, ResetCounters) {
    baro.init();
    baro.ping();
    baro.configure(OsrPressure::x4, OsrTemp::x1, IIRFilter::OFF);
    baro.triggerMeasurement();
    BaroData d{};
    baro.getFrame(d);

    baro.resetCounters();
    EXPECT_EQ(baro.call_init,      0);
    EXPECT_EQ(baro.call_ping,      0);
    EXPECT_EQ(baro.call_configure, 0);
    EXPECT_EQ(baro.call_trigger,   0);
    EXPECT_EQ(baro.call_getFrame,  0);
    EXPECT_FALSE(baro.isPending());
}

// ==========================================================
// POLYMORPHIC USAGE
// ==========================================================

TEST_F(BMP390MockTest, PolymorphicUsage) {
    baro.inject_data = { 101000.0f, 23.5f, 9999ULL };

    BMP390_Interface* iface = &baro;

    EXPECT_TRUE(iface->init());
    iface->configure(OsrPressure::x4, OsrTemp::x1, IIRFilter::c4);
    iface->triggerMeasurement();

    BaroData d{};
    EXPECT_TRUE(iface->getFrame(d));
    EXPECT_NEAR(d.pressure_pa, 101000.0f, 0.01f);
    EXPECT_EQ(iface->getStatus(), static_cast<uint32_t>(BMP390_STATUS_OK));
}
