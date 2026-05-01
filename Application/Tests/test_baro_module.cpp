#include <gtest/gtest.h>

#include <cmath>

#include "Application/Modules/baro_module.hpp"
#include "Drivers/BMP390/Impl/BMP390_mock.h"
#include "Drivers/STM32HAL/Simulations/stm32sim_ticks.hpp"

using Drivers::BMP390::BaroData;
using Drivers::BMP390::BMP390_Interface;
using Drivers::BMP390::BMP390_Mock;
using Drivers::BMP390::BMP390_STATUS_NOT_READY;
using Drivers::BMP390::IIRFilter;
using Drivers::BMP390::OsrPressure;
using Drivers::BMP390::OsrTemp;

namespace {
uint64_t g_last_trigger_us = 0;

void recordTrigger(uint64_t trigger_us) {
  g_last_trigger_us = trigger_us;
}
} // namespace

class BaroModuleTest : public ::testing::Test {
protected:
  BMP390_Mock b0;
  BMP390_Mock b1;
  BMP390_Mock b2;
  BMP390_Mock b3;

  RingBuffer<BaroData, 100> rb0;
  RingBuffer<BaroData, 100> rb1;
  RingBuffer<BaroData, 100> rb2;
  RingBuffer<BaroData, 100> rb3;

  BMP390_Interface *drivers[4] = {&b0, &b1, &b2, &b3};
  RingBuffer<BaroData, 100> *buffers[4] = {&rb0, &rb1, &rb2, &rb3};
  BaroModule module{drivers, buffers};

  void SetUp() override {
    stm32sim_ticks_init();
    app_timebase_init();
    stm32sim_ticks_advance(1);
    g_last_trigger_us = 0;
    b0.inject_data = {101325.0f, 20.0f, 0};
    b1.inject_data = {101326.0f, 20.5f, 0};
    b2.inject_data = {101327.0f, 21.0f, 0};
    b3.inject_data = {101328.0f, 21.5f, 0};
    module.setTriggerCallback(recordTrigger);
  }

  void TearDown() override {
    stm32sim_ticks_deinit();
  }
};

TEST_F(BaroModuleTest, InitConfiguresAllBarometers) {
  ASSERT_TRUE(module.init());

  EXPECT_EQ(b0.last_osr_p, OsrPressure::x4);
  EXPECT_EQ(b0.last_osr_t, OsrTemp::x1);
  EXPECT_EQ(b0.last_filter, IIRFilter::OFF);
  EXPECT_EQ(b3.last_osr_p, OsrPressure::x4);
}

TEST_F(BaroModuleTest, CommandModeTriggersThenPublishesAllFourSamples) {
  ASSERT_TRUE(module.init());

  module.update(1);
  EXPECT_GT(g_last_trigger_us, 0u);
  EXPECT_EQ(b0.call_trigger, 1);
  EXPECT_EQ(module.takeProducedCount(), 0u);

  stm32sim_ticks_advance(1);
  module.update(2);

  EXPECT_EQ(module.takeProducedCount(), 4u);
  EXPECT_EQ(rb0.size(), 1u);
  EXPECT_EQ(rb1.size(), 1u);
  EXPECT_EQ(rb2.size(), 1u);
  EXPECT_EQ(rb3.size(), 1u);

  BaroData sample{};
  ASSERT_TRUE(rb2.pop(sample));
  EXPECT_FLOAT_EQ(sample.pressure_pa, 101327.0f);
  EXPECT_EQ(sample.timestamp_us, g_last_trigger_us);
}

TEST_F(BaroModuleTest, TimeoutPublishesHardFailSentinelForMissingSensor) {
  ASSERT_TRUE(module.init());
  b1.not_ready_cycles = 100;

  module.update(1);
  stm32sim_ticks_advance(60);
  module.update(61);

  // 3 real samples from b0/b2/b3 + 0 from timed-out b1 (no NaN pushed)
  EXPECT_EQ(module.takeProducedCount(), 3u);
  EXPECT_NE(module.sensorStatusFlags(1) & BMP390_STATUS_NOT_READY, 0u);
  EXPECT_EQ(module.sensorDropCount(1), 1u);

  // Timed-out sensor's ring buffer must be empty (no NaN sentinel)
  BaroData sample{};
  EXPECT_FALSE(rb1.pop(sample));

  // Other sensors produced valid samples
  EXPECT_EQ(rb0.size(), 1u);
  EXPECT_EQ(rb2.size(), 1u);
  EXPECT_EQ(rb3.size(), 1u);
}

TEST_F(BaroModuleTest, InitFailedSensorPublishesHardFailSentinelPerCycle) {
  b1.ping_returns = false;
  ASSERT_TRUE(module.init());

  module.update(1);

  // Uninitialised sensor should not push anything to the ring buffer
  EXPECT_EQ(module.takeProducedCount(), 0u);
  BaroData invalid{};
  EXPECT_FALSE(rb1.pop(invalid));
  EXPECT_EQ(module.sensorDropCount(1), 1u);

  stm32sim_ticks_advance(1);
  module.update(2);
  EXPECT_EQ(module.takeProducedCount(), 3u);
  EXPECT_EQ(rb0.size(), 1u);
  EXPECT_EQ(rb2.size(), 1u);
  EXPECT_EQ(rb3.size(), 1u);
}
