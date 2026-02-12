#include <gtest/gtest.h>
#include <thread>
#include <chrono>

#include "Drivers/STM32HAL/Simulations/stm32sim_ticks.hpp"

class TickSimTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        stm32sim_ticks_init();
    }

    void TearDown() override
    {
        stm32sim_ticks_deinit();
    }
};

TEST_F(TickSimTest, TickStartsAtZero)
{
    uint32_t tick = HAL_GetTick();
    EXPECT_EQ(tick, 0u);
}

TEST_F(TickSimTest, TickIncrementsOverTime)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint32_t tick = HAL_GetTick();

    // We don't expect perfect timing because of OS scheduling
    EXPECT_GE(tick, 5u);
}
