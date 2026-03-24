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

TEST_F(TickSimTest, DoubleInitThrows)
{
    EXPECT_THROW(
        stm32sim_ticks_init(),
        std::logic_error
    );
}

TEST(TickSimStandaloneTest, DoubleDeinitThrows)
{
    stm32sim_ticks_init();
    stm32sim_ticks_deinit();

    EXPECT_THROW(
        stm32sim_ticks_deinit(),
        std::logic_error
    );
}
