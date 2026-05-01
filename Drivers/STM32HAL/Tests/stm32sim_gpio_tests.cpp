
#include <gtest/gtest.h>

#include "Drivers/STM32HAL/Simulations/stm32sim_gpio.hpp"

USING_SIMULATOR_NAMESPACE::gpio;

GPIO_TypeDef sample1 = {
    12,        /* port_uuid */
    "Sample#1" /* port_name */
};
GPIO_TypeDef sample2 = {
    23,        /* port_uuid */
    "Sample#2" /* port_name */
};

TEST(SimulatorGPIOTests, TestRegister) {
    EXPECT_EQ((12 << 16) | 1, SIM_RegisterGPIO(&sample1, 1));
    EXPECT_EQ((12 << 16) | 2, SIM_RegisterGPIO(&sample1, 2));
    EXPECT_EQ((23 << 16) | 5, SIM_RegisterGPIO(&sample2, 5));
}
TEST(SimulatorGPIOTests, TestRegisterFailsWhenDuplicate) {
    EXPECT_EQ((12 << 16) | 22, SIM_RegisterGPIO(&sample1, 22));
    
    bool went_through_error = false;
    try {
        SIM_RegisterGPIO(&sample1, 22);
    } catch (const std::runtime_error &error) {
        went_through_error = true;

        EXPECT_STREQ(
            error.what(), "Can't register twice a GPIO (port=Sample#1, pin=22)");
    }

    EXPECT_EQ(went_through_error, true);
}
TEST(SimulatorGPIOTests, TestUnregisterDoesNothing) {
    SIM_UnregisterGPIO(731);
}
TEST(SimulatorGPIOTests, TestRegisterAndUnregister) {
    uint32_t target = (12 << 16) | 1;
    EXPECT_EQ(target, SIM_RegisterGPIO(&sample1, 1));
    SIM_UnregisterGPIO(target);
    EXPECT_EQ(target, SIM_RegisterGPIO(&sample1, 1));
}

TEST(SimulatorGPIOTests, TestRead) {
    SIM_RegisterGPIO(&sample1, 22);
    EXPECT_EQ(HAL_GPIO_ReadPin(&sample1, 22), GPIO_PIN_RESET);
}
TEST(SimulatorGPIOTests, TestReadOnUnregistered) {
    bool went_through_error = false;
    try {
        HAL_GPIO_ReadPin(&sample1, 22);
    } catch (const std::runtime_error &error) {
        went_through_error = true;

        EXPECT_STREQ(
            error.what(), "Could not read on Unitialized PIN (port=Sample#1, pin=22)");
    }

    EXPECT_EQ(went_through_error, true);
}

TEST(SimulatorGPIOTests, TestWrite) {
    SIM_RegisterGPIO(&sample1, 22);
    HAL_GPIO_WritePin(&sample1, 22, GPIO_PIN_SET);
    EXPECT_EQ(HAL_GPIO_ReadPin(&sample1, 22), GPIO_PIN_SET);
    HAL_GPIO_WritePin(&sample1, 22, GPIO_PIN_SET);
    EXPECT_EQ(HAL_GPIO_ReadPin(&sample1, 22), GPIO_PIN_SET);
    HAL_GPIO_WritePin(&sample1, 22, GPIO_PIN_RESET);
    EXPECT_EQ(HAL_GPIO_ReadPin(&sample1, 22), GPIO_PIN_RESET);
}
TEST(SimulatorGPIOTests, TestWriteOnUnregistered) {
    bool went_through_error = false;
    try {
        HAL_GPIO_WritePin(&sample1, 22, GPIO_PIN_SET);
    } catch (const std::runtime_error &error) {
        went_through_error = true;

        EXPECT_STREQ(
            error.what(), "Could not write on Unitialized PIN (port=Sample#1, pin=22)");
    }

    EXPECT_EQ(went_through_error, true);
}

TEST(SimulatorGPIOTests, TestToggle) {
    SIM_RegisterGPIO(&sample1, 22);
    HAL_GPIO_TogglePin(&sample1, 22);
    EXPECT_EQ(HAL_GPIO_ReadPin(&sample1, 22), GPIO_PIN_SET);
    HAL_GPIO_WritePin(&sample1, 22, GPIO_PIN_SET);
    EXPECT_EQ(HAL_GPIO_ReadPin(&sample1, 22), GPIO_PIN_SET);
    HAL_GPIO_TogglePin(&sample1, 22);
    EXPECT_EQ(HAL_GPIO_ReadPin(&sample1, 22), GPIO_PIN_RESET);
}
TEST(SimulatorGPIOTests, TestToggleOnUnregistered) {
    bool went_through_error = false;
    try {
        HAL_GPIO_TogglePin(&sample1, 22);
    } catch (const std::runtime_error &error) {
        went_through_error = true;

        EXPECT_STREQ(
            error.what(), "Could not toggle on Unitialized PIN (port=Sample#1, pin=22)");
    }

    EXPECT_EQ(went_through_error, true);
}

TEST(SimulatorGPIOTests, TestDefaultCallback) {
    HAL_GPIO_EXTI_Callback(0);
    HAL_GPIO_EXTI_Callback(1);
}
