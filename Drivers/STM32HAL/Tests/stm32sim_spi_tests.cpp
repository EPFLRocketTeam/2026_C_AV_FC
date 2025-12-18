
#include <vector>
#include <utility>
#include <gtest/gtest.h>

#include "Drivers/STM32HAL/Simulations/stm32sim_spi.hpp"

USING_SIMULATOR_NAMESPACE::gpio;
USING_SIMULATOR_NAMESPACE::spi;

GPIO_TypeDef sample1 = {
    12,        /* port_uuid */
    "Sample#1" /* port_name */
};
GPIO_TypeDef sample2 = {
    23,        /* port_uuid */
    "Sample#2" /* port_name */
};

SPI_HandleTypeDef spi1 = {
    5,      /* hspi_uuid */
    "SPI#1" /* hspi_name */
};
SPI_HandleTypeDef spi2 = {
    21,     /* hspi_uuid */
    "SPI#2" /* hspi_name */
};

#define SPI_d1 &sample1, 12
#define SPI_d2 &sample1, 13
#define SPI_d3 &sample2, 3

uint32_t handle_d1, handle_d2, handle_d3;

std::vector<std::tuple<void*, uint16_t>> rDatas; 
std::vector<std::tuple<void*, std::vector<unsigned char>>> tDatas; 
void onReceive (void *deviceData, unsigned char *pData, uint16_t Size) {
    rDatas.push_back({ deviceData, Size });

    for (int i = 0; i < Size; i ++) {
        pData[i] = i ^ ((int)((long long)deviceData));
    }
}
void onTransmit (void *deviceData, const unsigned char *pData, uint16_t Size) {
    std::vector<unsigned char> chrs;
    for (int i = 0; i < Size; i ++) chrs.push_back(pData[i]);

    tDatas.push_back({ deviceData, chrs });
}
void onTxRx (void *deviceData, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size) {
    onTransmit (deviceData, pTxData, Size);
    onReceive  (deviceData, pRxData, Size);
}

void simpleSetup () {
    SIM_RegisterGPIO(SPI_d1);
    HAL_GPIO_WritePin(SPI_d1, GPIO_PIN_SET);

    handle_d1 = SIM_RegisterSPIDevice(&spi1, SPI_d1, {
        /* Device Data */
        (void*) 12, /* deviceData */

        /* Device Mode */
        SPI_Handler, /* mode */

        GPIO_PIN_RESET, /* activeState */

        /* SPI_Handler mode handlers */
        &onReceive,  /* onReceive */
        &onTransmit, /* onTransmit */
        &onTxRx      /* onRxTx */
    });

    SIM_RegisterGPIO(SPI_d2);
    HAL_GPIO_WritePin(SPI_d2, GPIO_PIN_SET);

    handle_d2 = SIM_RegisterSPIDevice(&spi1, SPI_d2, {
        /* Device Data */
        (void*) 7, /* deviceData */

        /* Device Mode */
        SPI_Handler, /* mode */

        GPIO_PIN_RESET, /* activeState */

        /* SPI_Handler mode handlers */
        &onReceive,  /* onReceive */
        &onTransmit, /* onTransmit */
        &onTxRx      /* onRxTx */
    });

    SIM_RegisterGPIO(SPI_d3);
    HAL_GPIO_WritePin(SPI_d3, GPIO_PIN_SET);

    handle_d3 = SIM_RegisterSPIDevice(&spi2, SPI_d3, {
        /* Device Data */
        (void*) 23, /* deviceData */

        /* Device Mode */
        SPI_Handler, /* mode */

        GPIO_PIN_RESET, /* activeState */

        /* SPI_Handler mode handlers */
        &onReceive,  /* onReceive */
        &onTransmit, /* onTransmit */
        &onTxRx      /* onRxTx */
    });

    EXPECT_EQ(handle_d1, 0);
    EXPECT_EQ(handle_d2, 1);
    EXPECT_EQ(handle_d3, 2);
}

TEST(SimulatorSPITests, TestSimpleSetup) {
    simpleSetup();
}
TEST(SimulatorSPITests, TestRegisterTwiceFails) {
    simpleSetup();

    bool threw_error = false;
    try {
        SIM_RegisterSPIDevice(&spi1, SPI_d2, {});
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI device already exists (bus=SPI#1, ss=(port=Sample#1, pin=13))");
        threw_error = true;
    }

    EXPECT_EQ(threw_error, true);
}
TEST(SimulatorSPITests, TestTooManyDevices) {
    for (int i = 0; i <= 255; i ++) {
        int16_t u = SIM_RegisterSPIDevice(&spi2, &sample1, i, {
            /* Device Data */
            (void*) 23, /* deviceData */

            /* Device Mode */
            SPI_Handler, /* mode */

            GPIO_PIN_RESET, /* activeState */

            /* SPI_Handler mode handlers */
            &onReceive,  /* onReceive */
            &onTransmit, /* onTransmit */
            &onTxRx      /* onRxTx */
        });

        EXPECT_EQ(u, i);
    }
    
    bool threw_error = false;
    try {
        SIM_RegisterSPIDevice(&spi2, &sample2, 1, {});
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "Too many SPI devices, could not create it (bus=SPI#2, ss=(port=Sample#2, pin=1))");
        threw_error = true;
    }

    EXPECT_EQ(threw_error, true);
}
TEST(SimulatorSPITests, TestUnregister) {
    for (int i = 0; i <= 1024; i ++) {
        int16_t u = SIM_RegisterSPIDevice(&spi2, &sample1, i, {
            /* Device Data */
            (void*) 23, /* deviceData */

            /* Device Mode */
            SPI_Handler, /* mode */

            GPIO_PIN_RESET, /* activeState */

            /* SPI_Handler mode handlers */
            &onReceive,  /* onReceive */
            &onTransmit, /* onTransmit */
            NULL         /* onRxTx */
        });

        EXPECT_EQ(u, 0);
        SIM_UnregisterSPIDevice(0);
    }
}

TEST(SimulatorSPITests, TestReceive) {
    simpleSetup();

    unsigned char buffer[8];

    HAL_GPIO_TogglePin(SPI_d1);
    HAL_SPI_Receive(&spi1, buffer, 8, 1);
    for (int i = 0; i < 8; i ++)
        EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    HAL_GPIO_TogglePin(SPI_d1);
    
    HAL_GPIO_TogglePin(SPI_d3);
    HAL_SPI_Receive(&spi2, buffer, 8, 1);
    for (int i = 0; i < 8; i ++)
        EXPECT_EQ(buffer[i], 0xFF & (i ^ 23));
    
    HAL_GPIO_TogglePin(SPI_d1);
    HAL_SPI_Receive(&spi1, buffer, 8, 1);
    for (int i = 0; i < 8; i ++)
        EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    HAL_GPIO_TogglePin(SPI_d1);

    HAL_GPIO_TogglePin(SPI_d2);
    HAL_SPI_Receive(&spi1, buffer, 8, 1);
    for (int i = 0; i < 8; i ++)
        EXPECT_EQ(buffer[i], 0xFF & (i ^ 7));
    HAL_GPIO_TogglePin(SPI_d2);

    EXPECT_EQ(rDatas.size(), 4);
    EXPECT_EQ((long long) std::get<0>(rDatas[0]), 12);
    EXPECT_EQ((long long) std::get<0>(rDatas[1]), 23);
    EXPECT_EQ((long long) std::get<0>(rDatas[2]), 12);
    EXPECT_EQ((long long) std::get<0>(rDatas[3]), 7);
    EXPECT_EQ((long long) std::get<1>(rDatas[0]), 8);
    EXPECT_EQ((long long) std::get<1>(rDatas[1]), 8);
    EXPECT_EQ((long long) std::get<1>(rDatas[2]), 8);
    EXPECT_EQ((long long) std::get<1>(rDatas[3]), 8);
};

TEST(SimulatorSPITests, TestReceiveErrors) {
    simpleSetup();

    unsigned char buffer[8];

    try {
        HAL_SPI_Receive(&spi1, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has no Slave Selects enabled."
        );
    }

    HAL_GPIO_TogglePin(SPI_d1);
    HAL_GPIO_TogglePin(SPI_d2);
    try {
        HAL_SPI_Receive(&spi1, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has too many Slave Selects enabled (>= 2)."
        );
    }
    HAL_GPIO_TogglePin(SPI_d1);
    HAL_GPIO_TogglePin(SPI_d2);

    handle_d1 = SIM_RegisterSPIDevice(&spi1, &sample1, 15, {
        /* Device Data */
        (void*) 17, /* deviceData */

        /* Device Mode */
        SPI_Handler, /* mode */

        GPIO_PIN_RESET, /* activeState */

        /* SPI_Handler mode handlers */
        &onReceive,  /* onReceive */
        &onTransmit, /* onTransmit */
        &onTxRx      /* onRxTx */
    });
    try {
        HAL_SPI_Receive(&spi1, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has an unregistered GPIO: Could not read on Unitialized PIN (port=Sample#1, pin=15)"
        );
    }
    
    EXPECT_EQ(rDatas.size(), 0);
}

TEST(SimulatorSPITests, TestTransmit) {
    simpleSetup();

    unsigned char buffer[8];

    HAL_GPIO_TogglePin(SPI_d1);
    for (int i = 0; i < 8; i ++)
        buffer[i] = i ^ 12;
    HAL_SPI_Transmit(&spi1, buffer, 8, 1);
    HAL_GPIO_TogglePin(SPI_d1);
    
    HAL_GPIO_TogglePin(SPI_d3);
    for (int i = 0; i < 8; i ++)
        buffer[i] = i ^ 23;
    HAL_SPI_Transmit(&spi2, buffer, 8, 1);
    
    HAL_GPIO_TogglePin(SPI_d1);
    for (int i = 0; i < 8; i ++)
        buffer[i] = i ^ 7;
    HAL_SPI_Transmit(&spi1, buffer, 8, 1);
    HAL_GPIO_TogglePin(SPI_d1);

    HAL_GPIO_TogglePin(SPI_d2);
    for (int i = 0; i < 8; i ++)
        buffer[i] = i ^ 11;
    HAL_SPI_Transmit(&spi1, buffer, 8, 1);
    HAL_GPIO_TogglePin(SPI_d2);

    EXPECT_EQ(tDatas.size(), 4);
    EXPECT_EQ((long long) std::get<0>(tDatas[0]), 12);
    EXPECT_EQ((long long) std::get<0>(tDatas[1]), 23);
    EXPECT_EQ((long long) std::get<0>(tDatas[2]), 12);
    EXPECT_EQ((long long) std::get<0>(tDatas[3]), 7);
    EXPECT_EQ(std::get<1>(tDatas[0]).size(), 8);
    EXPECT_EQ(std::get<1>(tDatas[1]).size(), 8);
    EXPECT_EQ(std::get<1>(tDatas[2]).size(), 8);
    EXPECT_EQ(std::get<1>(tDatas[3]).size(), 8);
    for (int i = 0; i < 8; i ++) {
        EXPECT_EQ(std::get<1>(tDatas[0]).at(i), i ^ 12);
        EXPECT_EQ(std::get<1>(tDatas[1]).at(i), i ^ 23);
        EXPECT_EQ(std::get<1>(tDatas[2]).at(i), i ^ 7);
        EXPECT_EQ(std::get<1>(tDatas[3]).at(i), i ^ 11);
    }
};

TEST(SimulatorSPITests, TestTransmitErrors) {
    simpleSetup();

    unsigned char buffer[8];

    try {
        HAL_SPI_Transmit(&spi1, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has no Slave Selects enabled."
        );
    }

    HAL_GPIO_TogglePin(SPI_d1);
    HAL_GPIO_TogglePin(SPI_d2);
    try {
        HAL_SPI_Transmit(&spi1, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has too many Slave Selects enabled (>= 2)."
        );
    }
    HAL_GPIO_TogglePin(SPI_d1);
    HAL_GPIO_TogglePin(SPI_d2);

    handle_d1 = SIM_RegisterSPIDevice(&spi1, &sample1, 15, {
        /* Device Data */
        (void*) 17, /* deviceData */

        /* Device Mode */
        SPI_Handler, /* mode */

        GPIO_PIN_RESET, /* activeState */

        /* SPI_Handler mode handlers */
        &onReceive,  /* onReceive */
        &onTransmit, /* onTransmit */
        &onTxRx      /* onRxTx */
    });
    try {
        HAL_SPI_Transmit(&spi1, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has an unregistered GPIO: Could not read on Unitialized PIN (port=Sample#1, pin=15)"
        );
    }
    
    EXPECT_EQ(rDatas.size(), 0);
}

TEST(SimulatorSPITests, TestTxRx) {
    simpleSetup();

    unsigned char buffer[8];

    HAL_GPIO_TogglePin(SPI_d1);
    for (int i = 0; i < 8; i ++)
        buffer[i] = i ^ 12;
    HAL_SPI_TransmitReceive(&spi1, buffer, buffer, 8, 1);
    for (int i = 0; i < 8; i ++)
        EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    HAL_GPIO_TogglePin(SPI_d1);
    
    HAL_GPIO_TogglePin(SPI_d3);
    for (int i = 0; i < 8; i ++)
        buffer[i] = i ^ 23;
    HAL_SPI_TransmitReceive(&spi2, buffer, buffer, 8, 1);
    for (int i = 0; i < 8; i ++)
        EXPECT_EQ(buffer[i], 0xFF & (i ^ 23));
    
    HAL_GPIO_TogglePin(SPI_d1);
    for (int i = 0; i < 8; i ++)
        buffer[i] = i ^ 7;
    HAL_SPI_TransmitReceive(&spi1, buffer, buffer, 8, 1);
    for (int i = 0; i < 8; i ++)
        EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    HAL_GPIO_TogglePin(SPI_d1);

    HAL_GPIO_TogglePin(SPI_d2);
    for (int i = 0; i < 8; i ++)
        buffer[i] = i ^ 11;
    HAL_SPI_TransmitReceive(&spi1, buffer, buffer, 8, 1);
    for (int i = 0; i < 8; i ++)
        EXPECT_EQ(buffer[i], 0xFF & (i ^ 7));
    HAL_GPIO_TogglePin(SPI_d2);

    EXPECT_EQ(tDatas.size(), 4);
    EXPECT_EQ((long long) std::get<0>(tDatas[0]), 12);
    EXPECT_EQ((long long) std::get<0>(tDatas[1]), 23);
    EXPECT_EQ((long long) std::get<0>(tDatas[2]), 12);
    EXPECT_EQ((long long) std::get<0>(tDatas[3]), 7);
    EXPECT_EQ(std::get<1>(tDatas[0]).size(), 8);
    EXPECT_EQ(std::get<1>(tDatas[1]).size(), 8);
    EXPECT_EQ(std::get<1>(tDatas[2]).size(), 8);
    EXPECT_EQ(std::get<1>(tDatas[3]).size(), 8);
    for (int i = 0; i < 8; i ++) {
        EXPECT_EQ(std::get<1>(tDatas[0]).at(i), i ^ 12);
        EXPECT_EQ(std::get<1>(tDatas[1]).at(i), i ^ 23);
        EXPECT_EQ(std::get<1>(tDatas[2]).at(i), i ^ 7);
        EXPECT_EQ(std::get<1>(tDatas[3]).at(i), i ^ 11);
    }

    EXPECT_EQ(rDatas.size(), 4);
    EXPECT_EQ((long long) std::get<0>(rDatas[0]), 12);
    EXPECT_EQ((long long) std::get<0>(rDatas[1]), 23);
    EXPECT_EQ((long long) std::get<0>(rDatas[2]), 12);
    EXPECT_EQ((long long) std::get<0>(rDatas[3]), 7);
    EXPECT_EQ((long long) std::get<1>(rDatas[0]), 8);
    EXPECT_EQ((long long) std::get<1>(rDatas[1]), 8);
    EXPECT_EQ((long long) std::get<1>(rDatas[2]), 8);
    EXPECT_EQ((long long) std::get<1>(rDatas[3]), 8);
};

TEST(SimulatorSPITests, TestTxRxErrors) {
    simpleSetup();

    unsigned char buffer[8];

    try {
        HAL_SPI_TransmitReceive(&spi1, buffer, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has no Slave Selects enabled."
        );
    }

    HAL_GPIO_TogglePin(SPI_d1);
    HAL_GPIO_TogglePin(SPI_d2);
    try {
        HAL_SPI_TransmitReceive(&spi1, buffer, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has too many Slave Selects enabled (>= 2)."
        );
    }
    HAL_GPIO_TogglePin(SPI_d1);
    HAL_GPIO_TogglePin(SPI_d2);

    handle_d1 = SIM_RegisterSPIDevice(&spi1, &sample1, 15, {
        /* Device Data */
        (void*) 17, /* deviceData */

        /* Device Mode */
        SPI_Handler, /* mode */

        GPIO_PIN_RESET, /* activeState */

        /* SPI_Handler mode handlers */
        &onReceive,  /* onReceive */
        &onTransmit, /* onTransmit */
        &onTxRx      /* onRxTx */
    });
    try {
        HAL_SPI_TransmitReceive(&spi1, buffer, buffer, 8, 1);
        for (int i = 0; i < 8; i ++)
            EXPECT_EQ(buffer[i], 0xFF & (i ^ 12));
    } catch (const std::runtime_error &error) {
        EXPECT_STREQ(
            error.what(),
            "SPI Handle (SPI#1) has an unregistered GPIO: Could not read on Unitialized PIN (port=Sample#1, pin=15)"
        );
    }
    
    EXPECT_EQ(rDatas.size(), 0);
}
