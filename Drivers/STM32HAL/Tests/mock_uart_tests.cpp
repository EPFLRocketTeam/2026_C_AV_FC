#include <gtest/gtest.h>
#include "Drivers/STM32HAL/Interfaces/mock_uart.hpp"

USING_SIMULATOR_NAMESPACE::interfaces;

// Handles are still needed
static UART_HandleTypeDef uart1 = {1, "UART#1"};
static UART_HandleTypeDef uart2 = {2, "UART#2"};

TEST(MockInterfaceTests, SimpleTransmitReceive) {
    // 1. Setup
    MockUARTDevice mockUart(&uart1);

    // 2. Feed data into RX (simulate external device sending)
    mockUart.feedRx({0xDE, 0xAD, 0xBE, 0xEF});

    // 3. Driver calls HAL
    uint8_t buf[4] = {};
    HAL_UART_Receive(&uart1, buf, 4, 100);

    // 4. Verify
    EXPECT_EQ(buf[0], 0xDE);
    EXPECT_EQ(buf[3], 0xEF);
}

TEST(MockInterfaceTests, CrossWiredCommunication) {
    // 1. Setup two devices
    MockUARTDevice devA(&uart1);
    MockUARTDevice devB(&uart2);

    // 2. Wire them together (A TX -> B RX)
    devA.connectTo(&devB);

    // 3. Transmit from A
    uint8_t msg[] = {0x10, 0x20};
    HAL_UART_Transmit(&uart1, msg, 2, 100);

    // 4. Verify A logged it
    auto aTx = devA.captureTx();
    ASSERT_EQ(aTx.size(), 2);
    EXPECT_EQ(aTx[0], 0x10);

    // 5. Verify B received it automatically
    uint8_t rxBuf[2] = {};
    HAL_UART_Receive(&uart2, rxBuf, 2, 100);

    EXPECT_EQ(rxBuf[0], 0x10);
    EXPECT_EQ(rxBuf[1], 0x20);
}

TEST(MockInterfaceTests, BidirectionalWire) {
    MockUARTDevice devA(&uart1);
    MockUARTDevice devB(&uart2);

    // Wire both directions
    devA.connectTo(&devB);
    devB.connectTo(&devA);

    // A sends
    uint8_t req[] = {0x01};
    HAL_UART_Transmit(&uart1, req, 1, 10);
    
    // B receives
    uint8_t bBuf[1] = {};
    HAL_UART_Receive(&uart2, bBuf, 1, 10);
    EXPECT_EQ(bBuf[0], 0x01);

    // B responds
    uint8_t resp[] = {0x99};
    HAL_UART_Transmit(&uart2, resp, 1, 10);

    // A receives
    uint8_t aBuf[1] = {};
    HAL_UART_Receive(&uart1, aBuf, 1, 10);
    EXPECT_EQ(aBuf[0], 0x99);
}
