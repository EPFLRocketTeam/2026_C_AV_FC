#include <cstring>
#include <queue>
#include <vector>

#include <gtest/gtest.h>

#include "Drivers/STM32HAL/Simulations/stm32sim_uart.hpp"

USING_SIMULATOR_NAMESPACE::uart;

/* ------------------------------------------------------------------ */
/*  Test UART handles                                                  */
/* ------------------------------------------------------------------ */

static UART_HandleTypeDef uart1 = {1, "UART#1"};
static UART_HandleTypeDef uart2 = {2, "UART#2"};
static UART_HandleTypeDef uart3 = {3, "UART#3"};

/* ------------------------------------------------------------------ */
/*  Per-device simulated state                                         */
/* ------------------------------------------------------------------ */

struct FakeUARTState {
    std::queue<uint8_t> rxQueue;
    std::vector<uint8_t> txLog;
    std::vector<uint32_t> txTimeouts;
    std::vector<uint32_t> rxTimeouts;
};

static FakeUARTState stateA, stateB, stateC;


static void fakeTransmit(
    void *data, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    auto *s = static_cast<FakeUARTState *>(data);
    s->txTimeouts.push_back(Timeout);
    for (uint16_t i = 0; i < Size; i++) {
        s->txLog.push_back(pData[i]);
    }
}

static HAL_StatusTypeDef fakeReceive(
    void *data, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    auto *s = static_cast<FakeUARTState *>(data);
    s->rxTimeouts.push_back(Timeout);
    if (s->rxQueue.size() < Size) {
        return HAL_TIMEOUT;
    }
    for (uint16_t i = 0; i < Size; i++) {
        pData[i] = s->rxQueue.front();
        s->rxQueue.pop();
    }
    return HAL_OK;
}


class SimulatorUARTTests : public ::testing::Test {
  protected:
    std::vector<SIM_UARTDevice> registered;

    void TearDown() override {
        for (auto d : registered) {
            SIM_UnregisterUARTDevice(d);
        }
        registered.clear();
        stateA = {};
        stateB = {};
        stateC = {};
    }

    SIM_UARTDevice reg(UART_HandleTypeDef *h, FakeUARTState *s) {
        auto d = SIM_RegisterUARTDevice(
            h, {s, fakeTransmit, fakeReceive});
        registered.push_back(d);
        return d;
    }
};


TEST_F(SimulatorUARTTests, RegisterSingleDevice) {
    auto d = reg(&uart1, &stateA);
    EXPECT_GE(d, HAL_OK);
}

TEST_F(SimulatorUARTTests, RegisterMultipleBuses) {
    auto d1 = reg(&uart1, &stateA);
    auto d2 = reg(&uart2, &stateB);
    EXPECT_GE(d1, HAL_OK);
    EXPECT_GE(d2, HAL_OK);
    EXPECT_NE(d1, d2);
}

TEST_F(SimulatorUARTTests, RegisterTwiceOnSameHandleFails) {
    reg(&uart1, &stateA);

    bool threw = false;
    try {
        SIM_RegisterUARTDevice(&uart1, {&stateB, fakeTransmit, fakeReceive});
    } catch (const std::runtime_error &e) {
        EXPECT_STREQ(
            e.what(), "UART device already exists (bus=UART#1)");
        threw = true;
    }
    EXPECT_TRUE(threw);
}

TEST_F(SimulatorUARTTests, UnregisterAllowsReRegister) {
    auto d = reg(&uart1, &stateA);
    SIM_UnregisterUARTDevice(d);
    registered.clear();

    auto d2 = reg(&uart1, &stateB);
    EXPECT_GE(d2, HAL_OK);
}

TEST_F(SimulatorUARTTests, UnregisterAndRegisterRepeatedly) {
    for (int i = 0; i < 512; i++) {
        auto d = SIM_RegisterUARTDevice(
            &uart1, {&stateA, fakeTransmit, fakeReceive});
        EXPECT_EQ(d, HAL_OK);
        SIM_UnregisterUARTDevice(d);
    }
}

TEST_F(SimulatorUARTTests, UnregisterNonexistentIsNoop) {
    EXPECT_NO_THROW(SIM_UnregisterUARTDevice(42));
    EXPECT_NO_THROW(SIM_UnregisterUARTDevice(-1));
}


TEST_F(SimulatorUARTTests, TransmitSingleMessage) {
    reg(&uart1, &stateA);

    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};
    auto status = HAL_UART_Transmit(&uart1, data, 4, 100);

    EXPECT_EQ(status, HAL_OK);
    ASSERT_EQ(stateA.txLog.size(), 4u);
    EXPECT_EQ(stateA.txLog[0], 0xAA);
    EXPECT_EQ(stateA.txLog[1], 0xBB);
    EXPECT_EQ(stateA.txLog[2], 0xCC);
    EXPECT_EQ(stateA.txLog[3], 0xDD);
}

TEST_F(SimulatorUARTTests, TransmitTimeoutForwarded) {
    reg(&uart1, &stateA);

    uint8_t data[] = {0x01};
    HAL_UART_Transmit(&uart1, data, 1, 500);
    HAL_UART_Transmit(&uart1, data, 1, 1000);

    ASSERT_EQ(stateA.txTimeouts.size(), 2u);
    EXPECT_EQ(stateA.txTimeouts[0], 500u);
    EXPECT_EQ(stateA.txTimeouts[1], 1000u);
}

TEST_F(SimulatorUARTTests, TransmitMultipleBusesIndependent) {
    reg(&uart1, &stateA);
    reg(&uart2, &stateB);

    uint8_t msgA[] = {0x11, 0x22};
    uint8_t msgB[] = {0x33, 0x44, 0x55};

    HAL_UART_Transmit(&uart1, msgA, 2, 10);
    HAL_UART_Transmit(&uart2, msgB, 3, 20);

    ASSERT_EQ(stateA.txLog.size(), 2u);
    EXPECT_EQ(stateA.txLog[0], 0x11);
    EXPECT_EQ(stateA.txLog[1], 0x22);

    ASSERT_EQ(stateB.txLog.size(), 3u);
    EXPECT_EQ(stateB.txLog[0], 0x33);
    EXPECT_EQ(stateB.txLog[1], 0x44);
    EXPECT_EQ(stateB.txLog[2], 0x55);
}

TEST_F(SimulatorUARTTests, TransmitZeroLength) {
    reg(&uart1, &stateA);

    uint8_t data = 0xFF;
    auto status = HAL_UART_Transmit(&uart1, &data, 0, 10);

    EXPECT_EQ(status, HAL_OK);
    EXPECT_TRUE(stateA.txLog.empty());
}

TEST_F(SimulatorUARTTests, TransmitOnUnregisteredHandleThrows) {
    /* uart3 has no device registered */
    uint8_t data[] = {0x01};
    bool threw = false;
    try {
        HAL_UART_Transmit(&uart3, data, 1, 10);
    } catch (const std::runtime_error &e) {
        EXPECT_STREQ(
            e.what(), "UART Handle (UART#3) has no registered device.");
        threw = true;
    }
    EXPECT_TRUE(threw);
}

/* ------------------------------------------------------------------ */
/*  Receive tests                                                      */
/* ------------------------------------------------------------------ */

TEST_F(SimulatorUARTTests, ReceiveSuccess) {
    reg(&uart1, &stateA);

    stateA.rxQueue.push(0xDE);
    stateA.rxQueue.push(0xAD);
    stateA.rxQueue.push(0xBE);
    stateA.rxQueue.push(0xEF);

    uint8_t buf[4] = {};
    auto status = HAL_UART_Receive(&uart1, buf, 4, 200);

    EXPECT_EQ(status, HAL_OK);
    EXPECT_EQ(buf[0], 0xDE);
    EXPECT_EQ(buf[1], 0xAD);
    EXPECT_EQ(buf[2], 0xBE);
    EXPECT_EQ(buf[3], 0xEF);
    EXPECT_TRUE(stateA.rxQueue.empty());
}

TEST_F(SimulatorUARTTests, ReceiveTimeoutWhenNotEnoughData) {
    reg(&uart1, &stateA);

    stateA.rxQueue.push(0x01); /* only 1 byte available */

    uint8_t buf[4] = {};
    auto status = HAL_UART_Receive(&uart1, buf, 4, 500);

    EXPECT_EQ(status, HAL_TIMEOUT);
    /* Queue should be untouched since handler couldn't fulfill request */
    EXPECT_EQ(stateA.rxQueue.size(), 1u);
}

TEST_F(SimulatorUARTTests, ReceiveTimeoutWhenEmpty) {
    reg(&uart1, &stateA);

    uint8_t buf[1] = {};
    auto status = HAL_UART_Receive(&uart1, buf, 1, 50);

    EXPECT_EQ(status, HAL_TIMEOUT);
}

TEST_F(SimulatorUARTTests, ReceiveTimeoutForwarded) {
    reg(&uart1, &stateA);

    uint8_t buf[1];
    HAL_UART_Receive(&uart1, buf, 1, 750);

    ASSERT_EQ(stateA.rxTimeouts.size(), 1u);
    EXPECT_EQ(stateA.rxTimeouts[0], 750u);
}

TEST_F(SimulatorUARTTests, ReceiveMultipleChunks) {
    reg(&uart1, &stateA);

    for (uint8_t i = 0; i < 10; i++) {
        stateA.rxQueue.push(i);
    }

    uint8_t buf[4] = {};
    EXPECT_EQ(HAL_UART_Receive(&uart1, buf, 4, 10), HAL_OK);
    EXPECT_EQ(buf[0], 0);
    EXPECT_EQ(buf[3], 3);

    EXPECT_EQ(HAL_UART_Receive(&uart1, buf, 4, 10), HAL_OK);
    EXPECT_EQ(buf[0], 4);
    EXPECT_EQ(buf[3], 7);

    EXPECT_EQ(HAL_UART_Receive(&uart1, buf, 2, 10), HAL_OK);
    EXPECT_EQ(buf[0], 8);
    EXPECT_EQ(buf[1], 9);

    /* Queue drained, next read should timeout */
    EXPECT_EQ(HAL_UART_Receive(&uart1, buf, 1, 10), HAL_TIMEOUT);
}

TEST_F(SimulatorUARTTests, ReceiveOnUnregisteredHandleThrows) {
    uint8_t buf[1];
    bool threw = false;
    try {
        HAL_UART_Receive(&uart3, buf, 1, 10);
    } catch (const std::runtime_error &e) {
        EXPECT_STREQ(
            e.what(), "UART Handle (UART#3) has no registered device.");
        threw = true;
    }
    EXPECT_TRUE(threw);
}

TEST_F(SimulatorUARTTests, ReceiveMultipleBusesIndependent) {
    reg(&uart1, &stateA);
    reg(&uart2, &stateB);

    stateA.rxQueue.push(0xAA);
    stateB.rxQueue.push(0xBB);

    uint8_t buf = 0;
    EXPECT_EQ(HAL_UART_Receive(&uart1, &buf, 1, 10), HAL_OK);
    EXPECT_EQ(buf, 0xAA);

    EXPECT_EQ(HAL_UART_Receive(&uart2, &buf, 1, 10), HAL_OK);
    EXPECT_EQ(buf, 0xBB);

    /* Cross-check: uart1 queue is now empty */
    EXPECT_EQ(HAL_UART_Receive(&uart1, &buf, 1, 10), HAL_TIMEOUT);
}

/* ------------------------------------------------------------------ */
/*  Mixed Tx / Rx tests                                                */
/* ------------------------------------------------------------------ */

TEST_F(SimulatorUARTTests, InterleavedTxRx) {
    reg(&uart1, &stateA);

    /* Simulate a request-response pattern */
    uint8_t cmd[] = {0x01, 0x02};
    HAL_UART_Transmit(&uart1, cmd, 2, 100);

    stateA.rxQueue.push(0xAC);
    stateA.rxQueue.push(0xCA);

    uint8_t resp[2] = {};
    EXPECT_EQ(HAL_UART_Receive(&uart1, resp, 2, 100), HAL_OK);

    ASSERT_EQ(stateA.txLog.size(), 2u);
    EXPECT_EQ(stateA.txLog[0], 0x01);
    EXPECT_EQ(stateA.txLog[1], 0x02);
    EXPECT_EQ(resp[0], 0xAC);
    EXPECT_EQ(resp[1], 0xCA);
}


TEST_F(SimulatorUARTTests, OperateAfterUnregisterThrows) {
    auto d = reg(&uart1, &stateA);
    SIM_UnregisterUARTDevice(d);
    registered.clear();

    uint8_t buf[] = {0x00};

    bool threwTx = false;
    try {
        HAL_UART_Transmit(&uart1, buf, 1, 10);
    } catch (const std::runtime_error &) {
        threwTx = true;
    }
    EXPECT_TRUE(threwTx);

    bool threwRx = false;
    try {
        HAL_UART_Receive(&uart1, buf, 1, 10);
    } catch (const std::runtime_error &) {
        threwRx = true;
    }
    EXPECT_TRUE(threwRx);
}

TEST_F(SimulatorUARTTests, InterleavedCrossConnectedDevices) {

    struct CrossWireState {
        std::queue<uint8_t> rxQueue;
        CrossWireState* peer = nullptr;
    };

    CrossWireState stateA_local;
    CrossWireState stateB_local;

    stateA_local.peer = &stateB_local;
    stateB_local.peer = &stateA_local;

    auto crossTransmit = [](
        void* data,
        const uint8_t* pData,
        uint16_t Size,
        uint32_t /*Timeout*/) {

        auto* self = static_cast<CrossWireState*>(data);

        for (uint16_t i = 0; i < Size; ++i) {
            self->peer->rxQueue.push(pData[i]);
        }
    };

    auto crossReceive = [](
        void* data,
        uint8_t* pData,
        uint16_t Size,
        uint32_t /*Timeout*/) -> HAL_StatusTypeDef {

        auto* self = static_cast<CrossWireState*>(data);

        if (self->rxQueue.size() < Size)
            return HAL_TIMEOUT;

        for (uint16_t i = 0; i < Size; ++i) {
            pData[i] = self->rxQueue.front();
            self->rxQueue.pop();
        }

        return HAL_OK;
    };

    auto devA = SIM_RegisterUARTDevice(
        &uart1, {&stateA_local, crossTransmit, crossReceive});
    auto devB = SIM_RegisterUARTDevice(
        &uart2, {&stateB_local, crossTransmit, crossReceive});

    registered.push_back(devA);
    registered.push_back(devB);

    /* -------------------------
       A sends to B
       ------------------------- */

    uint8_t msgA[] = {0x10, 0x20, 0x30};
    EXPECT_EQ(HAL_UART_Transmit(&uart1, msgA, 3, 100), HAL_OK);

    uint8_t rxB[3] = {};
    EXPECT_EQ(HAL_UART_Receive(&uart2, rxB, 3, 100), HAL_OK);

    EXPECT_EQ(rxB[0], 0x10);
    EXPECT_EQ(rxB[1], 0x20);
    EXPECT_EQ(rxB[2], 0x30);

    /* -------------------------
       B replies to A
       ------------------------- */

    uint8_t msgB[] = {0xAA, 0xBB};
    EXPECT_EQ(HAL_UART_Transmit(&uart2, msgB, 2, 100), HAL_OK);

    uint8_t rxA[2] = {};
    EXPECT_EQ(HAL_UART_Receive(&uart1, rxA, 2, 100), HAL_OK);

    EXPECT_EQ(rxA[0], 0xAA);
    EXPECT_EQ(rxA[1], 0xBB);

    /* -------------------------
       Interleaved behavior
       ------------------------- */

    uint8_t burstA[] = {1,2,3,4};
    uint8_t burstB[] = {9,8,7};

    HAL_UART_Transmit(&uart1, burstA, 4, 100);
    HAL_UART_Transmit(&uart2, burstB, 3, 100);

    uint8_t readB[4];
    uint8_t readA[3];

    EXPECT_EQ(HAL_UART_Receive(&uart2, readB, 4, 100), HAL_OK);
    EXPECT_EQ(HAL_UART_Receive(&uart1, readA, 3, 100), HAL_OK);

    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(readB[i], burstA[i]);

    for (int i = 0; i < 3; ++i)
        EXPECT_EQ(readA[i], burstB[i]);

    /* Ensure queues are empty afterward */
    uint8_t dummy;
    EXPECT_EQ(HAL_UART_Receive(&uart1, &dummy, 1, 10), HAL_TIMEOUT);
    EXPECT_EQ(HAL_UART_Receive(&uart2, &dummy, 1, 10), HAL_TIMEOUT);
}
