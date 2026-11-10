#include "Application/usb_log.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

// A fake CDC endpoint: busy for the next `busy_calls` attempts, then it
// queues. Each tick() call advances the fake clock by 1 ms.
int g_busy_calls = 0;
uint32_t g_now = 0;
std::vector<std::string> g_sent;

int FakeTransmit(const uint8_t* data, uint16_t length) {
    if (g_busy_calls > 0) {
        --g_busy_calls;
        return 1;
    }
    g_sent.emplace_back(reinterpret_cast<const char*>(data), length);
    return 0;
}

uint32_t FakeTick() { return g_now++; }

struct UsbLogTest : ::testing::Test {
    UsbLog log{};
    void SetUp() override {
        g_busy_calls = 0;
        g_now = 1000;
        g_sent.clear();
    }
    void Write(const std::string& s, int configured = 1) {
        usb_log_write(&log, s.data(), static_cast<int>(s.size()), configured, FakeTransmit, FakeTick);
    }
};

}  // namespace

TEST_F(UsbLogTest, UnconfiguredPortDiscardsWithoutCounting) {
    Write("hello\r\n", 0);
    EXPECT_TRUE(g_sent.empty());
    EXPECT_EQ(log.dropped, 0u);
    EXPECT_EQ(log.stalled, 0);
}

TEST_F(UsbLogTest, ReadyEndpointSendsAtOnce) {
    Write("hello\r\n");
    ASSERT_EQ(g_sent.size(), 1u);
    EXPECT_EQ(g_sent[0], "hello\r\n");
}

TEST_F(UsbLogTest, BusyEndpointIsRetriedWithinTheBudget) {
    g_busy_calls = 3;  // the clock moves 1 ms per check, under 5 ms
    Write("late\r\n");
    ASSERT_EQ(g_sent.size(), 1u);
    EXPECT_EQ(log.stalled, 0);
}

TEST_F(UsbLogTest, BudgetOfFiveMillisecondsStallsAndDrops) {
    g_busy_calls = 100;
    Write("lost\r\n");
    EXPECT_TRUE(g_sent.empty());
    EXPECT_EQ(log.stalled, 1);
    EXPECT_EQ(log.dropped, 1u);
    // Elapsed time stayed within the budget: start read plus five checks.
    EXPECT_LE(g_now - 1000u, USB_LOG_BUSY_BUDGET_MS + 1u);
}

TEST_F(UsbLogTest, StalledLogDropsWithoutTouchingTheEndpoint) {
    log.stalled = 1;
    g_busy_calls = 0;
    Write("a\r\n");
    Write("b\r\n");
    EXPECT_TRUE(g_sent.empty());
    EXPECT_EQ(log.dropped, 2u);
}

TEST_F(UsbLogTest, NextWriteAfterTheStallReportsTheDropsOnce) {
    log.stalled = 1;
    Write("a\r\n");
    Write("b\r\n");
    Write("c\r\n");
    usb_log_clear_stall(&log);
    Write("back\r\n");
    Write("again\r\n");
    ASSERT_EQ(g_sent.size(), 3u);
    EXPECT_EQ(g_sent[0], "[USB] dropped 3 lines\r\n");
    EXPECT_EQ(g_sent[1], "back\r\n");
    EXPECT_EQ(g_sent[2], "again\r\n");
    EXPECT_EQ(log.dropped, 0u);
}

TEST_F(UsbLogTest, NoticeThatCannotGoOutKeepsCounting) {
    log.dropped = 2;
    g_busy_calls = 100;
    Write("x\r\n");
    EXPECT_TRUE(g_sent.empty());
    EXPECT_EQ(log.dropped, 3u);
    EXPECT_EQ(log.stalled, 1);
}

TEST_F(UsbLogTest, LineThatCannotFollowTheNoticeIsCounted) {
    log.dropped = 4;
    // The notice queues at once; the line then finds the endpoint busy.
    struct Once {
        static int Transmit(const uint8_t* data, uint16_t length) {
            static int calls = 0;
            if (calls++ == 0) return FakeTransmit(data, length);
            return 1;
        }
    };
    usb_log_write(&log, "y\r\n", 3, 1, Once::Transmit, FakeTick);
    ASSERT_EQ(g_sent.size(), 1u);
    EXPECT_EQ(g_sent[0], "[USB] dropped 4 lines\r\n");
    EXPECT_EQ(log.dropped, 1u);
    EXPECT_EQ(log.stalled, 1);
}
