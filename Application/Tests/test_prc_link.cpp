#include <gtest/gtest.h>

#include "Application/FlightControl/prc_link.hpp"

using Board = PrcLinkMonitor::Board;

TEST(PrcLinkMonitor, DeadBeforeAnyBeat) {
    PrcLinkMonitor m;
    EXPECT_FALSE(m.alive(Board::Engine, 0));
    EXPECT_EQ(m.aliveMask(0), 0u);
}

TEST(PrcLinkMonitor, AliveAfterOneBeat) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Engine, 0, 1000);
    EXPECT_TRUE(m.alive(Board::Engine, 1000));
    EXPECT_TRUE(m.alive(Board::Engine, 1499));
    EXPECT_FALSE(m.alive(Board::Eth, 1000));
    EXPECT_EQ(m.link(Board::Engine).received, 1u);
    EXPECT_EQ(m.link(Board::Engine).last_rx_ms, 1000u);
}

TEST(PrcLinkMonitor, DeadAfter500ms) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Lox, 0, 1000);
    EXPECT_FALSE(m.alive(Board::Lox, 1500));
    m.onHeartbeat(Board::Lox, 1, 1600);
    EXPECT_TRUE(m.alive(Board::Lox, 1600));
}

TEST(PrcLinkMonitor, CounterWrapIsOneStep) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Eth, 0xFFFFFFFFu, 0);
    m.onHeartbeat(Board::Eth, 0, 50);
    EXPECT_EQ(m.link(Board::Eth).missed, 0u);
    EXPECT_EQ(m.link(Board::Eth).last_counter, 0u);
}

TEST(PrcLinkMonitor, MissedCountsCounterJumps) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Engine, 10, 0);
    m.onHeartbeat(Board::Engine, 11, 50);
    m.onHeartbeat(Board::Engine, 15, 250);   // 12, 13, 14 lost
    EXPECT_EQ(m.link(Board::Engine).missed, 3u);
    m.onHeartbeat(Board::Engine, 0, 300);    // reboot, not a loss
    EXPECT_EQ(m.link(Board::Engine).missed, 3u);
    EXPECT_EQ(m.link(Board::Engine).received, 4u);
}

TEST(PrcLinkMonitor, MaskBits) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Engine, 0, 0);
    EXPECT_EQ(m.aliveMask(10), 0x1u);
    m.onHeartbeat(Board::Eth, 0, 0);
    EXPECT_EQ(m.aliveMask(10), 0x3u);
    m.onHeartbeat(Board::Lox, 0, 300);
    EXPECT_EQ(m.aliveMask(310), 0x7u);
    EXPECT_EQ(m.aliveMask(600), 0x4u);       // engine and eth timed out
}
