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

TEST(PrcLinkMonitor, DuplicateBeatCountsNoLoss) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Eth, 5, 100);
    m.onHeartbeat(Board::Eth, 5, 150);
    EXPECT_EQ(m.link(Board::Eth).missed, 0u);
    EXPECT_EQ(m.link(Board::Eth).received, 2u);
    EXPECT_EQ(m.link(Board::Eth).last_rx_ms, 150u);
}

TEST(PrcLinkMonitor, RebootThresholdAtTwoPow31) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Lox, 0, 0);
    m.onHeartbeat(Board::Lox, 0x80000000u, 50);  // step 2^31: reboot, nothing counted
    EXPECT_EQ(m.link(Board::Lox).missed, 0u);
}

TEST(PrcLinkMonitor, JumpJustBelowTwoPow31IsLoss) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Lox, 0, 0);
    m.onHeartbeat(Board::Lox, 0x7FFFFFFFu, 50);  // step 2^31 - 1
    EXPECT_EQ(m.link(Board::Lox).missed, 0x7FFFFFFEu);
}

TEST(PrcLinkMonitor, NowWrapKeepsLiveBoardAlive) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Engine, 1, 0xFFFFFF00u);
    EXPECT_TRUE(m.alive(Board::Engine, 0x00000010u));    // 272 ms later
    EXPECT_FALSE(m.alive(Board::Engine, 0x00000200u));   // 768 ms later
}

TEST(PrcLinkMonitor, BootGraceReportsNeverSeenOnce) {
    PrcLinkMonitor m;
    EXPECT_EQ(m.takeReport(100), 0u);                    // first call: the grace runs to 2100
    m.onHeartbeat(Board::Engine, 0, 1800);
    EXPECT_EQ(m.takeReport(1800), 0x1u);                 // engine came alive: edge
    EXPECT_EQ(m.takeReport(2099), 0u);
    EXPECT_EQ(m.takeReport(2100), 0x6u);                 // eth and lox never seen: one dead line each
    EXPECT_EQ(m.aliveMask(2100), 0x1u);
    EXPECT_EQ(m.takeReport(2150), 0u);                   // the grace fires once
}

// The grace counts from the first call, not from reset: a setup longer than
// 2 s with the boards already sending prints no false "dead".
TEST(PrcLinkMonitor, GraceAnchorsOnFirstCall) {
    PrcLinkMonitor m;
    EXPECT_EQ(m.takeReport(5000), 0u);                   // first call 5 s after reset, nothing drained yet
    m.onHeartbeat(Board::Engine, 0, 5010);
    m.onHeartbeat(Board::Eth, 0, 5010);
    EXPECT_EQ(m.takeReport(5010), 0x3u);                 // engine and eth alive, no false dead
    m.onHeartbeat(Board::Engine, 39, 6990);
    m.onHeartbeat(Board::Eth, 39, 6990);
    EXPECT_EQ(m.takeReport(6999), 0u);                   // still inside the grace
    EXPECT_EQ(m.takeReport(7000), 0x4u);                 // lox silent 2 s after the first call: dead once
    EXPECT_EQ(m.aliveMask(7000), 0x3u);
    EXPECT_EQ(m.takeReport(7050), 0u);
}

TEST(PrcLinkMonitor, AfterGraceOnlyEdges) {
    PrcLinkMonitor m;
    EXPECT_EQ(m.takeReport(0), 0u);                      // first call at reset
    m.onHeartbeat(Board::Eth, 0, 1900);
    EXPECT_EQ(m.takeReport(1900), 0x2u);                 // eth alive
    EXPECT_EQ(m.takeReport(2100), 0x5u);                 // grace: engine and lox never seen
    EXPECT_EQ(m.takeReport(2200), 0u);                   // no repeat
    EXPECT_EQ(m.takeReport(2400), 0x2u);                 // eth dead at 2400
    EXPECT_EQ(m.takeReport(3000), 0u);
    m.onHeartbeat(Board::Lox, 0, 3000);
    EXPECT_EQ(m.takeReport(3000), 0x4u);                 // lox comes back: edge
}

TEST(PrcLinkMonitor, SeenBoardDyingInGraceIsNotRepeated) {
    PrcLinkMonitor m;
    m.onHeartbeat(Board::Engine, 0, 100);
    m.onHeartbeat(Board::Eth, 0, 100);
    m.onHeartbeat(Board::Lox, 0, 100);
    EXPECT_EQ(m.takeReport(100), 0x7u);
    EXPECT_EQ(m.takeReport(700), 0x7u);                  // all dead: edges
    EXPECT_EQ(m.takeReport(2100), 0u);                   // all were seen: grace adds nothing
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
