#include <gtest/gtest.h>

#include <string>

#include "Application/FlightControl/prc_link.hpp"

using prc_intranet::link::LinkMatrix;
using prc_intranet::payload::heartbeat;
using namespace prc_intranet::constants;

namespace {

constexpr uint8_t FC  = LINK_BIT_FC;
constexpr uint8_t ENG = LINK_BIT_ENGINE;
constexpr uint8_t ETH = LINK_BIT_ETH;
constexpr uint8_t LOX = LINK_BIT_LOX;

heartbeat Beat(uint32_t uptime_ms, uint16_t counter, uint8_t alive_mask) {
    heartbeat hb{};
    hb.uptime_ms  = uptime_ms;
    hb.counter    = counter;
    hb.alive_mask = alive_mask;
    return hb;
}

std::string Line(uint16_t matrix, uint8_t line) {
    char buf[64];
    EXPECT_TRUE(fc_link::format_matrix_line(matrix, line, buf, sizeof(buf)));
    return buf;
}

} // namespace

// --- The FC's matrix, built by LinkMatrix with self = FC ---------------------

TEST(FcLinkMatrix, OnlyFcBitBeforeAnyBeat) {
    LinkMatrix m(FC);
    EXPECT_EQ(m.matrix(0), 0x0001u);
    EXPECT_EQ(m.ownRow(0), 0x1u);                  // sent in heartbeat_fc
}

TEST(FcLinkMatrix, AssemblesTheRowsOfThePrcBoards) {
    LinkMatrix m(FC);
    m.onHeartbeat(ENG, Beat(5000, 0, 0x3), 100);   // engine hears FC, itself
    m.onHeartbeat(ETH, Beat(5000, 0, 0xF), 100);   // eth hears all
    m.onHeartbeat(LOX, Beat(5000, 0, 0x9), 100);   // lox hears FC, itself
    EXPECT_EQ(m.ownRow(200), 0xFu);
    EXPECT_EQ(m.matrix(200), 0x9F3Fu);
}

TEST(FcLinkMatrix, SilentBoardRowAndBitDrop) {
    LinkMatrix m(FC);
    m.onHeartbeat(ENG, Beat(5000, 0, 0x3), 100);
    m.onHeartbeat(LOX, Beat(5000, 0, 0x9), 400);
    // At 650 the engine is silent for 550 ms: its row is 0, the FC's bit 1 too.
    EXPECT_EQ(m.matrix(650), 0x9009u);
}

TEST(FcLinkMatrix, ReportsEdgesAndReboots) {
    LinkMatrix m(FC);
    EXPECT_EQ(m.takeReport(0), 0u);
    m.onHeartbeat(ETH, Beat(60000, 1200, 0x5), 1000);
    EXPECT_EQ(m.takeReport(1000), 0x4u);           // eth alive
    m.onHeartbeat(ETH, Beat(3000, 0, 0x5), 1600);  // its uptime went back
    EXPECT_EQ(m.takeRebootReport(), 0x4u);
    EXPECT_EQ(m.takeReport(2000), 0xAu);           // grace: engine, lox never heard
}

// --- Radio field -------------------------------------------------------------

TEST(PrcAliveFromMatrix, EachBoardKeepsItsDownlinkBit) {
    // FC row bits: 1 engine, 2 eth, 3 lox; the FC bit (0) is always set.
    EXPECT_EQ(fc_link::prc_alive_from_matrix(0x0001), 0u);
    EXPECT_EQ(fc_link::prc_alive_from_matrix(0x0003), AV_PRC_ALIVE_ENGINE);
    EXPECT_EQ(fc_link::prc_alive_from_matrix(0x0005), AV_PRC_ALIVE_ETH);
    EXPECT_EQ(fc_link::prc_alive_from_matrix(0x0009), AV_PRC_ALIVE_LOX);
    EXPECT_EQ(fc_link::prc_alive_from_matrix(0x000F),
              AV_PRC_ALIVE_ENGINE | AV_PRC_ALIVE_ETH | AV_PRC_ALIVE_LOX);
}

// Same 3 bits as before the matrix: bit 0 engine, bit 1 eth, bit 2 lox.
TEST(PrcAliveFromMatrix, SameMeaningAsTheOldMask) {
    EXPECT_EQ(AV_PRC_ALIVE_ENGINE, 1 << 0);
    EXPECT_EQ(AV_PRC_ALIVE_ETH,    1 << 1);
    EXPECT_EQ(AV_PRC_ALIVE_LOX,    1 << 2);
    for (uint8_t old_mask = 0; old_mask < 8; old_mask++) {
        const uint16_t fc_row = static_cast<uint16_t>(0x1 | (old_mask << 1));
        EXPECT_EQ(fc_link::prc_alive_from_matrix(fc_row), old_mask) << int(old_mask);
    }
}

TEST(PrcAliveFromMatrix, OnlyTheFcRowCounts) {
    // The PRC rows say what the PRC boards hear, not what the FC hears.
    EXPECT_EQ(fc_link::prc_alive_from_matrix(0xFFF1), 0u);
}

// --- Console view ------------------------------------------------------------

TEST(FormatMatrixLine, HeaderNamesTheColumns) {
    EXPECT_EQ(Line(0x0001, 0), "             FC ENGINE    ETH    LOX");
}

TEST(FormatMatrixLine, RowsShowWhoHearsWho) {
    const uint16_t m = 0x0F77u;                     // LOX silent
    EXPECT_EQ(Line(m, 1), "FC            X      X      X      .");
    EXPECT_EQ(Line(m, 2), "ENGINE        X      X      X      .");
    EXPECT_EQ(Line(m, 3), "ETH           X      X      X      X");
    EXPECT_EQ(Line(m, 4), "LOX           -      -      -      -   silent");
}

TEST(FormatMatrixLine, OutOfRangeLineAndNoBuffer) {
    char buf[64];
    EXPECT_FALSE(fc_link::format_matrix_line(0x0001, fc_link::MATRIX_LINES, buf, sizeof(buf)));
    EXPECT_FALSE(fc_link::format_matrix_line(0x0001, 1, nullptr, sizeof(buf)));
}

// prc_can.cpp prints with a 64-byte buffer: the longest line (a silent row)
// must fit in it.
TEST(FormatMatrixLine, LongestLineFitsTheConsoleBuffer) {
    EXPECT_LT(Line(0x0001, 4).size(), 64u);         // LOX silent
    EXPECT_EQ(Line(0x0001, 4).size(), 45u);
}
