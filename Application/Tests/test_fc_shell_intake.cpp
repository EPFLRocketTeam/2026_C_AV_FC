#include "Application/FlightControl/fc_shell_intake.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <string>
#include <vector>

namespace {

// Mirrors fc_shell.cpp: the USB interrupt pushes packets into the ring, the
// main loop drains it through the line assembler.
struct Intake {
    fc_shell::SpscRing<256> ring;
    fc_shell::LineAssembler<128> line;
    std::vector<std::string> lines;

    uint32_t Packet(const std::string& bytes) {
        return ring.Push(reinterpret_cast<const uint8_t*>(bytes.data()),
                         static_cast<uint32_t>(bytes.size()));
    }

    void Tick() {
        uint8_t byte;
        while (ring.Pop(byte)) {
            line.Feed(byte, [this](const char* text, uint32_t length) {
                lines.emplace_back(text, length);
            });
        }
    }
};

}  // namespace

TEST(FcShellIntake, LineSplitAcrossTwoPacketsSurvives) {
    Intake in;
    in.Packet("main lo");
    in.Tick();
    EXPECT_TRUE(in.lines.empty());
    in.Packet("x open\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "main lox open");
}

TEST(FcShellIntake, SplitSurvivesWithoutATickBetweenPackets) {
    Intake in;
    in.Packet("system ");
    in.Packet("dfu\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "system dfu");
}

TEST(FcShellIntake, CrLfIsOneLineEnding) {
    Intake in;
    in.Packet("arm\r\nabort\r");
    in.Packet("\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 2u);
    EXPECT_EQ(in.lines[0], "arm");
    EXPECT_EQ(in.lines[1], "abort");
}

TEST(FcShellIntake, TrailingSpacesAreTrimmed) {
    Intake in;
    in.Packet("main lox open   \r\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "main lox open");
}

TEST(FcShellIntake, EmptyLineStillEndsALine) {
    Intake in;
    in.Packet("\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "");
}

TEST(FcShellIntake, OverlongLineIsDroppedAndTheNextOneWorks) {
    Intake in;
    in.Packet(std::string(200, 'a'));
    in.Tick();
    in.Packet("\nreboot\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "reboot");
}

TEST(FcShellIntake, LineOfExactlyTheBufferSizeIsKept) {
    Intake in;
    const std::string full(128, 'b');
    in.Packet(full + "\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], full);
}

TEST(FcShellIntake, RingWrapsAroundManyTimes) {
    Intake in;
    // 100 byte packets against a 256 byte ring: indexes wrap every few
    // packets, and each packet is drained before the next.
    for (int i = 0; i < 50; ++i) {
        const std::string cmd = "cmd" + std::to_string(i);
        const std::string packet = cmd + std::string(100 - cmd.size() - 1, ' ') + "\n";
        ASSERT_EQ(in.Packet(packet), 100u);
        in.Tick();
        ASSERT_EQ(in.lines.size(), static_cast<size_t>(i + 1));
        EXPECT_EQ(in.lines.back(), cmd);
    }
}

TEST(FcShellIntake, FullRingDropsOnlyTheExcess) {
    Intake in;
    EXPECT_EQ(in.Packet(std::string(250, 'c')), 250u);
    EXPECT_EQ(in.Packet("0123456789"), 6u);
    EXPECT_EQ(in.Packet("x"), 0u);
    in.Tick();
    EXPECT_EQ(in.Packet("\nok\n"), 4u);
    in.Tick();
    // 256 bytes without a newline overflowed the 128 byte line: dropped.
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "ok");
}

TEST(FcShellIntake, PopOnEmptyRingReturnsFalse) {
    fc_shell::SpscRing<256> ring;
    uint8_t byte = 0;
    EXPECT_FALSE(ring.Pop(byte));
    const uint8_t data[2] = {'h', 'i'};
    EXPECT_EQ(ring.Push(data, 2), 2u);
    EXPECT_TRUE(ring.Pop(byte));
    EXPECT_EQ(byte, 'h');
    EXPECT_TRUE(ring.Pop(byte));
    EXPECT_EQ(byte, 'i');
    EXPECT_FALSE(ring.Pop(byte));
}
