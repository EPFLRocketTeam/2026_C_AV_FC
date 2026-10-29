#include "Application/FlightControl/fc_shell_intake.hpp"
#include "Application/FlightControl/fc_shell_replay.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

using Ring = fc_shell::SpscRing<256>;

// The same Intake<256, 128> that fc_shell.cpp drives: Receive stands in for
// the USB interrupt, Drain for FC_Shell_Tick.
struct Shell {
    fc_shell::Intake<256, 128> intake;
    std::vector<std::string> lines;

    uint32_t Packet(const std::string& bytes) {
        return intake.Receive(reinterpret_cast<const uint8_t*>(bytes.data()),
                              static_cast<uint32_t>(bytes.size()));
    }

    void Tick() {
        intake.Drain([this](const char* text, uint32_t length) {
            lines.emplace_back(text, length);
        });
    }
};

uint32_t Push(Ring& ring, const std::string& bytes) {
    return ring.Push(reinterpret_cast<const uint8_t*>(bytes.data()),
                     static_cast<uint32_t>(bytes.size()));
}

std::string Line(char c, size_t n) { return std::string(n, c) + "\n"; }

}  // namespace

TEST(FcShellIntake, LineSplitAcrossTwoPacketsSurvives) {
    Shell in;
    in.Packet("main lo");
    in.Tick();
    EXPECT_TRUE(in.lines.empty());
    in.Packet("x open\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "main lox open");
}

TEST(FcShellIntake, SplitSurvivesWithoutATickBetweenPackets) {
    Shell in;
    in.Packet("system ");
    in.Packet("dfu\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "system dfu");
}

TEST(FcShellIntake, CrLfIsOneLineEnding) {
    Shell in;
    in.Packet("arm\r\nabort\r");
    in.Packet("\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 2u);
    EXPECT_EQ(in.lines[0], "arm");
    EXPECT_EQ(in.lines[1], "abort");
}

TEST(FcShellIntake, CrInsideALineIsSkipped) {
    Shell in;
    in.Packet("ar\rm\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "arm");
}

TEST(FcShellIntake, TrailingSpacesAreTrimmed) {
    Shell in;
    in.Packet("main lox open   \r\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "main lox open");
}

TEST(FcShellIntake, LineOfOnlySpacesBecomesEmpty) {
    Shell in;
    in.Packet("   \n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "");
}

TEST(FcShellIntake, EmptyLineStillEndsALine) {
    Shell in;
    in.Packet("\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "");
}

TEST(FcShellIntake, OverlongLineIsDroppedAndTheNextOneWorks) {
    Shell in;
    in.Packet(std::string(200, 'a'));
    in.Tick();
    in.Packet("\nreboot\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "reboot");
}

TEST(FcShellIntake, LineOfExactlyTheBufferSizeIsKept) {
    Shell in;
    const std::string full(128, 'b');
    in.Packet(full + "\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], full);
}

TEST(FcShellIntake, The129thByteDropsTheLine) {
    Shell in;
    in.Packet(Line('b', 129));
    in.Packet("ok\n");
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], "ok");
}

TEST(FcShellIntake, RingWrapsAroundManyTimes) {
    Shell in;
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

// The review's scenario: the main loop stalls, the ring fills after
// "main lox", " close\nmain fuel" is dropped and " open\n" arrives later.
// The old intake would have run "main lox open".
TEST(FcShellIntake, CutLineIsNeverCompletedByALaterPacket) {
    Shell in;
    EXPECT_EQ(in.Packet(Line('a', 123) + Line('b', 123)), 248u);
    EXPECT_EQ(in.Packet("main lox close\nmain fuel"), 8u);
    in.Tick();
    ASSERT_EQ(in.lines.size(), 2u);  // the two lines received whole still run
    EXPECT_EQ(in.Packet(" open\n"), 0u);  // rest of the cut "main fuel" line
    in.Tick();
    EXPECT_EQ(in.lines.size(), 2u);
    EXPECT_EQ(in.Packet("arm\n"), 4u);
    in.Tick();
    ASSERT_EQ(in.lines.size(), 3u);
    EXPECT_EQ(in.lines[2], "arm");
}

TEST(FcShellIntake, CutLineIsDroppedWhenTheGapEndsOnANewline) {
    Shell in;
    in.Packet(Line('a', 247));  // overlong, dropped by the assembler
    EXPECT_EQ(in.Packet("main lox close\n"), 8u);
    in.Tick();
    EXPECT_EQ(in.Packet(" lox open\n"), 10u);  // a fresh line after the gap
    in.Tick();
    ASSERT_EQ(in.lines.size(), 1u);
    EXPECT_EQ(in.lines[0], " lox open");
}

TEST(FcShellIntake, PushOfNothingStoresNothing) {
    Ring ring;
    EXPECT_EQ(ring.Push(nullptr, 0), 0u);
    uint8_t byte = 0;
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kEmpty);
}

TEST(FcShellIntake, PopOnEmptyRingReturnsEmpty) {
    Ring ring;
    uint8_t byte = 0;
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kEmpty);
    EXPECT_EQ(Push(ring, "hi"), 2u);
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kByte);
    EXPECT_EQ(byte, 'h');
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kByte);
    EXPECT_EQ(byte, 'i');
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kEmpty);
}

// Two gaps before the consumer reaches the first: everything between them
// is skipped, and the byte after the last gap starts a fresh line.
TEST(FcShellIntake, SeveralGapsSkipEverythingBetweenThem) {
    Ring ring;
    uint8_t byte = 0;
    EXPECT_EQ(Push(ring, std::string(250, 'p')), 250u);
    EXPECT_EQ(Push(ring, "main lox close\n"), 6u);  // gap at 256
    for (int i = 0; i < 10; ++i) EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kByte);
    EXPECT_EQ(Push(ring, "vent lox open\n"), 10u);  // gap at 266
    for (int i = 10; i < 256; ++i) EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kByte);
    for (int i = 256; i < 266; ++i) EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kSkipped);
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kEmpty);
    EXPECT_EQ(Push(ring, "arm\n"), 4u);
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kAfterGap);
    EXPECT_EQ(byte, 'a');
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kByte);
    // A later gap starts a new run.
    EXPECT_EQ(Push(ring, std::string(260, 'q')), 256u - 2u);
    for (int i = 0; i < 2 + 254; ++i) EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kByte);
    EXPECT_EQ(Push(ring, "q"), 0u);  // still the cut line
    EXPECT_EQ(Push(ring, "q\nx"), 1u);
    EXPECT_EQ(ring.Pop(byte), Ring::PopResult::kAfterGap);
}

TEST(FcShellIntake, FeedReturnsTrueOnlyWhenALineIsEmitted) {
    fc_shell::LineAssembler<4> line;
    int emitted = 0;
    auto count = [&emitted](const char*, uint32_t) { ++emitted; };
    EXPECT_FALSE(line.Feed('a', count));
    EXPECT_FALSE(line.Feed('\r', count));
    EXPECT_TRUE(line.Feed('\n', count));
    EXPECT_EQ(emitted, 1);
    for (char c : std::string("abcde")) EXPECT_FALSE(line.Feed(c, count));
    EXPECT_FALSE(line.Feed('\n', count));  // overlong: dropped
    EXPECT_EQ(emitted, 1);
}

namespace {
int g_reboots = 0;
int g_dfus = 0;
void CountReboot(void*) { ++g_reboots; }
void CountDfu(void*) { ++g_dfus; }
}  // namespace

// FC_Shell_Tick's handoff: Drain into ReplayLine into the generated trie.
TEST(FcShellIntake, DrainedLinesReachTheGeneratedTrie) {
    g_reboots = 0;
    g_dfus = 0;
    driver drv{};
    drv.system_reboot = CountReboot;
    drv.system_dfu = CountDfu;
    context ctx{};
    fc_shell::Intake<256, 128> intake;
    auto receive = [&intake](const std::string& s) {
        intake.Receive(reinterpret_cast<const uint8_t*>(s.data()),
                       static_cast<uint32_t>(s.size()));
    };
    auto tick = [&] {
        intake.Drain([&](const char* line, uint32_t length) {
            fc_shell::ReplayLine(&ctx, &drv, line, length);
        });
    };
    receive("system reb");
    tick();
    EXPECT_EQ(g_reboots, 0);
    receive("oot\r\nsystem dfu  \r\n");
    tick();
    EXPECT_EQ(g_reboots, 1);
    EXPECT_EQ(g_dfus, 1);
}
