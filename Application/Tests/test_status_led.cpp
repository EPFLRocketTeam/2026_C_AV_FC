#include "Application/Modules/status_led.hpp"

#include <gtest/gtest.h>

using flight_computer::State;
using flight_computer::StatusLed;
using Rgb = StatusLed::Rgb;

namespace {

constexpr Rgb kOff{0, 0, 0};
constexpr Rgb kRed{64, 0, 0};
constexpr Rgb kGreen{0, 64, 0};

// Engine plus a fake clock. Every step feeds the same inputs at a new time.
class StatusLedTest : public ::testing::Test {
protected:
    StatusLed led;
    StatusLed::Input in{flight_computer::INIT, true, true, false, 1000};
    Rgb out[3]{};

    bool at(uint32_t t) {
        in.now_ms = t;
        return led.update(in, out);
    }

    // Enters state s at time t and waits past the entry blink.
    void settle(State s, uint32_t t) {
        in.av_state = s;
        at(t);
        at(t + 600);
    }
};

} // namespace

TEST(StatusLedPalette, ColourPerState) {
    const struct {
        State s;
        Rgb c;
    } table[] = {
        {flight_computer::INIT, {64, 64, 64}},
        {flight_computer::CALIBRATION, {0, 64, 64}},
        {flight_computer::FILLING, {0, 0, 64}},
        {flight_computer::ARMED, {64, 64, 0}},
        {flight_computer::PRESSURIZATION, {64, 24, 0}},
        {flight_computer::TRY_IGNITION, {64, 0, 64}},
        {flight_computer::IGNITION, {64, 0, 0}},
        {flight_computer::BURN, {64, 0, 0}},
        {flight_computer::ASCENT, {0, 64, 0}},
        {flight_computer::DESCENT, {32, 0, 64}},
        {flight_computer::LANDED, {0, 16, 0}},
        {flight_computer::ABORT_ON_GROUND, {64, 0, 0}},
        {flight_computer::ABORT_IN_FLIGHT, {64, 0, 0}},
    };
    for (const auto& row : table) {
        EXPECT_EQ(StatusLed::stateColour(row.s), row.c) << "state " << int(row.s);
    }
}

TEST(StatusLedPalette, BrightnessCapped) {
    for (size_t i = 0; i < StatusLed::kStateCount; ++i) {
        const Rgb c = StatusLed::stateColour(static_cast<State>(i));
        EXPECT_LE(c.r, StatusLed::kMaxChannel);
        EXPECT_LE(c.g, StatusLed::kMaxChannel);
        EXPECT_LE(c.b, StatusLed::kMaxChannel);
    }
}

TEST(StatusLedPalette, UnknownStateIsRed) {
    EXPECT_EQ(StatusLed::stateColour(static_cast<State>(200)), kRed);
}

TEST_F(StatusLedTest, SteadyColourAfterEntryBlink) {
    for (size_t i = 0; i < StatusLed::kStateCount; ++i) {
        const State s = static_cast<State>(i);
        if (s == flight_computer::ABORT_ON_GROUND || s == flight_computer::ABORT_IN_FLIGHT) {
            continue;
        }
        settle(s, 10000 * (i + 1));
        EXPECT_EQ(out[0], StatusLed::stateColour(s)) << "state " << i;
    }
}

TEST_F(StatusLedTest, ThreeEntryBlinksThenSteady) {
    in.av_state = flight_computer::ARMED;
    const Rgb yellow = StatusLed::stateColour(flight_computer::ARMED);
    const uint32_t t0 = 5000;
    at(t0);
    EXPECT_EQ(out[0], yellow);
    // 100 ms on, 100 ms off, three times, sampled every 20 ms.
    int rising_edges = 0;
    Rgb prev = out[0];
    for (uint32_t dt = 20; dt < 600; dt += 20) {
        at(t0 + dt);
        const bool on_slot = (dt / 100) % 2 == 0;
        EXPECT_EQ(out[0], on_slot ? yellow : kOff) << "dt " << dt;
        if (prev == kOff && out[0] == yellow) ++rising_edges;
        prev = out[0];
    }
    EXPECT_EQ(rising_edges, 2);  // on at entry, then two more rises: three blinks
    for (uint32_t dt = 600; dt < 3000; dt += 20) {
        at(t0 + dt);
        EXPECT_EQ(out[0], yellow) << "dt " << dt;
    }
}

TEST_F(StatusLedTest, EachStateChangeRestartsEntryBlink) {
    settle(flight_computer::FILLING, 1000);
    in.av_state = flight_computer::ARMED;
    at(2000);
    at(2100);
    EXPECT_EQ(out[0], kOff);
    at(2200);
    EXPECT_EQ(out[0], StatusLed::stateColour(flight_computer::ARMED));
}

TEST_F(StatusLedTest, AbortBlinksRedAtTwoHertz) {
    for (State s : {flight_computer::ABORT_ON_GROUND, flight_computer::ABORT_IN_FLIGHT}) {
        settle(flight_computer::ASCENT, 1000);
        in.av_state = s;
        const uint32_t t0 = 3000;
        at(t0);
        for (uint32_t dt = 0; dt < 5000; dt += 10) {
            at(t0 + dt);
            const bool on = (dt / 250) % 2 == 0;
            EXPECT_EQ(out[0], on ? kRed : kOff) << "state " << int(s) << " dt " << dt;
        }
    }
}

TEST_F(StatusLedTest, SensorLed) {
    settle(flight_computer::INIT, 1000);
    EXPECT_EQ(out[1], kGreen);
    in.sensors_ok = false;
    for (uint32_t t = 2000; t < 4000; t += 10) {
        at(t);
        EXPECT_EQ(out[1], (t / 250) % 2 == 0 ? kRed : kOff) << "t " << t;
    }
    in.sensors_ok = true;
    at(4000);
    EXPECT_EQ(out[1], kGreen);
}

TEST_F(StatusLedTest, SdLed) {
    settle(flight_computer::INIT, 1000);
    EXPECT_EQ(out[2], kGreen);
    in.sd_ok = false;
    EXPECT_TRUE(at(2000));
    EXPECT_EQ(out[2], kOff);
    EXPECT_FALSE(at(2500));
    EXPECT_EQ(out[2], kOff);
}

TEST_F(StatusLedTest, ErrorOverridesEverythingAtFourHertz) {
    settle(flight_computer::ABORT_IN_FLIGHT, 1000);
    in.sensors_ok = false;
    in.sd_ok = true;
    in.error = true;
    for (uint32_t t = 2000; t < 4000; t += 5) {
        at(t);
        const Rgb want = (t / 125) % 2 == 0 ? kRed : kOff;
        for (int i = 0; i < 3; ++i) {
            EXPECT_EQ(out[i], want) << "led " << i << " t " << t;
        }
    }
    in.error = false;
    in.sensors_ok = true;
    in.av_state = flight_computer::LANDED;
    settle(flight_computer::LANDED, 5000);
    EXPECT_EQ(out[0], StatusLed::stateColour(flight_computer::LANDED));
    EXPECT_EQ(out[1], kGreen);
    EXPECT_EQ(out[2], kGreen);
}

TEST_F(StatusLedTest, ReturnsTrueOnlyOnChange) {
    EXPECT_TRUE(at(1000));  // first call always reports
    at(1600);               // entry blink over
    EXPECT_FALSE(at(1620));
    EXPECT_FALSE(at(5000));
    in.sensors_ok = false;
    EXPECT_TRUE(at(5010));   // 5010/250 = 20, even: red
    EXPECT_FALSE(at(5100));  // still red
    EXPECT_TRUE(at(5250));   // off
}

// now_ms wraps at 2^32 ms (49.7 days). Times below are tracked in 64 bits and
// fed truncated, as HAL_GetTick() would deliver them.
TEST_F(StatusLedTest, SteadyStateAcrossTheWrapHasNoEntryBlink) {
    const uint64_t t0 = (1ull << 32) - 700;
    in.av_state = flight_computer::ARMED;
    const Rgb yellow = StatusLed::stateColour(flight_computer::ARMED);
    at(static_cast<uint32_t>(t0));
    at(static_cast<uint32_t>(t0 + 600));
    ASSERT_EQ(out[0], yellow);
    // Across the wrap itself, 20 ms steps.
    for (uint64_t t = t0 + 620; t < t0 + 10000; t += 20) {
        EXPECT_FALSE(at(static_cast<uint32_t>(t))) << "t " << t;
        EXPECT_EQ(out[0], yellow) << "t " << t;
    }
    // Long jumps up to 2^32 ms after the entry, then 20 ms steps through the
    // point where (now - entry) comes back small.
    for (uint64_t t = t0 + 10000; t < t0 + (1ull << 32) - 2000; t += 999983) {
        EXPECT_FALSE(at(static_cast<uint32_t>(t))) << "t " << t;
        EXPECT_EQ(out[0], yellow) << "t " << t;
    }
    for (uint64_t t = t0 + (1ull << 32) - 2000; t < t0 + (1ull << 32) + 2000; t += 20) {
        EXPECT_FALSE(at(static_cast<uint32_t>(t))) << "t " << t;
    }
}

TEST_F(StatusLedTest, BlinksKeepTheirPhaseAcrossTheWrap) {
    // Abort on LED 1 and failed sensors on LED 2, both 2 Hz from the entry.
    const uint64_t t0 = (1ull << 32) - 1003;
    in.av_state = flight_computer::ABORT_IN_FLIGHT;
    in.sensors_ok = false;
    auto check = [&](uint64_t t) {
        at(static_cast<uint32_t>(t));
        const Rgb want = ((t - t0) / 250) % 2 == 0 ? kRed : kOff;
        EXPECT_EQ(out[0], want) << "t " << t;
        EXPECT_EQ(out[1], want) << "t " << t;
    };
    for (uint64_t t = t0; t < t0 + 3000; ++t) {
        check(t);
    }
    // Long jumps, not multiples of the period, past a full 2^32 ms.
    for (uint64_t t = t0 + 3000; t < t0 + (1ull << 32) - 1500; t += 1234567) {
        check(t);
    }
    for (uint64_t t = t0 + (1ull << 32) - 1500; t < t0 + (1ull << 32) + 1500; ++t) {
        check(t);
    }
}

TEST_F(StatusLedTest, ErrorBlinkKeepsItsPhaseAcrossTheWrap) {
    const uint64_t t0 = (1ull << 32) - 333;
    in.av_state = flight_computer::FILLING;
    in.error = true;
    for (uint64_t t = t0; t < t0 + 2000; ++t) {
        at(static_cast<uint32_t>(t));
        const Rgb want = ((t - t0) / 125) % 2 == 0 ? kRed : kOff;
        for (int i = 0; i < 3; ++i) {
            EXPECT_EQ(out[i], want) << "led " << i << " t " << t;
        }
    }
}

// ── Interactions ───────────────────────────────────────────────────────────

namespace {
Rgb errorWant(uint32_t t, uint32_t origin) { return ((t - origin) / 125) % 2 == 0 ? kRed : kOff; }
} // namespace

TEST_F(StatusLedTest, StateChangeUnderErrorRunsEntryBlinkHidden) {
    settle(flight_computer::FILLING, 1000);
    in.error = true;
    at(2000);
    in.av_state = flight_computer::ARMED;  // blinks count from this entry
    for (uint32_t t = 2050; t < 2350; t += 5) {
        at(t);
        for (int i = 0; i < 3; ++i) {
            EXPECT_EQ(out[i], errorWant(t, 2050)) << "led " << i << " t " << t;
        }
    }
    // Error clears 300 ms into the entry blink: the rest of it shows.
    in.error = false;
    EXPECT_TRUE(at(2350));
    EXPECT_EQ(out[0], kOff);  // 300..400 is an off slot
    at(2450);
    EXPECT_EQ(out[0], StatusLed::stateColour(flight_computer::ARMED));
    at(2550);
    EXPECT_EQ(out[0], kOff);
    at(2650);
    EXPECT_EQ(out[0], StatusLed::stateColour(flight_computer::ARMED));
}

TEST_F(StatusLedTest, EntryBlinkConsumedUnderErrorIsNotReplayed) {
    settle(flight_computer::FILLING, 1000);
    in.error = true;
    in.av_state = flight_computer::ARMED;
    at(2000);
    at(2500);
    in.error = false;
    EXPECT_TRUE(at(2700));
    for (uint32_t t = 2700; t < 4000; t += 20) {
        at(t);
        EXPECT_EQ(out[0], StatusLed::stateColour(flight_computer::ARMED)) << "t " << t;
    }
}

TEST_F(StatusLedTest, ErrorStartingDuringEntryBlinkOverrides) {
    in.av_state = flight_computer::ARMED;
    at(1000);
    at(1050);
    EXPECT_EQ(out[0], StatusLed::stateColour(flight_computer::ARMED));
    in.error = true;
    for (uint32_t t = 1060; t < 2000; t += 5) {
        at(t);
        for (int i = 0; i < 3; ++i) {
            EXPECT_EQ(out[i], errorWant(t, 1000)) << "led " << i << " t " << t;
        }
    }
}

TEST_F(StatusLedTest, ErrorClearingAloneResumesAbortPhase) {
    in.av_state = flight_computer::ABORT_ON_GROUND;
    in.error = true;
    for (uint32_t t = 1000; t < 2000; t += 10) {
        at(t);
    }
    in.error = false;
    for (uint32_t t = 2010; t < 4000; t += 10) {
        at(t);
        EXPECT_EQ(out[0], ((t - 1000) / 250) % 2 == 0 ? kRed : kOff) << "t " << t;
        EXPECT_EQ(out[1], kGreen);
        EXPECT_EQ(out[2], kGreen);
    }
}

TEST_F(StatusLedTest, HiddenChangesUnderErrorReturnFalse) {
    in.error = true;
    at(1000);  // red slot 1000..1125
    in.sensors_ok = false;
    EXPECT_FALSE(at(1010));
    in.sd_ok = false;
    EXPECT_FALSE(at(1020));
    in.av_state = flight_computer::FILLING;  // new epoch at 1030, still red
    EXPECT_FALSE(at(1030));
    EXPECT_TRUE(at(1155));  // 125 ms after the new epoch: off
}

TEST_F(StatusLedTest, UnknownStateThroughUpdate) {
    const State unknown = static_cast<State>(200);
    in.av_state = unknown;
    at(1000);
    EXPECT_EQ(out[0], kRed);
    at(1100);
    EXPECT_EQ(out[0], kOff);  // entry blink, not the 2 Hz abort blink
    at(1200);
    EXPECT_EQ(out[0], kRed);
    at(1600);
    EXPECT_EQ(out[0], kRed);
    for (uint32_t t = 1620; t < 3000; t += 10) {
        EXPECT_FALSE(at(t)) << "t " << t;
        EXPECT_EQ(out[0], kRed);
    }
}

TEST_F(StatusLedTest, ReturnsTrueWhenOnlyLed1Changes) {
    in.av_state = flight_computer::ARMED;
    EXPECT_TRUE(at(1000));
    for (uint32_t t = 1020; t <= 1700; t += 20) {
        const bool edge = t <= 1600 && (t - 1000) % 100 == 0;
        EXPECT_EQ(at(t), edge) << "t " << t;
        EXPECT_EQ(out[1], kGreen);
        EXPECT_EQ(out[2], kGreen);
    }
}

TEST_F(StatusLedTest, AbortEnteredMidEntryBlink) {
    in.av_state = flight_computer::ASCENT;
    at(1000);
    at(1120);
    EXPECT_EQ(out[0], kOff);  // off slot of the ASCENT entry blink
    in.av_state = flight_computer::ABORT_IN_FLIGHT;
    EXPECT_TRUE(at(1150));
    EXPECT_EQ(out[0], kRed);
    for (uint32_t t = 1150; t < 3000; t += 10) {
        at(t);
        EXPECT_EQ(out[0], ((t - 1150) / 250) % 2 == 0 ? kRed : kOff) << "t " << t;
    }
}

TEST_F(StatusLedTest, AbortWithFailedSensorsBlinksInPhase) {
    // Both 2 Hz blinks count from the abort entry, so LED 1 and LED 2 flash
    // together rather than drifting apart.
    in.sensors_ok = false;
    settle(flight_computer::ASCENT, 1000);
    in.av_state = flight_computer::ABORT_IN_FLIGHT;
    for (uint32_t t = 1337; t < 4000; ++t) {
        at(t);
        const Rgb want = ((t - 1337) / 250) % 2 == 0 ? kRed : kOff;
        EXPECT_EQ(out[0], want) << "t " << t;
        EXPECT_EQ(out[1], want) << "t " << t;
    }
}
