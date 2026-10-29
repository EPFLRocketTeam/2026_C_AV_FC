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

TEST_F(StatusLedTest, NextDueNeverUnderTwentyMsAhead) {
    // Sweep every pattern at 1 ms steps: due stays at least 20 ms ahead of the
    // last update, and never further than one blink half period, so a 20 ms
    // tick neither has to run faster nor misses a blink.
    struct Case {
        State s;
        bool sensors_ok;
        bool error;
    } cases[] = {
        {flight_computer::ARMED, true, false},
        {flight_computer::ABORT_ON_GROUND, true, false},
        {flight_computer::INIT, false, false},
        {flight_computer::BURN, true, true},
        {flight_computer::ABORT_IN_FLIGHT, false, false},
    };
    for (const auto& c : cases) {
        led = StatusLed{};
        in = {c.s, c.sensors_ok, true, c.error, 7003};
        for (uint32_t t = 7003; t < 9000; ++t) {
            at(t);
            const uint32_t due = led.nextDueMs();
            EXPECT_GE(due, t + StatusLed::kMinUpdateMs) << "t " << t;
            EXPECT_LE(due, t + StatusLed::kAbortHalfMs) << "t " << t;
        }
    }
}

TEST_F(StatusLedTest, NextDueLandsOnTheBlinkEdge) {
    in.av_state = flight_computer::ARMED;
    at(1000);
    EXPECT_EQ(led.nextDueMs(), 1100u);  // first entry blink turns off
    at(1090);
    EXPECT_EQ(led.nextDueMs(), 1110u);  // edge at 1100 is under 20 ms away
}

TEST_F(StatusLedTest, NextDueIsPollRateWhenSteady) {
    at(1000);
    at(1600);
    EXPECT_EQ(led.nextDueMs(), 1620u);
}
