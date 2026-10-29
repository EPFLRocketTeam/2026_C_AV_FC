#include "Application/FlightControl/fc_shell_guard.hpp"

#include <gtest/gtest.h>

#include <string>

using flight_computer::State;

// The review's truth table: system dfu and system reboot run only on the
// ground, never armed, pressurized, igniting or flying.
TEST(FcShellGuard, ResetAllowedOnlyInGroundStates) {
    struct Row {
        State state;
        bool allowed;
    };
    const Row rows[] = {
        {State::INIT, true},          {State::CALIBRATION, true},
        {State::FILLING, true},       {State::ARMED, false},
        {State::PRESSURIZATION, false}, {State::TRY_IGNITION, false},
        {State::IGNITION, false},     {State::BURN, false},
        {State::ASCENT, false},       {State::DESCENT, false},
        {State::LANDED, true},        {State::ABORT_ON_GROUND, true},
        {State::ABORT_IN_FLIGHT, false},
    };
    for (const Row& row : rows) {
        EXPECT_EQ(fc_shell::IsResetAllowed(row.state), row.allowed)
            << fc_shell::StateName(row.state);
    }
}

TEST(FcShellGuard, EveryStateHasItsName) {
    EXPECT_STREQ(fc_shell::StateName(State::INIT), "INIT");
    EXPECT_STREQ(fc_shell::StateName(State::CALIBRATION), "CALIBRATION");
    EXPECT_STREQ(fc_shell::StateName(State::FILLING), "FILLING");
    EXPECT_STREQ(fc_shell::StateName(State::ARMED), "ARMED");
    EXPECT_STREQ(fc_shell::StateName(State::PRESSURIZATION), "PRESSURIZATION");
    EXPECT_STREQ(fc_shell::StateName(State::TRY_IGNITION), "TRY_IGNITION");
    EXPECT_STREQ(fc_shell::StateName(State::IGNITION), "IGNITION");
    EXPECT_STREQ(fc_shell::StateName(State::BURN), "BURN");
    EXPECT_STREQ(fc_shell::StateName(State::ASCENT), "ASCENT");
    EXPECT_STREQ(fc_shell::StateName(State::DESCENT), "DESCENT");
    EXPECT_STREQ(fc_shell::StateName(State::LANDED), "LANDED");
    EXPECT_STREQ(fc_shell::StateName(State::ABORT_ON_GROUND), "ABORT_ON_GROUND");
    EXPECT_STREQ(fc_shell::StateName(State::ABORT_IN_FLIGHT), "ABORT_IN_FLIGHT");
    EXPECT_STREQ(fc_shell::StateName(static_cast<State>(200)), "UNKNOWN");
}
