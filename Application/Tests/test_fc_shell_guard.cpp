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

using Command = fc_shell::ConfirmWindow::Command;

TEST(FcShellConfirm, SecondCallInsideTheWindowConfirms) {
    fc_shell::ConfirmWindow w;
    EXPECT_FALSE(w.Request(Command::kDfu, 1000));
    EXPECT_TRUE(w.Request(Command::kDfu, 1000 + 9999));
    // Confirming consumes the window: the next call arms again.
    EXPECT_FALSE(w.Request(Command::kDfu, 12000));
}

TEST(FcShellConfirm, WindowOfTenSecondsRunsOut) {
    fc_shell::ConfirmWindow w;
    EXPECT_FALSE(w.Request(Command::kReboot, 0));
    EXPECT_FALSE(w.Request(Command::kReboot, 10000));  // too late: re-armed
    EXPECT_TRUE(w.Request(Command::kReboot, 10001));
}

TEST(FcShellConfirm, OtherCommandInBetweenRearms) {
    fc_shell::ConfirmWindow w;
    EXPECT_FALSE(w.Request(Command::kDfu, 0));
    EXPECT_FALSE(w.Request(Command::kReboot, 100));
    EXPECT_FALSE(w.Request(Command::kDfu, 200));
    EXPECT_TRUE(w.Request(Command::kDfu, 300));
}

TEST(FcShellConfirm, DisarmCancelsTheWindow) {
    fc_shell::ConfirmWindow w;
    EXPECT_FALSE(w.Request(Command::kDfu, 0));
    w.Disarm();
    EXPECT_FALSE(w.Request(Command::kDfu, 1));
    EXPECT_TRUE(w.Request(Command::kDfu, 2));
}

TEST(FcShellConfirm, TickWrapAroundIsHandled) {
    fc_shell::ConfirmWindow w;
    EXPECT_FALSE(w.Request(Command::kDfu, 0xFFFFF000u));
    EXPECT_TRUE(w.Request(Command::kDfu, 0x00000500u));
}
