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

// Truth table of the two step reset: what system confirm does after each
// sequence. The second step is always a different command than the first.
TEST(FcShellConfirm, ConfirmActsOnWhatWasArmedWithinTheWindow) {
    fc_shell::ConfirmWindow w;
    w.Arm(Command::kDfu, 1000);
    EXPECT_EQ(w.Confirm(1000 + 9999), Command::kDfu);
    // Confirming consumes the window.
    EXPECT_EQ(w.Confirm(1000 + 9999), Command::kNone);

    w.Arm(Command::kReboot, 5000);
    EXPECT_EQ(w.Confirm(5001), Command::kReboot);
}

TEST(FcShellConfirm, ConfirmWithNothingArmedDoesNothing) {
    fc_shell::ConfirmWindow w;
    EXPECT_EQ(w.Confirm(0), Command::kNone);
    EXPECT_EQ(w.Confirm(123456), Command::kNone);
}

TEST(FcShellConfirm, WindowOfTenSecondsRunsOut) {
    fc_shell::ConfirmWindow w;
    w.Arm(Command::kDfu, 0);
    EXPECT_EQ(w.Confirm(10000), Command::kNone);
    // A late confirm also disarms: confirming again later does nothing.
    EXPECT_EQ(w.Confirm(10001), Command::kNone);
}

// A reflex double entry (system dfu typed twice, or up arrow and Enter)
// only re-arms; nothing happens without system confirm.
TEST(FcShellConfirm, RepeatingTheCommandOnlyRearms) {
    fc_shell::ConfirmWindow w;
    w.Arm(Command::kDfu, 0);
    w.Arm(Command::kDfu, 9000);  // the repeat restarts the window
    EXPECT_EQ(w.Confirm(18000), Command::kDfu);
}

TEST(FcShellConfirm, LastArmedCommandWins) {
    fc_shell::ConfirmWindow w;
    w.Arm(Command::kDfu, 0);
    w.Arm(Command::kReboot, 100);
    EXPECT_EQ(w.Confirm(200), Command::kReboot);
}

TEST(FcShellConfirm, DisarmCancelsTheWindow) {
    fc_shell::ConfirmWindow w;
    w.Arm(Command::kDfu, 0);
    w.Disarm();
    EXPECT_EQ(w.Confirm(1), Command::kNone);
}

TEST(FcShellConfirm, TickWrapAroundIsHandled) {
    fc_shell::ConfirmWindow w;
    w.Arm(Command::kDfu, 0xFFFFF000u);
    EXPECT_EQ(w.Confirm(0x00000500u), Command::kDfu);
}
