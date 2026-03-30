#include "Application/FlightControl/av_state.h"
#include "Application/FlightControl/threshold.h"

#include <gtest/gtest.h>

using namespace flight_computer;

// =============================================================================
// Command-ID convention (matches av_state.cpp)
//   0  – no command / neutral  → stay in current state
//   1  – ABORT                 → transition to abort state
//   2  – PROCEED               → advance to next state
//   3  – RESET                 → recover from abort back to INIT
// =============================================================================

// Returns a DataDump that does NOT trigger any transition (no command sent).
static DataDump neutralDump() {
  DataDump d{};
  d.uplinkCmd.id = 0;
  return d;
}

// Returns a DataDump with all conditions satisfied to leave PRESSURIZATION.
static DataDump validIgnitionDump() {
  DataDump d = neutralDump();
  d.event.timer_launch_delay    = true;
  // Instantaneous pressures must stay below the safety check limit.
  d.propSensors.N2_pressure     = PRESSURIZATION_CHECK_PRESSURE - 5.0;
  d.propSensors.fuel_pressure   = PRESSURIZATION_CHECK_PRESSURE - 5.0;
  d.propSensors.LOX_pressure    = PRESSURIZATION_CHECK_PRESSURE - 5.0;
  // Mean pressures must be inside the valid pressurisation band.
  const double mid = (PRESSURE_LOWER + PRESSURE_UPPER) / 2.0;
  d.propSensors.N2_pressure_mean   = mid;
  d.propSensors.fuel_pressure_mean = mid;
  d.propSensors.LOX_pressure_mean  = mid;
  return d;
}

// =============================================================================
// Advance helpers – each asserts the expected intermediate state so that a
// broken SetUp path fails clearly rather than silently in the wrong test.
// =============================================================================

static void advanceToCalibration(AvState &fsm) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 2; // PROCEED
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::CALIBRATION);
}

static void advanceToFilling(AvState &fsm) {
  advanceToCalibration(fsm);
  DataDump d = neutralDump();
  d.event.calibrated = true;
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::FILLING);
}

static void advanceToArmed(AvState &fsm) {
  advanceToFilling(fsm);
  DataDump d = neutralDump();
  d.uplinkCmd.id = 2; // PROCEED
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::ARMED);
}

static void advanceToPressurization(AvState &fsm) {
  advanceToArmed(fsm);
  DataDump d = neutralDump();
  d.uplinkCmd.id = 2; // PROCEED
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::PRESSURIZATION);
}

static void advanceToIgnition(AvState &fsm) {
  advanceToPressurization(fsm);
  fsm.update(validIgnitionDump());
  ASSERT_EQ(fsm.getCurrentState(), State::IGNITION);
}

static void advanceToBurn(AvState &fsm) {
  advanceToIgnition(fsm);
  DataDump d = neutralDump();
  d.vehiculeOverview.no_cable_continuity = 1; // umbilical disconnected → liftoff
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::BURN);
}

static void advanceToAscent(AvState &fsm) {
  advanceToBurn(fsm);
  DataDump d = neutralDump();
  d.event.cut_off_detected = true;
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::ASCENT);
}

static void advanceToDescent(AvState &fsm) {
  advanceToAscent(fsm);
  DataDump d = neutralDump();
  d.event.apogee_detected = true;
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::DESCENT);
}

static void advanceToLanded(AvState &fsm) {
  advanceToDescent(fsm);
  DataDump d = neutralDump();
  d.event.touchdown_detected        = true;
  d.flightEventTimers.descent_duration = static_cast<uint32_t>(DESCENT_THRESHOLD_DURATION) + 1;
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::LANDED);
}

static void advanceToAbortOnGround(AvState &fsm) {
  advanceToCalibration(fsm);
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::ABORT_ON_GROUND);
}

static void advanceToAbortInFlight(AvState &fsm) {
  advanceToBurn(fsm);
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm.update(d);
  ASSERT_EQ(fsm.getCurrentState(), State::ABORT_IN_FLIGHT);
}

// =============================================================================
// Initial state
// =============================================================================

TEST(AvStateInit, StartsInInitState) {
  AvState fsm;
  EXPECT_EQ(fsm.getCurrentState(), State::INIT);
}

// =============================================================================
// INIT transitions
// =============================================================================

TEST(AvStateFromInit, TransitionsToCalibrationWhenProceedCmdReceived) {
  AvState fsm;
  DataDump d = neutralDump();
  d.uplinkCmd.id = 2; // PROCEED
  fsm.update(d);
  EXPECT_EQ(fsm.getCurrentState(), State::CALIBRATION);
}

TEST(AvStateFromInit, StaysInInitOnNeutralDump) {
  AvState fsm;
  fsm.update(neutralDump());
  EXPECT_EQ(fsm.getCurrentState(), State::INIT);
}

// =============================================================================
// CALIBRATION transitions
// =============================================================================

class AvStateCalibrationTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToCalibration(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateCalibrationTest, AbortOnGroundWhenAbortCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStateCalibrationTest, TransitionsToFillingWhenCalibrated) {
  DataDump d = neutralDump();
  d.event.calibrated = true;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::FILLING);
}

TEST_F(AvStateCalibrationTest, StaysInCalibrationWhenNotCalibrated) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::CALIBRATION);
}

// =============================================================================
// FILLING transitions
// =============================================================================

class AvStateFillingTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToFilling(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateFillingTest, AbortOnGroundWhenAbortCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStateFillingTest, TransitionsToArmedWhenProceedCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 2; // PROCEED
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ARMED);
}

TEST_F(AvStateFillingTest, StaysInFillingOnNeutralDump) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::FILLING);
}

// =============================================================================
// ARMED transitions
// =============================================================================

class AvStateArmedTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToArmed(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateArmedTest, TransitionsToPressurization) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 2; // PROCEED
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::PRESSURIZATION);
}

TEST_F(AvStateArmedTest, AbortOnGroundWhenAbortCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStateArmedTest, StaysInArmedOnNeutralDump) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::ARMED);
}

// =============================================================================
// PRESSURIZATION transitions
// =============================================================================

class AvStatePressurization : public ::testing::Test {
protected:
  void SetUp() override { advanceToPressurization(fsm_); }
  AvState fsm_;
};

TEST_F(AvStatePressurization, TransitionsToIgnitionWhenPressureOkAndTimerReached) {
  fsm_.update(validIgnitionDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::IGNITION);
}

TEST_F(AvStatePressurization, AbortOnGroundWhenAbortCmdReceived) {
  DataDump d = validIgnitionDump();
  d.uplinkCmd.id = 1; // ABORT overrides even valid pressures
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStatePressurization, AbortOnGroundWhenN2InstantPressureTooHigh) {
  DataDump d = neutralDump();
  d.propSensors.N2_pressure = PRESSURIZATION_CHECK_PRESSURE + 1.0;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStatePressurization, AbortOnGroundWhenFuelInstantPressureTooHigh) {
  DataDump d = neutralDump();
  d.propSensors.fuel_pressure = PRESSURIZATION_CHECK_PRESSURE + 1.0;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStatePressurization, AbortOnGroundWhenLOXInstantPressureTooHigh) {
  DataDump d = neutralDump();
  d.propSensors.LOX_pressure = PRESSURIZATION_CHECK_PRESSURE + 1.0;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStatePressurization, StaysWhenTimerNotReachedYet) {
  DataDump d = validIgnitionDump();
  d.event.timer_launch_delay = false;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::PRESSURIZATION);
}

TEST_F(AvStatePressurization, StaysWhenMeanPressureAboveBand) {
  DataDump d = validIgnitionDump();
  d.propSensors.fuel_pressure_mean = PRESSURE_UPPER + 1.0;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::PRESSURIZATION);
}

TEST_F(AvStatePressurization, StaysWhenMeanPressureBelowBand) {
  DataDump d = validIgnitionDump();
  d.propSensors.LOX_pressure_mean = PRESSURE_LOWER - 1.0;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::PRESSURIZATION);
}

// =============================================================================
// IGNITION transitions
// =============================================================================

class AvStateIgnitionTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToIgnition(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateIgnitionTest, TransitionsToBurnWhenCableDisconnected) {
  DataDump d = neutralDump();
  d.vehiculeOverview.no_cable_continuity = 1;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::BURN);
}

TEST_F(AvStateIgnitionTest, TransitionsToBurnWhenVerticalAccHoldDetected) {
  DataDump d = neutralDump();
  d.event.vertical_acc_hold = true;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::BURN);
}

TEST_F(AvStateIgnitionTest, AbortOnGroundWhenAbortCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStateIgnitionTest, AbortOnGroundWhenNeitherCableNorAccDetected) {
  // Default neutral dump has no_cable_continuity=0 and vertical_acc_hold=false
  // → engine did not ignite, safety abort.
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

// =============================================================================
// BURN transitions
// =============================================================================

class AvStateBurnTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToBurn(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateBurnTest, TransitionsToAscentWhenCutOffDetected) {
  DataDump d = neutralDump();
  d.event.cut_off_detected = true;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ASCENT);
}

TEST_F(AvStateBurnTest, TransitionsToAscentWhenBurnTimerExceededMaxDuration) {
  DataDump d = neutralDump();
  d.propSensors.timer_burn = static_cast<uint32_t>(BURN_MAX_DURATION) + 1;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ASCENT);
}

TEST_F(AvStateBurnTest, AbortInFlightWhenAbortCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_IN_FLIGHT);
}

TEST_F(AvStateBurnTest, StaysInBurnOnNeutralDump) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::BURN);
}

// =============================================================================
// ASCENT transitions
// =============================================================================

class AvStateAscentTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToAscent(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateAscentTest, TransitionsToDescentWhenApogeeDetected) {
  DataDump d = neutralDump();
  d.event.apogee_detected = true;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::DESCENT);
}

TEST_F(AvStateAscentTest, TransitionsToDescentWhenFlightTimerExceedsAscentMax) {
  DataDump d = neutralDump();
  d.flightEventTimers.flight_duration = static_cast<uint32_t>(ASCENT_MAX_DURATION) + 1;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::DESCENT);
}

TEST_F(AvStateAscentTest, AbortInFlightWhenAbortCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_IN_FLIGHT);
}

TEST_F(AvStateAscentTest, StaysInAscentOnNeutralDump) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::ASCENT);
}

// =============================================================================
// DESCENT transitions
// =============================================================================

class AvStateDescentTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToDescent(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateDescentTest, TransitionsToLandedWhenTouchdownAndDurationOk) {
  DataDump d = neutralDump();
  d.event.touchdown_detected        = true;
  d.flightEventTimers.descent_duration = static_cast<uint32_t>(DESCENT_THRESHOLD_DURATION) + 1;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::LANDED);
}

TEST_F(AvStateDescentTest, StaysInDescentWhenTouchdownButDurationTooShort) {
  DataDump d = neutralDump();
  d.event.touchdown_detected        = true;
  d.flightEventTimers.descent_duration = 0; // below threshold
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::DESCENT);
}

TEST_F(AvStateDescentTest, StaysInDescentWhenDurationOkButNoTouchdown) {
  DataDump d = neutralDump();
  d.flightEventTimers.descent_duration = static_cast<uint32_t>(DESCENT_THRESHOLD_DURATION) + 1;
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::DESCENT);
}

TEST_F(AvStateDescentTest, AbortInFlightWhenAbortCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_IN_FLIGHT);
}

TEST_F(AvStateDescentTest, StaysInDescentOnNeutralDump) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::DESCENT);
}

// =============================================================================
// LANDED transitions
// =============================================================================

class AvStateLandedTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToLanded(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateLandedTest, AbortOnGroundWhenAbortCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 1; // ABORT
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

TEST_F(AvStateLandedTest, StaysInLandedOnNeutralDump) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::LANDED);
}

// =============================================================================
// ABORT_ON_GROUND transitions
// =============================================================================

class AvStateAbortOnGroundTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToAbortOnGround(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateAbortOnGroundTest, TransitionsToInitWhenResetCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 3; // RESET
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::INIT);
}

TEST_F(AvStateAbortOnGroundTest, StaysInAbortOnGroundOnNeutralDump) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_ON_GROUND);
}

// =============================================================================
// ABORT_IN_FLIGHT transitions
// =============================================================================

class AvStateAbortInFlightTest : public ::testing::Test {
protected:
  void SetUp() override { advanceToAbortInFlight(fsm_); }
  AvState fsm_;
};

TEST_F(AvStateAbortInFlightTest, TransitionsToInitWhenResetCmdReceived) {
  DataDump d = neutralDump();
  d.uplinkCmd.id = 3; // RESET
  fsm_.update(d);
  EXPECT_EQ(fsm_.getCurrentState(), State::INIT);
}

TEST_F(AvStateAbortInFlightTest, StaysInAbortInFlightOnNeutralDump) {
  fsm_.update(neutralDump());
  EXPECT_EQ(fsm_.getCurrentState(), State::ABORT_IN_FLIGHT);
}

// =============================================================================
// stateToString
// =============================================================================

TEST(AvStateToString, ReturnsCorrectStringForEveryState) {
  AvState fsm;
  EXPECT_EQ(fsm.stateToString(State::INIT),           "INIT");
  EXPECT_EQ(fsm.stateToString(State::CALIBRATION),    "CALIBRATION");
  EXPECT_EQ(fsm.stateToString(State::FILLING),        "FILLING");
  EXPECT_EQ(fsm.stateToString(State::ARMED),          "ARMED");
  EXPECT_EQ(fsm.stateToString(State::PRESSURIZATION), "PRESSURIZATION");
  EXPECT_EQ(fsm.stateToString(State::IGNITION),       "IGNITION");
  EXPECT_EQ(fsm.stateToString(State::BURN),           "BURN");
  EXPECT_EQ(fsm.stateToString(State::ASCENT),         "ASCENT");
  EXPECT_EQ(fsm.stateToString(State::DESCENT),        "DESCENT");
  EXPECT_EQ(fsm.stateToString(State::LANDED),         "LANDED");
  EXPECT_EQ(fsm.stateToString(State::ABORT_ON_GROUND),  "ABORT_ON_GROUND");
  EXPECT_EQ(fsm.stateToString(State::ABORT_IN_FLIGHT),  "ABORT_IN_FLIGHT");
}

// =============================================================================
// Integration chains
// =============================================================================

TEST(AvStateChain, FullHappyPathFromInitToLanded) {
  AvState fsm;

  { DataDump d = neutralDump(); d.uplinkCmd.id = 2; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::CALIBRATION);

  { DataDump d = neutralDump(); d.event.calibrated = true; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::FILLING);

  { DataDump d = neutralDump(); d.uplinkCmd.id = 2; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::ARMED);

  { DataDump d = neutralDump(); d.uplinkCmd.id = 2; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::PRESSURIZATION);

  fsm.update(validIgnitionDump());
  EXPECT_EQ(fsm.getCurrentState(), State::IGNITION);

  { DataDump d = neutralDump(); d.vehiculeOverview.no_cable_continuity = 1; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::BURN);

  { DataDump d = neutralDump(); d.event.cut_off_detected = true; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::ASCENT);

  { DataDump d = neutralDump(); d.event.apogee_detected = true; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::DESCENT);

  {
    DataDump d = neutralDump();
    d.event.touchdown_detected        = true;
    d.flightEventTimers.descent_duration = static_cast<uint32_t>(DESCENT_THRESHOLD_DURATION) + 1;
    fsm.update(d);
  }
  EXPECT_EQ(fsm.getCurrentState(), State::LANDED);
}

TEST(AvStateChain, AbortInFlightAndResetToInit) {
  AvState fsm;
  advanceToBurn(fsm);

  { DataDump d = neutralDump(); d.uplinkCmd.id = 1; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::ABORT_IN_FLIGHT);

  { DataDump d = neutralDump(); d.uplinkCmd.id = 3; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::INIT);
}

TEST(AvStateChain, AbortOnGroundAndResetToInit) {
  AvState fsm;
  advanceToAbortOnGround(fsm);

  { DataDump d = neutralDump(); d.uplinkCmd.id = 3; fsm.update(d); }
  EXPECT_EQ(fsm.getCurrentState(), State::INIT);
}
