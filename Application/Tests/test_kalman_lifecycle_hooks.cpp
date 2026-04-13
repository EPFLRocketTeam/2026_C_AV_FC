#include "Application/Data/data.hpp"
#include "Application/Data/fsm.hpp"
#include "Application/FlightControl/av_state.h"
#include "Application/FlightControl/threshold.h"
#include "Application/Kalman/kalman_lifecycle.h"

#include <gtest/gtest.h>

using namespace flight_computer;

namespace {

DataDump neutralDump() {
    DataDump d{};
    d.uplinkCmd.id = 0;
    return d;
}

DataDump ignitionReadyDump() {
    DataDump d = neutralDump();
    d.event.timer_launch_delay = true;

    // Keep means inside the accepted pressurization window.
    const double mid = (PRESSURE_LOWER + PRESSURE_UPPER) / 2.0;
    d.propSensors.N2_pressure_mean = mid;
    d.propSensors.fuel_pressure_mean = mid;
    d.propSensors.LOX_pressure_mean = mid;

    // Keep instantaneous values below abort threshold.
    d.propSensors.N2_pressure = PRESSURIZATION_CHECK_PRESSURE - 5.0;
    d.propSensors.fuel_pressure = PRESSURIZATION_CHECK_PRESSURE - 5.0;
    d.propSensors.LOX_pressure = PRESSURIZATION_CHECK_PRESSURE - 5.0;
    return d;
}

void advanceToIgnition(AvState &fsm) {
    DataDump d = neutralDump();

    // INIT -> CALIBRATION
    d.uplinkCmd.id = 2;
    fsm.update(d);
    ASSERT_EQ(fsm.getCurrentState(), State::CALIBRATION);

    // CALIBRATION -> FILLING
    d = neutralDump();
    d.event.calibrated = true;
    fsm.update(d);
    ASSERT_EQ(fsm.getCurrentState(), State::FILLING);

    // FILLING -> ARMED
    d = neutralDump();
    d.uplinkCmd.id = 2;
    fsm.update(d);
    ASSERT_EQ(fsm.getCurrentState(), State::ARMED);

    // ARMED -> PRESSURIZATION
    d = neutralDump();
    d.uplinkCmd.id = 2;
    fsm.update(d);
    ASSERT_EQ(fsm.getCurrentState(), State::PRESSURIZATION);

    // PRESSURIZATION -> IGNITION
    d = ignitionReadyDump();
    fsm.update(d);
    ASSERT_EQ(fsm.getCurrentState(), State::IGNITION);
}

}  // namespace

TEST(KalmanLifecycleHooks, LiftoffHookArmsOnBurnTransition) {
    AvState fsm;
    kalman_reset_lifecycle();

    advanceToIgnition(fsm);

    DataDump burn = neutralDump();
    burn.vehiculeOverview.no_cable_continuity = true;
    burn.av_timestamp = 4242;
    fsm.update(burn);

    ASSERT_EQ(fsm.getCurrentState(), State::BURN);
    EXPECT_EQ(kalman_current_state(), static_cast<uint32_t>(State::BURN));

    uint32_t liftoff_ms = 0;
    EXPECT_EQ(kalman_take_pending_liftoff(&liftoff_ms), 1u);
    EXPECT_EQ(liftoff_ms, 4242u);
    EXPECT_EQ(kalman_take_pending_liftoff(&liftoff_ms), 0u);
}

TEST(KalmanLifecycleHooks, StateHookTracksTransitionsAfterLiftoff) {
    AvState fsm;
    kalman_reset_lifecycle();

    advanceToIgnition(fsm);

    DataDump burn = neutralDump();
    burn.vehiculeOverview.no_cable_continuity = true;
    burn.av_timestamp = 1000;
    fsm.update(burn);
    ASSERT_EQ(fsm.getCurrentState(), State::BURN);

    DataDump ascent = neutralDump();
    ascent.event.cut_off_detected = true;
    fsm.update(ascent);

    ASSERT_EQ(fsm.getCurrentState(), State::ASCENT);
    EXPECT_EQ(kalman_current_state(), static_cast<uint32_t>(State::ASCENT));
}
