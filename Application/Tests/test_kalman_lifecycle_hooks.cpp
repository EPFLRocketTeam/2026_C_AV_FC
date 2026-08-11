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

    // Keep instantaneous LOX/Fuel values below abort threshold (N2/COPV
    // isn't part of that check -- see fromPressurization()).
    d.propSensors.fuel_pressure = PRESSURIZATION_MAX_CRITICAL_PRESSURE - 5.0;
    d.propSensors.LOX_pressure = PRESSURIZATION_MAX_CRITICAL_PRESSURE - 5.0;
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

TEST(KalmanLifecycleHooks, LiftoffIsOneShotUntilResetToInit) {
    kalman_reset_lifecycle();

    kalman_on_liftoff(100);
    kalman_on_liftoff(200);

    uint32_t liftoff_ms = 0;
    EXPECT_EQ(kalman_take_pending_liftoff(&liftoff_ms), 1u);
    EXPECT_EQ(liftoff_ms, 100u);
    EXPECT_EQ(kalman_take_pending_liftoff(&liftoff_ms), 0u);

    // Re-arm by returning to INIT state.
    kalman_on_state_change(static_cast<uint32_t>(State::INIT));
    kalman_on_liftoff(300);

    EXPECT_EQ(kalman_take_pending_liftoff(&liftoff_ms), 1u);
    EXPECT_EQ(liftoff_ms, 300u);
}

TEST(KalmanLifecycleHooks, InitTransitionClearsStickyEventFlags) {
    kalman_reset_lifecycle();

    auto &goat = GOATStore::get_instance();
    Event event = goat.eventStore.get();
    event.catastrophic_failure = true;
    event.apogee_detected = true;
    goat.eventStore.set(event);

    kalman_on_state_change(static_cast<uint32_t>(State::ASCENT));
    kalman_on_state_change(static_cast<uint32_t>(State::INIT));

    const Event cleared = goat.eventStore.get();
    EXPECT_FALSE(cleared.catastrophic_failure);
    EXPECT_FALSE(cleared.apogee_detected);
}

TEST(KalmanLifecycleHooks, StateHookIsIdempotentForSameValue) {
    kalman_reset_lifecycle();

    kalman_on_state_change(static_cast<uint32_t>(State::INIT));
    EXPECT_EQ(kalman_current_state(), static_cast<uint32_t>(State::INIT));

    kalman_on_state_change(static_cast<uint32_t>(State::CALIBRATION));
    EXPECT_EQ(kalman_current_state(), static_cast<uint32_t>(State::CALIBRATION));

    kalman_on_state_change(static_cast<uint32_t>(State::CALIBRATION));
    EXPECT_EQ(kalman_current_state(), static_cast<uint32_t>(State::CALIBRATION));
}
