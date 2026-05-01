#include "av_state.h"
#include "Application/Data/data.hpp"
#include "Application/FlightControl/threshold.h"
#include "Application/Kalman/kalman_lifecycle.h"
#include <iostream>

AvState::AvState() { this->currentState = State::INIT; }

// Destructor
AvState::~AvState() {
  // Nothing to do
}

// This function allows to get the current state of the FSM
State AvState::getCurrentState() { return currentState; }

State AvState::fromInit(DataDump const &dump) {
  if (dump.uplinkCmd.id ==
      2) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition INIT->CALIBRATION");
    return State::CALIBRATION;
  }
  return currentState;
}

State AvState::fromCalibration(DataDump const &dump) {
  if (dump.uplinkCmd.id ==
      1) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.event.calibrated) {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::FILLING;
  }
  return currentState;
}

State AvState::fromFilling(DataDump const &dump) {
  if (dump.uplinkCmd.id == 1)
  // TODO: replace this with proper cmd
  // id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.uplinkCmd.id ==
           2) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::ARMED;
  }
  return currentState;
}

State AvState::fromArmed(DataDump const &dump) {
  if (dump.uplinkCmd.id ==
      1) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.uplinkCmd.id ==
           2) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::PRESSURIZATION;
  }
  return currentState;
}

State AvState::fromPressurization(DataDump const &dump) {
  if (dump.uplinkCmd.id == 1 ||
      PRESSURIZATION_CHECK_PRESSURE < dump.propSensors.N2_pressure ||
      PRESSURIZATION_CHECK_PRESSURE < dump.propSensors.fuel_pressure ||
      PRESSURIZATION_CHECK_PRESSURE < dump.propSensors.LOX_pressure)
  // TODO: replace this with proper cmd id from the
  // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  } else if (dump.event.timer_launch_delay &&
             dump.propSensors.fuel_pressure_mean < PRESSURE_UPPER &&
             dump.propSensors.LOX_pressure_mean < PRESSURE_UPPER &&
             dump.propSensors.N2_pressure_mean < PRESSURE_UPPER &&
             dump.propSensors.fuel_pressure_mean > PRESSURE_LOWER &&
             dump.propSensors.LOX_pressure_mean > PRESSURE_LOWER &&
             dump.propSensors.N2_pressure_mean > PRESSURE_LOWER) {
    // Logger::log_eventf("FSM transition PRESSURIZATION->INGITION");
    return State::IGNITION;
  }
  return currentState;
}

State AvState::fromIgnition(DataDump const &dump) {
  if (dump.event.catastrophic_failure) {
    return State::ABORT_IN_FLIGHT;
  }

  if (dump.uplinkCmd.id == 1) { // ABORT command
    return State::ABORT_ON_GROUND;
  }

  // --- Liftoff detection (DONE: tri-state vertical_acc_hold) ---
  // The cable is always authoritative: if the umbilical disconnects,
  // the rocket has left the pad regardless of the IMU evaluation.
  if (dump.vehiculeOverview.no_cable_continuity) {
    return State::BURN;
  }

  // IMU-based liftoff: the Kalman subsystem evaluates whether the mean
  // vertical acceleration exceeded ACCEL_LIFTOFF for ACCEL_LIFTOFF_DURATION_MS
  // after entering IGNITION (post-RAMP_UP).  Three possible values:
  //   ACC_HOLD_NOT_ELAPSED     – window still running → stay in IGNITION.
  //   ACC_HOLD_DID_HOLD        – confirmed liftoff    → transition to BURN.
  //   ACC_HOLD_DID_NOT_HOLD    – motor failed         → abort on ground.
  if (dump.event.vertical_acc_hold == ACC_HOLD_DID_HOLD) {
    return State::BURN;
  }

  if (dump.event.vertical_acc_hold == ACC_HOLD_DID_NOT_HOLD) {
    return State::ABORT_ON_GROUND;
  }

  // ACC_HOLD_NOT_ELAPSED – evaluation still in progress.
  return currentState;
}

State AvState::fromBurn(DataDump const &dump) {
  if (dump.event.catastrophic_failure) {
    return State::ABORT_IN_FLIGHT;
  }

  if (dump.uplinkCmd.id == 1) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_IN_FLIGHT;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.propSensors.timer_burn > BURN_MAX_DURATION ||
           dump.event.cut_off_detected) {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::ASCENT;
  }
  return currentState;
}

State AvState::fromAscent(DataDump const &dump) {
  if (dump.event.catastrophic_failure) {
    return State::ABORT_IN_FLIGHT;
  }

  if (dump.uplinkCmd.id == 1) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_IN_FLIGHT;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.event.apogee_detected ||
           dump.flightEventTimers.flight_duration > ASCENT_MAX_DURATION) {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::DESCENT;
  }
  return currentState;
}

State AvState::fromDescent(DataDump const &dump) {
  if (dump.event.catastrophic_failure) {
    return State::ABORT_IN_FLIGHT;
  }

  if (dump.uplinkCmd.id == 1) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_IN_FLIGHT;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.event.touchdown_detected &&
           dump.flightEventTimers.descent_duration > DESCENT_THRESHOLD_DURATION) {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::LANDED;
  }
  return currentState;
}

State AvState::fromLanded(DataDump const &dump) {
  if (dump.uplinkCmd.id == 1) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  return currentState;
}

State AvState::fromAbortOnGround(DataDump const &dump) {
  if (dump.uplinkCmd.id == 3) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::INIT;
  }
  return currentState;
}

State AvState::fromAbortInFlight(DataDump const &dump) {
  if (dump.uplinkCmd.id == 3) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::INIT;
  }
  return currentState;
}

void AvState::update(const DataDump &dump) {
  const State previous_state = currentState;

  switch (currentState) {
  case State::INIT:
    currentState = fromInit(dump);
    break;
  case State::CALIBRATION:
    currentState = fromCalibration(dump);
    break;
  case State::FILLING:
    currentState = fromFilling(dump);
    break;
  case State::ARMED:
    currentState = fromArmed(dump);
    break;
  case State::PRESSURIZATION:
    currentState = fromPressurization(dump);
    break;
  case State::IGNITION:
    currentState = fromIgnition(dump);
    break;
  case State::BURN:
    currentState = fromBurn(dump);
    break;
  case State::ASCENT:
    currentState = fromAscent(dump);
    break;
  case State::DESCENT:
    currentState = fromDescent(dump);
    break;
  case State::LANDED:
    currentState = fromLanded(dump);
    break;
  case State::ABORT_ON_GROUND:
    currentState = fromAbortOnGround(dump);
    break;
  case State::ABORT_IN_FLIGHT:
    currentState = fromAbortInFlight(dump);
    break;
  default:
    currentState = State::ABORT_ON_GROUND;
  }

  if (currentState != previous_state) {
    kalman_on_state_change(static_cast<uint32_t>(currentState));

    // Liftoff contract: first in-flight state reached by this FSM is BURN.
    if (currentState == State::BURN) {
      kalman_on_liftoff(dump.av_timestamp);
    }
  }
}
std::string AvState::stateToString(State state) {
  switch (state) {
  case State::INIT:
    return "INIT";
    break;
  case State::LANDED:
    return "LANDED";
    break;
  case State::DESCENT:
    return "DESCENT";
    break;
  case State::ASCENT:
    return "ASCENT";
    break;
  case State::CALIBRATION:
    return "CALIBRATION";
    break;
  case State::ABORT_ON_GROUND:
    return "ABORT_ON_GROUND";
    break;
  case State::ABORT_IN_FLIGHT:
    return "ABORT_IN_FLIGHT";
    break;
  case State::ARMED:
    return "ARMED";
    break;
  case State::PRESSURIZATION:
    return "PRESSURIZATION";
    break;
  case State::BURN:
    return "BURN";
    break;
  case State::FILLING:
    return "FILLING";
    break;
  case State::IGNITION:
    return "IGNITION";
    break;
  default:
    return "ERROR";
    break;
  }
}
