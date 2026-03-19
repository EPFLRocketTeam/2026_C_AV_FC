#include "av_state.h"
#include "Application/Data/data.hpp"
#include "Application/FlightControl/threshold.h"
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
      0) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition INIT->CALIBRATION");
    return State::CALIBRATION;
  }
  return currentState;
}

State AvState::fromCalibration(DataDump const &dump) {
  if (dump.uplinkCmd.id ==
      0) // TODO: replace this with proper cmd id from the protocol
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
  if (dump.uplinkCmd.id == 0)
  // TODO: replace this with proper cmd
  // id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.uplinkCmd.id ==
           0) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::ARMED;
  }
  return currentState;
}

State AvState::fromArmed(DataDump const &dump) {
  if (dump.uplinkCmd.id ==
      0) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.uplinkCmd.id ==
           0) // TODO: replace this with proper cmd id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::PRESSURIZATION;
  }
  return currentState;
}

State AvState::fromPressurization(DataDump const &dump) {
  if (dump.uplinkCmd.id == 0 ||
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
  if (dump.uplinkCmd.id == 0 ||
      (!dump.vehiculeOverview.no_cable_continuity &&
       !dump.event.vertical_acc_hold)) // TODO: replace this with proper cmd id
                                       // from the protocol and add the
                                       // condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.vehiculeOverview.no_cable_continuity ||
           dump.event.vertical_acc_hold) // TODO: replace this with proper cmd
                                         // id from the protocol and check
                                         // additional condition
  {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::BURN;
  }
  return currentState;
}

State AvState::fromBurn(DataDump const &dump) {
  if (dump.uplinkCmd.id == 0) // TODO: replace this with proper cmd id from the
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
  if (dump.uplinkCmd.id == 0) // TODO: replace this with proper cmd id from the
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
  if (dump.uplinkCmd.id == 0) // TODO: replace this with proper cmd id from the
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
  if (dump.uplinkCmd.id == 0) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  return currentState;
}

State AvState::fromAbortOnGround(DataDump const &dump) {
  if (dump.uplinkCmd.id == 0) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::INIT;
  }
  return currentState;
}

State AvState::fromAbortInFlight(DataDump const &dump) {
  if (dump.uplinkCmd.id == 0) // TODO: replace this with proper cmd id from the
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::INIT;
  }
  return currentState;
}

void AvState::update(const DataDump &dump) {
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
