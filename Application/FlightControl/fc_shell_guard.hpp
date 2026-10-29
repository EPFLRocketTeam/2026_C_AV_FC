//    FILE: fc_shell_guard.hpp
// PURPOSE: The HAL free rules behind the shell's system commands
//          (system dfu, system reboot in fc_shell.cpp): which FC states
//          allow a reset at all. Shared by the handlers and the host tests.

#ifndef APP_FC_SHELL_GUARD_H
#define APP_FC_SHELL_GUARD_H

#include <cstdint>

#include "Application/Data/fsm.hpp"

namespace fc_shell {

// A reset reboots the FC through INIT and loses the FSM history, and DFU
// also stops the radio relay until NRST or a power cycle. Both are allowed
// only in the ground states below; never when armed, pressurized, igniting
// or flying.
inline bool IsResetAllowed(flight_computer::State state) {
  switch (state) {
    case flight_computer::State::INIT:
    case flight_computer::State::CALIBRATION:
    case flight_computer::State::FILLING:
    case flight_computer::State::LANDED:
    case flight_computer::State::ABORT_ON_GROUND:
      return true;
    default:
      return false;
  }
}

inline const char* StateName(flight_computer::State state) {
  switch (state) {
    case flight_computer::State::INIT: return "INIT";
    case flight_computer::State::CALIBRATION: return "CALIBRATION";
    case flight_computer::State::FILLING: return "FILLING";
    case flight_computer::State::ARMED: return "ARMED";
    case flight_computer::State::PRESSURIZATION: return "PRESSURIZATION";
    case flight_computer::State::TRY_IGNITION: return "TRY_IGNITION";
    case flight_computer::State::IGNITION: return "IGNITION";
    case flight_computer::State::BURN: return "BURN";
    case flight_computer::State::ASCENT: return "ASCENT";
    case flight_computer::State::DESCENT: return "DESCENT";
    case flight_computer::State::LANDED: return "LANDED";
    case flight_computer::State::ABORT_ON_GROUND: return "ABORT_ON_GROUND";
    case flight_computer::State::ABORT_IN_FLIGHT: return "ABORT_IN_FLIGHT";
  }
  return "UNKNOWN";
}

}  // namespace fc_shell

#endif  // APP_FC_SHELL_GUARD_H
