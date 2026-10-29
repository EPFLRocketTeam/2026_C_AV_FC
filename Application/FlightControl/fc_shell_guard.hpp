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

// Two step confirmation for system dfu and system reboot: the first call
// arms a 10 s window for that command, a second call of the same command
// inside the window confirms it. Anything else (the window ran out, or the
// other command came in between) arms again. A refused call disarms.
// Times are milliseconds from any free running clock (HAL_GetTick on the
// board); wrap around is handled.
class ConfirmWindow {
 public:
  enum class Command : uint8_t { kNone, kDfu, kReboot };
  static constexpr uint32_t kWindowMs = 10000;

  // Returns true when this call confirms the command; false when it armed.
  bool Request(Command command, uint32_t now_ms) {
    if (command == armed_ && now_ms - armed_at_ms_ < kWindowMs) {
      armed_ = Command::kNone;
      return true;
    }
    armed_ = command;
    armed_at_ms_ = now_ms;
    return false;
  }

  void Disarm() { armed_ = Command::kNone; }

 private:
  Command armed_ = Command::kNone;
  uint32_t armed_at_ms_ = 0;
};

}  // namespace fc_shell

#endif  // APP_FC_SHELL_GUARD_H
