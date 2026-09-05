#include "av_state.h"
#include "Application/Data/data.hpp"
#include "Application/Config/config.hpp"
#include "Application/FlightControl/prc_can.hpp"
#include "Application/FlightControl/threshold.h"
#include "Application/Kalman/kalman_lifecycle.h"
#include "Drivers/STM32HAL/stm32hal.h"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"

extern "C" {
#include "Application/FlightControl/uart_cmd.h"
}

#include <cstdio>
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
      AV_CMD_CALIBRATE)
  {
    // Logger::log_eventf("FSM transition INIT->CALIBRATION");
    // TODO replace this with State::CALIBRATION
	//   Dirty fix for DPR tests
	return State::FILLING;
  }
  return currentState;
}

State AvState::fromCalibration(DataDump const &dump) {
  if (dump.uplinkCmd.id ==
		  AV_CMD_ABORT)
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
  if (dump.uplinkCmd.id == AV_CMD_ABORT)
  // TODO: replace this with proper cmd
  // id from the protocol
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.uplinkCmd.id ==
		  AV_CMD_ARM)
  {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::ARMED;
  }
  return currentState;
}

State AvState::fromArmed(DataDump const &dump) {
  if (dump.uplinkCmd.id ==
		  AV_CMD_ABORT)
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  else if (dump.uplinkCmd.id ==
		  AV_CMD_PRESSURIZE)
  {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::PRESSURIZATION;
  }
  return currentState;
}

// Temporary bench-test relaxation: hold-delay OR nominal pressure, not
// AND, and the hold delay is a real fixed 10 s timer computed locally
// instead of the never-set event.timer_launch_delay flag. Without real
// pressurant, mean pressures never reach the nominal band, so this lets
// the hold delay alone carry the transition on the bench.
// static constexpr uint32_t kPressurizationHoldDelayMs = 30000;

State AvState::fromPressurization(DataDump const &dump) {
  if (dump.uplinkCmd.id == AV_CMD_ABORT ||
      config::get().Pressurization.MaxCriticalPressure < dump.propSensors.ETA_pressure ||
      config::get().Pressurization.MaxCriticalPressure < dump.propSensors.OTA_pressure)
  // TODO: replace this with proper cmd id from the
  // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }

  const bool hold_delay_elapsed =
      HAL_GetTick() - pressurization_entry_ms_ >= config::get().Pressurization.HoldDelayMs;
  // TODO nominal pressure per tank
  // TODO add commands for 
  const bool pressure_nominal =
      dump.propSensors.OTA_pressure < config::get().Pressurization.MaxLoxNominalPressure  &&
      dump.propSensors.ETA_pressure < config::get().Pressurization.MaxFuelNominalPressure &&
      dump.propSensors.OTA_pressure > config::get().Pressurization.MinLoxNominalPressure  &&
      dump.propSensors.ETA_pressure > config::get().Pressurization.MinFuelNominalPressure; 

  if (hold_delay_elapsed && pressure_nominal) {
    // Logger::log_eventf("FSM transition PRESSURIZATION->INGITION");
    return State::IGNITION;
  }
  return currentState;
}

State AvState::fromIgnition(DataDump const &dump) {
  // IGNITION only has ABORT_ON_GROUND/BURN as valid outgoing edges per the fsm diagram,
  // ABORT_IN_FLIGHT only exists from BURN/ASCENT/DESCENT (after liftoff is confirmed),
  // so a catastrophic failure here goes to ABORT_ON_GROUND, not ABORT_IN_FLIGHT.
  if (dump.event.catastrophic_failure) {
    return State::ABORT_ON_GROUND;
  }

  if (dump.uplinkCmd.id == AV_CMD_ABORT) { // ABORT command
    return State::ABORT_ON_GROUND;
  }

  // TEMPORARY bench bypass: no real motor burn on the bench means the
  // accel-hold check always reads ACC_HOLD_DID_NOT_HOLD, so IGNITION would
  // always auto-abort. Force BURN here instead; real liftoff logic below is
  // left intact and unreachable until this is removed.
  return State::BURN;

  // Liftoff detection, per spec: cable disconnect OR the accel hold
  // confirming it is enough for BURN, either signal alone is trusted.
  // No liftoff (ABORT_ON_GROUND) still needs both signals to agree
  // nothing happened. While the accel hold hasn't concluded yet
  // (ACC_HOLD_NOT_ELAPSED) and the cable is still connected, stay in
  // IGNITION.
  const bool cable_lost = dump.vehiculeOverview.no_cable_continuity;
  if (cable_lost || dump.event.vertical_acc_hold == ACC_HOLD_DID_HOLD) {
    return State::BURN;
  }
  // TODO wtf, maybe add a delay idk
  if (!cable_lost && dump.event.vertical_acc_hold == ACC_HOLD_DID_NOT_HOLD) {
    return State::ABORT_ON_GROUND;
  }

  return currentState;
}

State AvState::fromBurn(DataDump const &dump) {
  if (dump.event.catastrophic_failure) {
    return State::ABORT_IN_FLIGHT;
  }

  if (dump.uplinkCmd.id == AV_CMD_ABORT)
                              // protocol and add the condition p_tanks > p_prvs
  {
    return State::ABORT_IN_FLIGHT;
  }

  // Engine cut-off (ECO) detected (MO/ME both closed -- see prc_can.cpp's
  // OnPrcState, fed by 2026_C_AV_PRC's engine board prc_state telemetry)
  // OR max burn duration elapsed, per spec. cut_off_detected alone can't
  // tell "burn finished" apart from "hasn't ignited yet" (both read as
  // MO/ME closed), so it's only trusted once MIN_BURN_DURATION has passed.
  // timer_burn is milliseconds (HAL_GetTick()-based); MIN_BURN_DURATION/
  // BURN_MAX_DURATION are seconds, so both need the *1000 conversion --
  // it was missing here, which made BURN_MAX_DURATION's "120 s" behave as
  // 120 ms in practice.
  if (dump.event.cut_off_detected ||
      dump.propSensors.timer_burn > config::get().Burn.FcMaxDurationMs) {
    return State::ASCENT;
  }

  return currentState;
}

State AvState::fromAscent(DataDump const &dump) {
  if (dump.event.catastrophic_failure) {
    return State::ABORT_IN_FLIGHT;
  }

  if (dump.uplinkCmd.id == AV_CMD_ABORT)
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_IN_FLIGHT;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  // ascent_duration is milliseconds (HAL_GetTick()-based); ASCENT_MAX_DURATION
  // is seconds, hence *1000 -- same unit bug fromBurn() had with
  // BURN_MAX_DURATION/MIN_BURN_DURATION.
  // TODO add Ascent to parameters ????
  else if (dump.event.apogee_detected ||
           dump.flightEventTimers.ascent_duration > config::get().Ascent.AscentMaxDurationMs) {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::DESCENT;
  }
  return currentState;
}

State AvState::fromDescent(DataDump const &dump) {
  if (dump.event.catastrophic_failure) {
    return State::ABORT_IN_FLIGHT;
  }

  if (dump.uplinkCmd.id == AV_CMD_ABORT)
                              // protocol and add the condition p_tanks > p_prvs
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_IN_FLIGHT;
  }
  // If all the sensors are calibrated and ready for use we go to the MANUAL
  // state
  // descent_duration is milliseconds; DESCENT_THRESHOLD_DURATION is
  // seconds, same *1000 fix as ASCENT_MAX_DURATION above.
  else if (dump.event.touchdown_detected &&
    // TODO also put Ms as a suffix
           dump.flightEventTimers.descent_duration > config::get().Descent.MaxDurationMs) {
    // Logger::log_eventf("FSM transition CALIBRATION->MANUAL");
    return State::LANDED;
  }
  return currentState;
}

State AvState::fromLanded(DataDump const &dump) {
  if (dump.uplinkCmd.id == AV_CMD_ABORT)
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::ABORT_ON_GROUND;
  }
  return currentState;
}

State AvState::fromAbortOnGround(DataDump const &dump) {
  if (dump.uplinkCmd.id == AV_CMD_RECOVER)
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::INIT;
  }
  return currentState;
}

State AvState::fromAbortInFlight(DataDump const &dump) {
  if (dump.uplinkCmd.id == AV_CMD_RECOVER)
  {
    // Logger::log_eventf("FSM transition CALIBRATION->ERROR_GROUND");
    return State::INIT;
  }
  return currentState;
}

void AvState::update(const DataDump &dump) {
  const State previous_state = currentState;

  // ── Override: UART force-liftoff command ──────────────────────────
  // If the "LIFTOFF" command was received via USB CDC, jump straight
  // to BURN from any ground state.  The flag is consumed (cleared)
  // here so it acts as a one-shot.
  if (g_uart_force_liftoff) {
    g_uart_force_liftoff = false;
    if (currentState <= State::IGNITION) {  // any pre-flight state
      currentState = State::BURN;
    }
  }

  // ── Override: IMU-based liftoff detection ─────────────────────────
  // The dual-window detector in the Kalman subsystem sets this flag
  // once sustained excess acceleration is confirmed.
  if (dump.event.imu_liftoff_detected && currentState <= State::IGNITION) {
    currentState = State::BURN;
  }

  // ── Normal FSM transitions ────────────────────────────────────────
  if (currentState == previous_state) {
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
  } // end if (currentState == previous_state)

  if (currentState != previous_state) {
    printf("[FSM] %s -> %s\r\n",
           stateToString(previous_state).c_str(),
           stateToString(currentState).c_str());
    kalman_on_state_change(static_cast<uint32_t>(currentState));

    if (currentState == State::PRESSURIZATION) {
      pressurization_entry_ms_ = HAL_GetTick();
    }

    // Liftoff contract: first in-flight state reached by this FSM is BURN.
    if (currentState == State::BURN) {
      liftoff_entry_ms_ = HAL_GetTick();
      has_lifted_off_   = true;
      kalman_on_liftoff(dump.av_timestamp);
    }
    if (currentState == State::ASCENT) {
      ascent_entry_ms_ = HAL_GetTick();
      // Pressurize-off side effect: DPR boards drop REGULATE -> PRESSURIZE_OFF
      // on their own (2026_C_AV_PRC's prc_state.cpp), which closes Safety,
      // closes Vent, and sets the ball valve to 0% on entry.
      Fc_Can_SendDprLoxPressurize(0);
      Fc_Can_SendDprEthPressurize(0);
    }
    if (currentState == State::DESCENT) {
      descent_entry_ms_ = HAL_GetTick();
    }

    // Igniter side effect: entering IGNITION sends prc_ignite so the
    // Engine board starts its own ignition sequence (ClearToIgnite ->
    // IgnitionPrechill, see 2026_C_AV_PRC's engine_state.cpp), matching
    // OnPressurize's clear_to_ignite send that got it to ClearToIgnite.
    if (currentState == State::IGNITION) {
      Fc_Can_SendPrcIgnite();
    }

    // Abort side effect (spec: "Immediately gives the command to the DPRs
    // to depressurize the tanks (ABORT) and propulsion board to shutdown
    // the engine with subsequent passivation") -- broadcast_abort is
    // recognized by both 2026_C_AV_PRC's DPR FSM (PrcState::IsAbortCmd)
    // and its engine FSM (PrcEngineState::AbortCmd), role-agnostic, so one
    // send reaches every board.
    if (currentState == State::ABORT_IN_FLIGHT || currentState == State::ABORT_ON_GROUND) {
      Fc_Can_SendBroadcastAbort();
    }

    // TODO(SepMech): spec requires "immediately triggers the SepMech" on
    // entering ABORT_IN_FLIGHT. No separation mechanism driver/CAN message
    // exists anywhere in this codebase yet -- call it here once it does.
    if (currentState == State::ABORT_IN_FLIGHT) {
      // Fc_Can_SendSepMechTrigger();
    }

    // TODO(SepMech): spec's DESCENT description: "Apogee reached and
    // detected, separation mechanism triggered." Same missing driver as
    // above -- call it here once it exists.
    if (currentState == State::DESCENT) {
      // Fc_Can_SendSepMechTrigger();

      // Passivate side effect: tells the Engine board to run its
      // passivation sequence (2026_C_AV_PRC's engine_state.cpp:
      // WaitForPassivation -> PassivationSeparationDelay -> PassivationEth
      // -> PassivationCloseMe -> PassivationLox -> Shutoff ->
      // DepressurizeOpen). The delayed DPR depressurize send below is
      // timed to when that finishes.
      Fc_Can_SendPrcPassivate();
      descent_depressurize_sent_ = false;
    }
  }

  // Published every tick (not just on transition) so fromAscent()/
  // fromDescent()'s duration checks see a live, continuously advancing
  // value -- ascent/descent_duration reset to 0 on entry (stamped above)
  // and only mean anything while actually in that state; flight_duration
  // keeps advancing for the rest of the flight once liftoff has happened.
  auto& timers = GOATStore::get_instance().flightEventTimersStore;
  if (has_lifted_off_) {
    timers.set_flight_timer(HAL_GetTick() - liftoff_entry_ms_);
  }
  if (currentState == State::ASCENT) {
    timers.set_ascent_timer(HAL_GetTick() - ascent_entry_ms_);
  }
  if (currentState == State::DESCENT) {
    timers.set_descent_timer(HAL_GetTick() - descent_entry_ms_);

    // Engine's passivation sequence (WaitForPassivation, sent above, through
    // DepressurizeOpen) is 5 sequential placeholder stages -- separation
    // delay, passivation fuel duration, interlude, passivation ox duration,
    // depressurize delay -- each currently 10 s in engine_state.cpp, so
    // 50 s total. Keep this in sync if those stop being uniform.
    // constexpr uint32_t kDescentDepressurizeDelayMs = 50000;
    if (!descent_depressurize_sent_ &&
        HAL_GetTick() - descent_entry_ms_ >= config::get().Descent.Depressurize.DelayMs) {
      Fc_Can_SendDprLoxPassivate();
      Fc_Can_SendDprEthPassivate();
      descent_depressurize_sent_ = true;
    }
  }

  // fromBurn()'s BURN_MAX_DURATION fallback reads this -- reuses
  // liftoff_entry_ms_ since BURN entry is liftoff, same timestamp.
  if (currentState == State::BURN) {
    GOATStore::get_instance().propSensorsStore.set_timer_burn(HAL_GetTick() - liftoff_entry_ms_);
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

// ── Global FSM instance and super-loop tick ────────────────────────────────
// Called from main.cpp super-loop after kalman_loop().  We own the AvState
// instance here to keep the BMP390-conflicting data.hpp include out of
// main.cpp.
// NOTE: Meyers singleton pattern avoids file-scope static constructor order
// issues with <iostream> / <string> on bare-metal.
static AvState& fsm_instance() {
  static AvState inst;
  return inst;
}

void fsm_tick(void) {
  auto& goat = flight_computer::GOATStore::get_instance();
  const auto& dump = goat.get();
  AvState& fsm = fsm_instance();

  fsm.update(dump);
  // Publish the new FSM state via stateStore so get() returns the correct value.
  goat.stateStore.set(fsm.getCurrentState());
  // One-shot: a command id is only meant to fire the transition it was sent
  // for, not linger and re-trigger a same-numbered check in a later state.
  goat.uplinkCmdStore.set_id(0);
}
