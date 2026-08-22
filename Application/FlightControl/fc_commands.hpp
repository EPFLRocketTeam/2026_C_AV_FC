//    FILE: fc_commands.hpp
// PURPOSE: The command handlers behind FC's USB shell (see fc_shell.cpp),
//          exposed so other command sources -- the radio uplink, CAN, a
//          test harness -- can trigger the exact same actions instead of
//          re-implementing them. Signatures match the generated trie's
//          `driver` callbacks (fc_commands_generated.hpp), so they can be
//          used either as plain functions or as function pointers.
//
//          The leading `void*` parameter is the trie's handler context; it
//          is unused by every handler here, so calling them directly with
//          nullptr is fine, e.g. fc_commands::OnPrcIgnite(nullptr).
//
//          These run in interrupt context when driven from the USB shell --
//          keep anything added here short.

#ifndef APP_FC_COMMANDS_H
#define APP_FC_COMMANDS_H

#include "fc_commands_generated.hpp"

namespace fc_commands {

// Binds every handler below into `drv`, so a new command source only needs
// its own context + this call to get the full command set.
void FillDriver(driver& drv);

// -- AvState top-level FSM (av_state.cpp) ----------------------------------
// Drives AvState's own FSM, separate from the prc_*/dpr_* commands below
// which talk to the PRC boards over CAN. The numeric ids match
// av_state.cpp's current placeholder scheme: 2 is reused for
// CALIBRATE/ARM/PRESSURIZE depending on which state AvState is currently
// in, 1 is ABORT, 3 is RECOVER. fsm_tick() consumes the id one-shot after
// each tick, so it is safe to just set it.
void OnAvCalibrate(void*) noexcept;
void OnAvPressurize(void*) noexcept;
void OnAvLaunch(void*) noexcept;
void OnAvArm(void*) noexcept;
void OnAvAbort(void*) noexcept;
void OnAvRecover(void*) noexcept;

// Bench-test bypass: nothing in the real firmware ever calls
// EventStore::set_calibrated(true) yet (no real calibration-completion
// logic exists), so CALIBRATION -> FILLING can never fire on its own.
void OnAvForceCalibrated(void*) noexcept;

// -- Combined / high-level -------------------------------------------------
// PRESSURIZE per the doc is a single command: it starts pressurizing the
// DPR boards, is the trigger for AvState's ARMED -> PRESSURIZATION
// transition, and also clears the Engine board to ignite -- not three
// separate commands.
void OnPressurize(void*, bool value) noexcept;

// -- Engine board valves (bypass the FSM) ----------------------------------
void OnMainLox(void*, bool value) noexcept;
void OnMainFuel(void*, bool value) noexcept;

// -- DPR manual valve overrides -------------------------------------------
// Only take effect while the receiving DPR board is in State::MANUAL.
void OnVentCopv(void*, bool value) noexcept;
void OnVentLox(void*, bool value) noexcept;
void OnVentFuel(void*, bool value) noexcept;
void OnSafetyLox(void*, bool value) noexcept;
void OnSafetyFuel(void*, bool value) noexcept;
void OnBallLox(void*, float value) noexcept;
void OnBallFuel(void*, float value) noexcept;

// -- PRC / Engine bay ------------------------------------------------------
void OnPrcClearToIgnite(void*) noexcept;
void OnPrcIgnite(void*) noexcept;
void OnPrcPassivate(void*) noexcept;
void OnPrcReset(void*) noexcept;
void OnPrcAbort(void*) noexcept;

// Manual bench test, separate from the Engine board's real FSM: prechill,
// igniter, then MO+ME open together (no delay between them, unlike the real
// IGNITION_BURN_START_MO -> IGNITION_BURN_START_ME stagger), hold 1 s, close
// both together. Retriggerable. See engine_state.cpp's ColdflowSequence.
void OnPrcColdflow(void*) noexcept;

// -- DPR board commands ----------------------------------------------------
void OnDprLoxPressurize(void*, bool value) noexcept;
void OnDprLoxAbort(void*) noexcept;
void OnDprLoxPassivate(void*) noexcept;
void OnDprLoxReset(void*) noexcept;
void OnDprEthPressurize(void*, bool value) noexcept;
void OnDprEthAbort(void*) noexcept;
void OnDprEthPassivate(void*) noexcept;
void OnDprEthReset(void*) noexcept;
void OnDprBroadcastAbort(void*) noexcept;

void handleRxCommand(void* data) noexcept;
}  // namespace fc_commands

#endif  // APP_FC_COMMANDS_H
