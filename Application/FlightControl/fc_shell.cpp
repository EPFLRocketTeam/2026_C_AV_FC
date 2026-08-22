#include "Application/FlightControl/fc_shell.hpp"

#include <cstdio>

#include "Application/Data/data.hpp"
#include "Application/FlightControl/fc_commands.hpp"
#include "Application/FlightControl/prc_can.hpp"
#include "fc_commands_generated.hpp"

namespace {

constexpr uint32_t kLineBufferSize = 128;
char     g_line_buffer[kLineBufferSize];
uint32_t g_line_len = 0;

context g_cmd_ctx;
driver  g_driver;

};

// The handlers below are declared in fc_commands.hpp so other command
// sources (radio uplink, tests) can call them or bind them into their own
// `driver`. Doc comments live in the header; only implementation notes
// stay here.
namespace fc_commands {

// PRESSURIZE per the doc is a single command: it starts pressurizing the
// DPR boards, is the trigger for AvState's ARMED -> PRESSURIZATION
// transition (fromArmed() checks the same placeholder id as CALIBRATE/ARM,
// see OnAvCalibrate's comment below), and also clears the Engine board to
// ignite -- not three separate commands.
void OnPressurize(void*, bool value) noexcept {
  printf("[SHELL] pressurize %s\r\n", value ? "on" : "off");
  Fc_Can_SendDprLoxPressurize(value ? 1 : 0);
  Fc_Can_SendDprEthPressurize(value ? 1 : 0);
  if (value) {
    flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(2);
    Fc_Can_SendPrcClearToIgnite();
  }
}
// Toggles the engine board's MO valve directly, bypassing the FSM.
void OnMainLox(void*, bool value)  noexcept {
  printf("[SHELL] main lox %s\r\n", value ? "open" : "close");
  Fc_Can_SendMainValveCmd(0, value ? 1 : 0);
}
void OnMainFuel(void*, bool value) noexcept {
  printf("[SHELL] main fuel %s\r\n", value ? "open" : "close");
  Fc_Can_SendMainValveCmd(1, value ? 1 : 0);
}
void OnVentCopv(void*, bool value) noexcept {
  printf("[SHELL] vent copv %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprLoxCopvVent(value ? 1 : 0);
  Fc_Can_SendDprEthCopvVent(value ? 1 : 0);
}
void OnVentLox(void*, bool value)  noexcept {
  printf("[SHELL] vent lox %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprLoxVent(value ? 1 : 0);
}
void OnVentFuel(void*, bool value) noexcept {
  printf("[SHELL] vent fuel %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprEthVent(value ? 1 : 0);
}
void OnSafetyLox(void*, bool value)  noexcept {
  printf("[SHELL] safety lox %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprLoxSafety(value ? 1 : 0);
}
void OnSafetyFuel(void*, bool value) noexcept {
  printf("[SHELL] safety fuel %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprEthSafety(value ? 1 : 0);
}
void OnBallLox(void*, float value) noexcept {
  printf("[SHELL] ball lox %.1f\r\n", value);
  Fc_Can_SendDprLoxBallValve(value);
}
void OnBallFuel(void*, float value) noexcept {
  printf("[SHELL] ball fuel %.1f\r\n", value);
  Fc_Can_SendDprEthBallValve(value);
}

// Drives AvState's own top-level FSM (av_state.cpp), separate from the
// prc_*/dpr_* commands below which talk to the PRC boards over CAN. The
// numeric ids match av_state.cpp's current placeholder scheme: 2 is
// reused for CALIBRATE/ARM/PRESSURIZE (see OnPressurize above) depending
// on which state AvState is currently in, 1 is ABORT, 3 is RECOVER.
// fsm_tick() consumes the id one-shot after each tick, so it's safe to
// just set it here.
void OnAvCalibrate(void*) noexcept {
  printf("[SHELL] calibrate\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(2);
}
void OnAvArm(void*) noexcept {
  printf("[SHELL] arm\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(2);
}
void OnAvAbort(void*) noexcept {
  printf("[SHELL] abort\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(1);
}
void OnAvRecover(void*) noexcept {
  printf("[SHELL] recover\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(3);
}

// Bench-test bypass: nothing in the real firmware ever calls
// EventStore::set_calibrated(true) yet (no real calibration-completion
// logic exists), so CALIBRATION -> FILLING can never fire on its own.
// Same idea as g_uart_force_liftoff, just wired into this shell instead
// of the older raw uart_cmd_process parser.
void OnAvForceCalibrated(void*) noexcept {
  printf("[SHELL] force_calibrated\r\n");
  flight_computer::GOATStore::get_instance().eventStore.set_calibrated(true);
}

void OnPrcClearToIgnite(void*) noexcept {
  printf("[SHELL] prc clear_to_ignite\r\n");
  Fc_Can_SendPrcClearToIgnite();
}
void OnPrcIgnite(void*) noexcept {
  printf("[SHELL] prc ignite\r\n");
  Fc_Can_SendPrcIgnite();
}
void OnPrcPassivate(void*) noexcept {
  printf("[SHELL] prc passivate\r\n");
  Fc_Can_SendPrcPassivate();
}
void OnPrcReset(void*) noexcept {
  printf("[SHELL] prc reset\r\n");
  Fc_Can_SendPrcReset();
}
void OnPrcAbort(void*) noexcept {
  printf("[SHELL] prc abort\r\n");
  Fc_Can_SendBroadcastAbort();
}
// Manual bench test, separate from the Engine board's real FSM: prechill,
// igniter, then MO+ME open together (no delay between them, unlike the
// real IGNITION_BURN_START_MO -> IGNITION_BURN_START_ME stagger), hold
// 1 s, close both together. Retriggerable -- send "coldflow" again to
// run it another time. See engine_state.cpp's ColdflowSequence.
void OnPrcColdflow(void*) noexcept {
  printf("[SHELL] coldflow\r\n");
  Fc_Can_SendPrcColdflow();
}

void OnDprLoxPressurize(void*, bool value) noexcept {
  printf("[SHELL] dpr lox pressurize %s\r\n", value ? "on" : "off");
  Fc_Can_SendDprLoxPressurize(value ? 1 : 0);
}
void OnDprLoxAbort(void*) noexcept {
  printf("[SHELL] dpr lox abort\r\n");
  Fc_Can_SendDprLoxAbort();
}
void OnDprLoxPassivate(void*) noexcept {
  printf("[SHELL] dpr lox passivate\r\n");
  Fc_Can_SendDprLoxPassivate();
}
void OnDprLoxReset(void*) noexcept {
  printf("[SHELL] dpr lox reset\r\n");
  Fc_Can_SendDprLoxReset();
}
void OnDprEthPressurize(void*, bool value) noexcept {
  printf("[SHELL] dpr eth pressurize %s\r\n", value ? "on" : "off");
  Fc_Can_SendDprEthPressurize(value ? 1 : 0);
}
void OnDprEthAbort(void*) noexcept {
  printf("[SHELL] dpr eth abort\r\n");
  Fc_Can_SendDprEthAbort();
}
void OnDprEthPassivate(void*) noexcept {
  printf("[SHELL] dpr eth passivate\r\n");
  Fc_Can_SendDprEthPassivate();
}
void OnDprEthReset(void*) noexcept {
  printf("[SHELL] dpr eth reset\r\n");
  Fc_Can_SendDprEthReset();
}
void OnDprBroadcastAbort(void*) noexcept {
  printf("[SHELL] dpr broadcast_abort\r\n");
  Fc_Can_SendBroadcastAbort();
}

void FillDriver(driver& drv) {
    drv.av_calibrate = OnAvCalibrate;
    drv.av_arm = OnAvArm;
    drv.av_abort = OnAvAbort;
    drv.av_recover = OnAvRecover;
    drv.av_force_calibrated = OnAvForceCalibrated;
    drv.pressurize = OnPressurize;
    drv.main_lox = OnMainLox;
    drv.main_fuel = OnMainFuel;
    drv.vent_copv = OnVentCopv;
    drv.vent_lox = OnVentLox;
    drv.vent_fuel = OnVentFuel;
    drv.safety_lox = OnSafetyLox;
    drv.safety_fuel = OnSafetyFuel;
    drv.ball_lox = OnBallLox;
    drv.ball_fuel = OnBallFuel;
    drv.prc_clear_to_ignite = OnPrcClearToIgnite;
    drv.prc_ignite = OnPrcIgnite;
    drv.prc_passivate = OnPrcPassivate;
    drv.prc_reset = OnPrcReset;
    drv.prc_abort = OnPrcAbort;
    drv.prc_coldflow = OnPrcColdflow;
    drv.dpr_lox_pressurize = OnDprLoxPressurize;
    drv.dpr_lox_abort = OnDprLoxAbort;
    drv.dpr_lox_passivate = OnDprLoxPassivate;
    drv.dpr_lox_reset = OnDprLoxReset;
    drv.dpr_eth_pressurize = OnDprEthPressurize;
    drv.dpr_eth_abort = OnDprEthAbort;
    drv.dpr_eth_passivate = OnDprEthPassivate;
    drv.dpr_eth_reset = OnDprEthReset;
    drv.dpr_broadcast_abort = OnDprBroadcastAbort;
}

}  // namespace fc_commands

namespace {

bool g_initialized = false;

void EnsureInit() {
    if (g_initialized) return;
    g_initialized = true;

    fc_commands::FillDriver(g_driver);
}

};

void Fc_Shell_ProcessRxBytes(const uint8_t *data, uint32_t length) {
    EnsureInit();

    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = data[i];

        // Skip CR so CRLF terminals don't leave a stray \r in bool/float args.
        if (byte == '\r') {
            continue;
        }

        if (byte == '\n') {
            // Trim trailing spaces so e.g. "main lox open " (typed with a
            // stray space before Enter) doesn't fail the %bool leaf's exact
            // strcmp(arg_buffer, "open") match and silently fall through
            // to the false/close case.
            while (g_line_len > 0 && g_line_buffer[g_line_len - 1] == ' ') {
                g_line_len--;
            }
            for (uint32_t j = 0; j < g_line_len; j++) {
                push_char(&g_cmd_ctx, &g_driver, g_line_buffer[j]);
            }
            push_char(&g_cmd_ctx, &g_driver, '\n');
            g_line_len = 0;
        } else if (g_line_len < kLineBufferSize) {
            g_line_buffer[g_line_len++] = byte;
        }
    }
}
