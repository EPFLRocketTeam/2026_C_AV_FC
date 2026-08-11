#include "Application/FlightControl/fc_shell.hpp"

#include <cstdio>

#include "Application/FlightControl/prc_can.hpp"
#include "fc_commands_generated.hpp"

namespace {

constexpr uint32_t kLineBufferSize = 128;
char     g_line_buffer[kLineBufferSize];
uint32_t g_line_len = 0;

context g_cmd_ctx;
driver  g_driver;

// PRESSURIZE ("Start the pressurization process") has no per-board split
// and no float target in the CAN dictionary -- dpr_lox_pressurize/
// dpr_eth_pressurize are both just on_off (see prc_intranet's
// message_list.hpp), and the actual target pressure is a hardcoded
// firmware constant on the DPR boards themselves (prc_state.cpp's
// k_lox_set_pressure_bar/k_fuel_set_pressure_bar), not something
// commanded per-mission. So this just starts/stops pressurization on
// both boards together.
void OnPressurize(void*, bool value) noexcept {
  printf("[SHELL] pressurize %s\r\n", value ? "on" : "off");
  Fc_Can_SendDprLoxPressurize(value ? 1 : 0);
  Fc_Can_SendDprEthPressurize(value ? 1 : 0);
}
// Bench-test hookup: sends the CAN command that drives the two spare
// solenoids wired to the DPR-LOX bench board (Sol3/Sol4), repurposed as
// "LOX main"/"Ethanol main" -- see 2026_C_AV_PRC's prc_can.cpp
// ApplyCmdValves. Not a real mission valve mapping yet.
void OnMainLox(void*, bool value)  noexcept {
  printf("[SHELL] main lox %s\r\n", value ? "open" : "close");
  Fc_Can_SendMainValveCmd(0, value ? 1 : 0);
}
void OnMainFuel(void*, bool value) noexcept {
  printf("[SHELL] main fuel %s\r\n", value ? "open" : "close");
  Fc_Can_SendMainValveCmd(1, value ? 1 : 0);
}
// VENT_COPV has no dedicated valve -- N2/COPV venting is a bundled
// Vent+Safety+ball-valve sequence (see 2026_C_AV_PRC's
// Prc_Fsm_ManualVentCopv), sent to both DPR boards since it isn't
// board-specific.
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
void OnPressureLox(void*, bool value)  noexcept {
  printf("[SHELL] pressure lox %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprLoxSafety(value ? 1 : 0);
}
void OnPressureFuel(void*, bool value) noexcept {
  printf("[SHELL] pressure fuel %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprEthSafety(value ? 1 : 0);
}

// Engine bay (PRC-P) ignition sequence -- manual/bench-test triggers for
// 2026_C_AV_PRC's PrcEngineState (engine_state.cpp). See prc_can.hpp's
// comment on the Fc_Can_SendPrc* functions -- not yet tied to AvState's
// own IGNITION state.
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
// Same broadcast_abort send as "dpr broadcast_abort" -- kept as its own
// "prc abort" alias since it's the more discoverable name when
// bench-testing the engine board specifically.
void OnPrcAbort(void*) noexcept {
  printf("[SHELL] prc abort\r\n");
  Fc_Can_SendBroadcastAbort();
}

// DPR (LOX/ETH) bench-test triggers -- see prc_can.hpp's comment on the
// Fc_Can_SendDpr* functions. Exercises 2026_C_AV_PRC's prc_state.cpp
// PrcState FSM, role-gated on the receiving board.
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

bool g_initialized = false;

void EnsureInit() {
    if (g_initialized) return;
    g_initialized = true;

    g_driver.pressurize = OnPressurize;
    g_driver.main_lox = OnMainLox;
    g_driver.main_fuel = OnMainFuel;
    g_driver.vent_copv = OnVentCopv;
    g_driver.vent_lox = OnVentLox;
    g_driver.vent_fuel = OnVentFuel;
    g_driver.pressure_lox = OnPressureLox;
    g_driver.pressure_fuel = OnPressureFuel;
    g_driver.prc_clear_to_ignite = OnPrcClearToIgnite;
    g_driver.prc_ignite = OnPrcIgnite;
    g_driver.prc_passivate = OnPrcPassivate;
    g_driver.prc_reset = OnPrcReset;
    g_driver.prc_abort = OnPrcAbort;
    g_driver.dpr_lox_pressurize = OnDprLoxPressurize;
    g_driver.dpr_lox_abort = OnDprLoxAbort;
    g_driver.dpr_lox_passivate = OnDprLoxPassivate;
    g_driver.dpr_lox_reset = OnDprLoxReset;
    g_driver.dpr_eth_pressurize = OnDprEthPressurize;
    g_driver.dpr_eth_abort = OnDprEthAbort;
    g_driver.dpr_eth_passivate = OnDprEthPassivate;
    g_driver.dpr_eth_reset = OnDprEthReset;
    g_driver.dpr_broadcast_abort = OnDprBroadcastAbort;
}

};

void Fc_Shell_ProcessRxBytes(const uint8_t *data, uint32_t length) {
    EnsureInit();

    for (uint32_t i = 0; i < length; i++) {
        uint8_t byte = data[i];

        if (byte == '\n') {
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
