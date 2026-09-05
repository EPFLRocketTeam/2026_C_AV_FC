#include "Application/FlightControl/fc_shell.hpp"

#include <cstdio>

#include "Application/Data/data.hpp"
#include "Application/Config/config.hpp"
#include "Application/FlightControl/fc_commands.hpp"
#include "Application/FlightControl/prc_can.hpp"
#include "fc_commands_generated.hpp"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"
#include "Drivers/PRC_CAN/2026_C_AV_FC_PRC_INTRANET/include/prc_intranet/payload.hpp"

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
    flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_PRESSURIZE);
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
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_CALIBRATE);
}
void OnAvPressurize(void*) noexcept {
	  printf("[SHELL] pressurize\r\n");
	  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_PRESSURIZE);
}
void OnAvArm(void*) noexcept {
  printf("[SHELL] arm\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_ARM);
}
void OnAvLaunch(void*) noexcept {
  printf("[SHELL] launch\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_LAUNCH);
}
void OnAvAbort(void*) noexcept {
  printf("[SHELL] abort\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_ABORT);
}
void OnAvRecover(void*) noexcept {
  printf("[SHELL] recover\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_RECOVER);
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

void config_burn_cutoff_delay (void* ctx, int value) {
  config::internal::write().Burn.CutoffDelayMs = value;
}
void config_burn_impulse (void* ctx, float value) {
  config::internal::write().Burn.Impulse = value;
}
void config_burn_max_duration_engine (void* ctx, int value) {
  config::internal::write().Burn.EngineMaxDurationMs = value;
}
void config_burn_max_duration_fc (void* ctx, int value) {
  config::internal::write().Burn.FcMaxDurationMs = value;
}
void config_burn_min_duration (void* ctx, int value) {
  config::internal::write().Burn.MinDurationMs = value;
}
void config_ignition_delay (void* ctx, int value) {
  config::internal::write().Ignition.DelayMs = value;
}
void config_ignition_igniter_duration (void* ctx, int value) {
  config::internal::write().Ignition.IgniterDurationMs = value;
}
void config_ignition_prechill_duration (void* ctx, int value) {
  config::internal::write().Ignition.PrechillDurationMs = value;
}
void config_ignition_ramp_up (void* ctx, int value) {
  config::internal::write().Ignition.RampUpMs = value;
}
void config_pressurize_hold_delay (void* ctx, int value) {
  config::internal::write().Pressurization.HoldDelayMs = value;
}
void config_pressurize_max_fuel_nominal_pressure (void* ctx, float value) {
  config::internal::write().Pressurization.MaxFuelNominalPressure = value;
}
void config_pressurize_max_lox_nominal_pressure (void* ctx, float value) {
  config::internal::write().Pressurization.MaxLoxNominalPressure = value;
}
void config_pressurize_min_fuel_nominal_pressure (void* ctx, float value) {
  config::internal::write().Pressurization.MinFuelNominalPressure = value;
}
void config_pressurize_min_lox_nominal_pressure (void* ctx, float value) {
  config::internal::write().Pressurization.MinLoxNominalPressure = value;
}
void config_pressurize_target_pressure_fuel (void* ctx, float value) {
  config::internal::write().Pressurization.TargetPressureFuel = value;
}
void config_pressurize_target_pressure_lox (void* ctx, float value) {
  config::internal::write().Pressurization.TargetPressureLox = value;
}

#define app_printf(...) printf(__VA_ARGS__);
void config_print_buffer (void* ctx) {
  FlightParams params = config::internal::write();
  PRINT_FLIGHT_PARAMS(params);
}
void config_print_commited (void* ctx) {
  FlightParams params = config::get();
  PRINT_FLIGHT_PARAMS(params);
}
void config_print_status (void* ctx) {
	config::internal::print_status();
}
#undef app_printf

void config_commit (void* ctx) {
  config::internal::commit();
}

void logs_can_engine(void* ctx, bool value) {
  printf("[SHELL] can engine %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogEngine(true, value);
}
void logs_can_eth(void* ctx, bool value) {
  printf("[SHELL] can eth %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogDprEth(true, value);
}
void logs_can_fc(void* ctx, bool value) { /* Non sense, TODO remove */ }
void logs_can_lox(void* ctx, bool value) {
  printf("[SHELL] can lox %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogDprLox(true, value);
}
void logs_usb_engine(void* ctx, bool value) {
  printf("[SHELL] usb engine %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogEngine(false, value);
}
void logs_usb_eth(void* ctx, bool value) {
  printf("[SHELL] usb eth %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogDprEth(false, value);
}
void logs_usb_fc(void* ctx, bool value) { /* TODO add feature */ }
void logs_usb_lox(void* ctx, bool value) {
  printf("[SHELL] usb lox %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogDprLox(false, value);
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
    drv.config_burn_cutoff_delay = config_burn_cutoff_delay;
    drv.config_burn_impulse = config_burn_impulse;
    drv.config_burn_max_duration_engine = config_burn_max_duration_engine;
    drv.config_burn_max_duration_fc = config_burn_max_duration_fc;
    drv.config_burn_min_duration = config_burn_min_duration;
    drv.config_commit = config_commit;
    drv.config_ignition_delay = config_ignition_delay;
    drv.config_ignition_igniter_duration = config_ignition_igniter_duration;
    drv.config_ignition_prechill_duration = config_ignition_prechill_duration;
    drv.config_ignition_ramp_up = config_ignition_ramp_up;
    drv.config_pressurize_hold_delay = config_pressurize_hold_delay;
    drv.config_pressurize_max_fuel_nominal_pressure = config_pressurize_max_fuel_nominal_pressure;
    drv.config_pressurize_max_lox_nominal_pressure = config_pressurize_max_lox_nominal_pressure;
    drv.config_pressurize_min_fuel_nominal_pressure = config_pressurize_min_fuel_nominal_pressure;
    drv.config_pressurize_min_lox_nominal_pressure = config_pressurize_min_lox_nominal_pressure;
    drv.config_pressurize_target_pressure_fuel = config_pressurize_target_pressure_fuel;
    drv.config_pressurize_target_pressure_lox = config_pressurize_target_pressure_lox;
    drv.config_print_buffer = config_print_buffer;
    drv.config_print_commited = config_print_commited;
    drv.config_print_status = config_print_status;
    drv.logs_can_engine = logs_can_engine;
    drv.logs_can_eth = logs_can_eth;
    drv.logs_can_fc = logs_can_fc;
    drv.logs_can_lox = logs_can_lox;
    drv.logs_usb_engine = logs_usb_engine;
    drv.logs_usb_eth = logs_usb_eth;
    drv.logs_usb_fc = logs_usb_fc;
    drv.logs_usb_lox = logs_usb_lox;
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

namespace {
volatile uint8_t  rx_flag = 0;
uint8_t  rx_buffer[64];
uint16_t rx_len = 0;
};

void Fc_Shell_ProcessRxBytes(const uint8_t *data, uint32_t length) {
	if (length + rx_len <= sizeof(rx_buffer)) {
		memcpy(rx_buffer + rx_len, data, length);
		rx_len += length;
		rx_flag = 1; // Set flag for main loop
	}
}

void __Fc_Shell_ProcessRxBytes(const uint8_t *data, uint32_t length) {
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
void FC_Shell_Tick () {
	if (rx_flag == 0) return ;

	int start = 0;
	for (int offset = 0; offset < rx_len; offset ++) {
		if (rx_buffer[offset] == '\n') {
			__Fc_Shell_ProcessRxBytes(rx_buffer + start, offset + 1 - start);
			start = offset + 1;
		}
	}

	rx_len = 0;
	rx_flag = 0;
}
