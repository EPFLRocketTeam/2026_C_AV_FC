#include "Application/FlightControl/fc_shell.hpp"

#include <cstdio>

#include "Application/Data/data.hpp"
#include "Application/Config/config.hpp"
#include "Application/app_printf.h"
#include "Application/FlightControl/fc_commands.hpp"
#include "Application/FlightControl/fc_shell_intake.hpp"
#include "Application/FlightControl/fc_shell_replay.hpp"
#include "Application/FlightControl/fc_shell_guard.hpp"
#include "Application/FlightControl/prc_can.hpp"
#include "fc_commands_generated.hpp"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"
#include "Drivers/PRC_CAN/2026_C_AV_FC_PRC_INTRANET/include/prc_intranet/payload.hpp"
#include "Drivers/Camera/Camera.hpp"
#include "Application/System/bootloader.h"
#include "stm32h7xx_hal.h"

namespace {

// Bytes from the USB interrupt wait in a 256 byte ring until FC_Shell_Tick
// drains them into a 128 byte line, rebuilt across USB packets.
fc_shell::Intake<256, 128> g_intake;

// The first bytes from a host turn the USB log on, once, so a fresh board
// answers without `logs usb fc on`. `logs usb fc off` still holds after.
volatile bool g_usb_log_auto_enabled = false;

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
bool IsProtectedMode () {
  flight_computer::State state = flight_computer::GOATStore::get_instance().stateStore.get();

  return state == flight_computer::State::PRESSURIZATION
      || state == flight_computer::State::IGNITION
      || state == flight_computer::State::BURN;
}
void OnPressurize(void*, bool value) noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] pressurize %s\r\n", value ? "on" : "off");
  Fc_Can_SendDprLoxPressurize(value ? 1 : 0);
  Fc_Can_SendDprEthPressurize(value ? 1 : 0);
  if (value) {
    flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_PRESSURIZE);
    Fc_Can_SendPrcClearToIgnite();
  }
}
// Toggles the engine board's MO valve directly, bypassing the FSM.
void OnMainLox(void*, bool value)  noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] main lox %s\r\n", value ? "open" : "close");
  Fc_Can_SendMainValveCmd(0, value ? 1 : 0);
}
void OnMainFuel(void*, bool value) noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] main fuel %s\r\n", value ? "open" : "close");
  Fc_Can_SendMainValveCmd(1, value ? 1 : 0);
}
void OnVentCopv(void*, bool value) noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] vent copv %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprLoxCopvVent(value ? 1 : 0);
  Fc_Can_SendDprEthCopvVent(value ? 1 : 0);
}
void OnVentLox(void*, bool value)  noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] vent lox %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprLoxVent(value ? 1 : 0);
}
void OnVentFuel(void*, bool value) noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] vent fuel %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprEthVent(value ? 1 : 0);
}
void OnSafetyLox(void*, bool value)  noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] safety lox %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprLoxSafety(value ? 1 : 0);
}
void OnSafetyFuel(void*, bool value) noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] safety fuel %s\r\n", value ? "open" : "close");
  Fc_Can_SendDprEthSafety(value ? 1 : 0);
}
void OnBallLox(void*, float value) noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] ball lox %.1f\r\n", value);
  Fc_Can_SendDprLoxBallValve(value);
}
void OnBallFuel(void*, float value) noexcept {
  if (IsProtectedMode()) return ;
  app_printf("[SHELL] ball fuel %.1f\r\n", value);
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
  app_printf("[SHELL] calibrate\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_CALIBRATE);
}
void OnAvPressurize(void*) noexcept {
	  app_printf("[SHELL] pressurize\r\n");
	  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_PRESSURIZE);
}
void OnAvArm(void*) noexcept {
  app_printf("[SHELL] arm\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_ARM);
}
void OnAvLaunch(void*) noexcept {
  app_printf("[SHELL] launch\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_LAUNCH);
}
void OnAvAbort(void*) noexcept {
  app_printf("[SHELL] abort\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_ABORT);
}
void OnAvRecover(void* ctx) noexcept {
  app_printf("[SHELL] recover\r\n");
  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_RECOVER);
  OnPrcReset(ctx);
}

// Bench-test bypass: nothing in the real firmware ever calls
// EventStore::set_calibrated(true) yet (no real calibration-completion
// logic exists), so CALIBRATION -> FILLING can never fire on its own.
// Same idea as g_uart_force_liftoff, just wired into this shell instead
// of the older raw uart_cmd_process parser.
void OnAvForceCalibrated(void*) noexcept {
  app_printf("[SHELL] force_calibrated\r\n");
  flight_computer::GOATStore::get_instance().eventStore.set_calibrated(true);
}

void OnPrcClearToIgnite(void*) noexcept {
  app_printf("[SHELL] prc clear_to_ignite\r\n");
  Fc_Can_SendPrcClearToIgnite();
}
void OnPrcIgnite(void*) noexcept {
  app_printf("[SHELL] prc ignite\r\n");
  Fc_Can_SendPrcIgnite();
}
void OnPrcPassivate(void*) noexcept {
  app_printf("[SHELL] prc passivate\r\n");
  Fc_Can_SendPrcPassivate();
}
void OnPrcReset(void*) noexcept {
  app_printf("[SHELL] prc reset\r\n");
  Fc_Can_SendPrcReset();
  Fc_Can_SendDprEthReset();
  Fc_Can_SendDprLoxReset();
}
void OnPrcAbort(void*) noexcept {
  app_printf("[SHELL] prc abort\r\n");
  Fc_Can_SendBroadcastAbort();
}
// Manual bench test, separate from the Engine board's real FSM: prechill,
// igniter, then MO+ME open together (no delay between them, unlike the
// real IGNITION_BURN_START_MO -> IGNITION_BURN_START_ME stagger), hold
// 1 s, close both together. Retriggerable -- send "coldflow" again to
// run it another time. See engine_state.cpp's ColdflowSequence.
void OnPrcColdflow(void*) noexcept {
  app_printf("[SHELL] coldflow\r\n");
  Fc_Can_SendPrcColdflow();
}

void OnDprLoxPressurize(void*, bool value) noexcept {
  app_printf("[SHELL] dpr lox pressurize %s\r\n", value ? "on" : "off");
  Fc_Can_SendDprLoxPressurize(value ? 1 : 0);
}
void OnDprLoxAbort(void*) noexcept {
  app_printf("[SHELL] dpr lox abort\r\n");
  Fc_Can_SendDprLoxAbort();
}
void OnDprLoxPassivate(void*) noexcept {
  app_printf("[SHELL] dpr lox passivate\r\n");
  Fc_Can_SendDprLoxPassivate();
}
void OnDprLoxReset(void*) noexcept {
  app_printf("[SHELL] dpr lox reset\r\n");
  Fc_Can_SendDprLoxReset();
}
void OnDprEthPressurize(void*, bool value) noexcept {
  app_printf("[SHELL] dpr eth pressurize %s\r\n", value ? "on" : "off");
  Fc_Can_SendDprEthPressurize(value ? 1 : 0);
}
void OnDprEthAbort(void*) noexcept {
  app_printf("[SHELL] dpr eth abort\r\n");
  Fc_Can_SendDprEthAbort();
}
void OnDprEthPassivate(void*) noexcept {
  app_printf("[SHELL] dpr eth passivate\r\n");
  Fc_Can_SendDprEthPassivate();
}
void OnDprEthReset(void*) noexcept {
  app_printf("[SHELL] dpr eth reset\r\n");
  Fc_Can_SendDprEthReset();
}
void OnDprBroadcastAbort(void*) noexcept {
  app_printf("[SHELL] dpr broadcast_abort\r\n");
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
void config_set_coldflow_mode(void* ctx, bool value) {
  config::internal::write().ColdflowMode = value;
}

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

void config_commit (void* ctx) {
  config::internal::commit();
}
void coldflow_ream (void* ctx) {
  if (!config::get().ColdflowMode) return ;
  if (flight_computer::GOATStore::get_instance().stateStore.get() != flight_computer::State::ASCENT)
    return ;

  flight_computer::GOATStore::get_instance().uplinkCmdStore.set_id(AV_CMD_ARM);
}

void logs_can_engine(void* ctx, bool value) {
  app_printf("[SHELL] can engine %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogEngine(true, value);
}
void logs_can_eth(void* ctx, bool value) {
  app_printf("[SHELL] can eth %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogDprEth(true, value);
}
void logs_can_fc(void* ctx, bool value) { /* Non sense, TODO remove */ }
void logs_can_lox(void* ctx, bool value) {
  app_printf("[SHELL] can lox %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogDprLox(true, value);
}
void logs_usb_engine(void* ctx, bool value) {
  app_printf("[SHELL] usb engine %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogEngine(false, value);
}
void logs_usb_eth(void* ctx, bool value) {
  app_printf("[SHELL] usb eth %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogDprEth(false, value);
}
void logs_usb_fc(void* ctx, bool value) { 
  app_printf_use_usb(value);
}
void logs_usb_lox(void* ctx, bool value) {
  app_printf("[SHELL] usb lox %s\r\n", value ? "on" : "off");
  Fc_Can_SendLogDprLox(false, value);
}

void config_pressurize_bv_opening_fuel (void* ctx, float value) {
  config::internal::write().Pressurization.RampBVOpeningFuel = value;
}
void config_pressurize_bv_opening_lox (void* ctx, float value) {
  config::internal::write().Pressurization.RampBVOpeningLox = value;
}
void config_pressurize_preburn_duration_fuel (void* ctx, float value) {
  config::internal::write().Pressurization.PreburnDurationFuelMs = value;
}
void config_pressurize_preburn_duration_lox (void* ctx, float value) {
  config::internal::write().Pressurization.PreburnDurationLoxMs = value;
}
void config_pressurize_stable_opening_fuel (void* ctx, float value) {
  config::internal::write().Pressurization.StableBVOpeningFuel = value;
}
void config_pressurize_stable_opening_lox (void* ctx, float value) {
  config::internal::write().Pressurization.StableBVOpeningLox = value;
}

void cameras_abort (void* ctx) {
  cameraDriver.abort();
}
void cameras_display_aero_bot (void* ctx) {
  cameraDriver.display(Camera::CAM_AERO_BOT);
}
void cameras_display_aero_top (void* ctx) {
  cameraDriver.display(Camera::CAM_AERO_TOP);
}
void cameras_display_all (void* ctx) {
  cameraDriver.displayAll();
}
void cameras_display_sepmech (void* ctx) {
  cameraDriver.display(Camera::CAM_SEPMECH);
}
void cameras_recover (void* ctx) {
  cameraDriver.recover();
}
void cameras_start (void* ctx) {
  cameraDriver.start();
}
void cameras_stop (void* ctx) {
  cameraDriver.stop();
}

// system dfu and system reboot both reset the FC, so both refuse outside
// the ground states of fc_shell::IsResetAllowed (fc_shell_guard.hpp), and
// neither acts on its own: each arms a 10 s window, and only a different
// command, system confirm, acts. FILLING is the bench idle state and also
// the loaded rocket, which the FC cannot tell apart.
fc_shell::ConfirmWindow g_reset_confirm;

// Prints the refusal and disarms when the FC state forbids a reset.
bool ResetAllowedNow(const char* command) {
  const flight_computer::State state = flight_computer::GOATStore::get_instance().stateStore.get();
  if (fc_shell::IsResetAllowed(state)) return true;
  g_reset_confirm.Disarm();
  app_printf("[SHELL] %s refused: FC state %s, allowed only in INIT, CALIBRATION, FILLING, LANDED or ABORT_ON_GROUND\r\n",
             command, fc_shell::StateName(state));
  return false;
}

void ArmReset(const char* command, fc_shell::ConfirmWindow::Command which) {
  if (!ResetAllowedNow(command)) return;
  g_reset_confirm.Arm(which, HAL_GetTick());
  const flight_computer::State state = flight_computer::GOATStore::get_instance().stateStore.get();
  // 0x90 is ROM bootloader V9.0, with AN2606 Rev 61 Table 114 limitations.
  app_printf("[SHELL] %s armed: FC state %s, ROM bootloader id 0x%02X, type system confirm within 10 s\r\n",
             command, fc_shell::StateName(state), static_cast<unsigned>(Bootloader_RomId()));
}

// Reboots into the STM32H743 ROM DFU bootloader (Application/System/
// bootloader.h) once confirmed; the host then flashes over the DFU USB port.
void OnSystemDfu(void*) noexcept {
  app_printf("[SHELL] system dfu\r\n");
  ArmReset("system dfu", fc_shell::ConfirmWindow::Command::kDfu);
}
void OnSystemReboot(void*) noexcept {
  app_printf("[SHELL] system reboot\r\n");
  ArmReset("system reboot", fc_shell::ConfirmWindow::Command::kReboot);
}
void OnSystemConfirm(void*) noexcept {
  app_printf("[SHELL] system confirm\r\n");
  const fc_shell::ConfirmWindow::Command armed = g_reset_confirm.Confirm(HAL_GetTick());
  if (armed == fc_shell::ConfirmWindow::Command::kNone) {
    app_printf("[SHELL] system confirm: nothing armed in the last 10 s, type system dfu or system reboot first\r\n");
    return;
  }
  if (!ResetAllowedNow("system confirm")) return;
  if (armed == fc_shell::ConfirmWindow::Command::kDfu) {
    app_printf("[SHELL] rebooting into the ROM DFU bootloader, flash over the DFU USB port\r\n");
    HAL_Delay(50);  // let the last USB packet leave before the reset
    Bootloader_RequestDfu();
  }
  app_printf("[SHELL] rebooting\r\n");
  HAL_Delay(50);  // let the last USB packet leave before the reset
  Bootloader_Reboot();
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
    drv.dpr_lox_pressurize = OnDprLoxPressurize;
    drv.dpr_lox_abort = OnDprLoxAbort;
    drv.dpr_lox_passivate = OnDprLoxPassivate;
    drv.dpr_lox_reset = OnDprLoxReset;
    drv.dpr_eth_pressurize = OnDprEthPressurize;
    drv.dpr_eth_abort = OnDprEthAbort;
    drv.dpr_eth_passivate = OnDprEthPassivate;
    drv.dpr_eth_reset = OnDprEthReset;
    drv.dpr_broadcast_abort = OnDprBroadcastAbort;
    drv.cameras_abort = cameras_abort;
    drv.cameras_display_aero_bot = cameras_display_aero_bot;
    drv.cameras_display_aero_top = cameras_display_aero_top;
    drv.cameras_display_all = cameras_display_all;
    drv.cameras_display_sepmech = cameras_display_sepmech;
    drv.cameras_recover = cameras_recover;
    drv.cameras_start = cameras_start;
    drv.cameras_stop = cameras_stop;
    drv.coldflow_rearm = coldflow_ream;
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
    drv.config_pressurize_bv_opening_lox = config_pressurize_bv_opening_lox;
    drv.config_pressurize_bv_opening_fuel = config_pressurize_bv_opening_fuel;
    drv.config_pressurize_max_fuel_nominal_pressure = config_pressurize_max_fuel_nominal_pressure;
    drv.config_pressurize_max_lox_nominal_pressure = config_pressurize_max_lox_nominal_pressure;
    drv.config_pressurize_min_fuel_nominal_pressure = config_pressurize_min_fuel_nominal_pressure;
    drv.config_pressurize_min_lox_nominal_pressure = config_pressurize_min_lox_nominal_pressure;
    drv.config_pressurize_preburn_duration_fuel = config_pressurize_preburn_duration_fuel;
    drv.config_pressurize_preburn_duration_lox = config_pressurize_preburn_duration_lox;
    drv.config_pressurize_stable_opening_fuel = config_pressurize_stable_opening_fuel;
    drv.config_pressurize_stable_opening_lox = config_pressurize_stable_opening_lox;
    drv.config_pressurize_target_pressure_fuel = config_pressurize_target_pressure_fuel;
    drv.config_pressurize_target_pressure_lox = config_pressurize_target_pressure_lox;
    drv.config_print_buffer = config_print_buffer;
    drv.config_print_commited = config_print_commited;
    drv.config_print_status = config_print_status;
    drv.config_set_coldflow_mode = config_set_coldflow_mode;
    drv.logs_can_engine = logs_can_engine;
    drv.logs_can_eth = logs_can_eth;
    drv.logs_can_fc = logs_can_fc;
    drv.logs_can_lox = logs_can_lox;
    drv.logs_usb_engine = logs_usb_engine;
    drv.logs_usb_eth = logs_usb_eth;
    drv.logs_usb_fc = logs_usb_fc;
    drv.logs_usb_lox = logs_usb_lox;
    drv.system_dfu = OnSystemDfu;
    drv.system_reboot = OnSystemReboot;
    drv.system_confirm = OnSystemConfirm;
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

// USB interrupt context: only stores the bytes. When the ring is full the
// excess is dropped and the intake throws away the line it cut (see
// SpscRing in fc_shell_intake.hpp), so a cut line never runs.
void Fc_Shell_ProcessRxBytes(const uint8_t *data, uint32_t length) {
    if (!g_usb_log_auto_enabled) {
        g_usb_log_auto_enabled = true;
        app_printf_use_usb(true);
    }
    g_intake.Receive(data, length);
}

// Main loop: drains the ring and replays each complete line through the
// generated trie. Bytes of an unfinished line wait for the rest to arrive
// in a later packet.
void FC_Shell_Tick () {
    EnsureInit();

    g_intake.Drain([](const char* line, uint32_t length) {
        fc_shell::ReplayLine(&g_cmd_ctx, &g_driver, line, length);
    });
}
