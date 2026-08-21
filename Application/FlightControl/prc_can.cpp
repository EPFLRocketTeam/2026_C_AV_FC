#include "Application/FlightControl/prc_can.hpp"

#include <cstdio>
#include <cstring>

#include "Application/Data/data.hpp"
#include "log_aggregator/reassembler.hpp"
#include "prc_intranet/const.hpp"
#include "prc_intranet/dispatch.hpp"
#include "prc_intranet/transmit.hpp"

using namespace flight_computer;
namespace pi = prc_intranet;

namespace {

FDCAN_HandleTypeDef* g_hfdcan = nullptr;

// Pads to the full word-aligned payload since the HAL always word-copies.
void CbSend(void* driver_ptr, uint16_t can_id, const uint8_t* buffer, uint32_t dlc) noexcept {
  auto* hfdcan = static_cast<FDCAN_HandleTypeDef*>(driver_ptr);

  FDCAN_TxHeaderTypeDef txHeader;
  txHeader.Identifier          = can_id;
  txHeader.IdType              = FDCAN_STANDARD_ID;
  txHeader.TxFrameType         = FDCAN_DATA_FRAME;
  txHeader.DataLength          = dlc;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  txHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker       = 0;

  uint8_t txData[pi::constants::MAX_PAYLOAD_SIZE] = {0};
  memcpy(txData, buffer, dlc);

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    printf("[FC CAN] TX failed, id=0x%X\r\n", can_id);
  }
}

// One reassembler per source board, each with its own buffer.
log_aggregator::LineReassembler g_log_prc_engine;
log_aggregator::LineReassembler g_log_dpr_eth;
log_aggregator::LineReassembler g_log_dpr_lox;

void PrintTaggedLine(void* ctx, const char* line, uint32_t length) noexcept {
  printf("%s %.*s\r\n", static_cast<const char*>(ctx), static_cast<int>(length), line);
}

void OnLogChunkPrcEngine(void*, pi::payload::log_chunk chunk) noexcept {
  g_log_prc_engine.feed(chunk.bytes, const_cast<char*>("[PRC-ENGINE]"), PrintTaggedLine);
}
void OnLogChunkDprEth(void*, pi::payload::log_chunk chunk) noexcept {
  g_log_dpr_eth.feed(chunk.bytes, const_cast<char*>("[PRC-ETH]"), PrintTaggedLine);
}
void OnLogChunkDprLox(void*, pi::payload::log_chunk chunk) noexcept {
  g_log_dpr_lox.feed(chunk.bytes, const_cast<char*>("[PRC-LOX]"), PrintTaggedLine);
}

void OnDprEthPressures(void*, pi::payload::dpr_eth_pressures p) noexcept {
  GOATStore::get_instance().propSensorsStore.set_fuel_pressure_mean(p.p_eta);
}
void OnDprLoxPressures(void*, pi::payload::dpr_lox_pressures p) noexcept {
  GOATStore::get_instance().propSensorsStore.set_LOX_pressure_mean(p.p_ota);
}

void OnPrcPInjector(void*, pi::payload::prc_p_injector p) noexcept {
  auto& sensors = GOATStore::get_instance().propSensorsStore;
  sensors.set_LOX_inj_pressure(p.p_oin);
  sensors.set_fuel_inj_pressure(p.p_ein);
}
void OnPrcPChamber(void*, pi::payload::prc_p_chamber p) noexcept {
  GOATStore::get_instance().propSensorsStore.set_chamber_pressure(p.p_ccc);
}
void OnPrcTInjector(void*, pi::payload::prc_t_injector t) noexcept {
  auto& sensors = GOATStore::get_instance().propSensorsStore;
  sensors.set_LOX_inj_temperature(t.t_oin);
  sensors.set_fuel_inj_temperature(t.t_ein);
}
void OnPrcTChamber(void*, pi::payload::prc_t_chamber t) noexcept {
  GOATStore::get_instance().propSensorsStore.set_chamber_temperature(t.t_ccc);
}

// Both valves closed means the engine cut off.
void OnPrcState(void*, pi::payload::prc_state state) noexcept {
  const bool mo_open = (state.valve_mask & pi::constants::VALVE_MASK_BIT_MO) != 0;
  const bool me_open = (state.valve_mask & pi::constants::VALVE_MASK_BIT_ME) != 0;
  GOATStore::get_instance().eventStore.set_cut_off_detected(!mo_open && !me_open);
}

pi::context& Ctx() {
  static pi::context ctx = [] {
    pi::prc_driver driver{};
    driver.send                 = CbSend;
    driver.on_dpr_eth_pressures = OnDprEthPressures;
    driver.on_dpr_lox_pressures = OnDprLoxPressures;
    driver.on_prc_p_chamber     = OnPrcPChamber;
    driver.on_prc_p_injector    = OnPrcPInjector;
    driver.on_prc_t_chamber     = OnPrcTChamber;
    driver.on_prc_t_injector    = OnPrcTInjector;
    driver.on_prc_state         = OnPrcState;
    driver.on_log_chunk_prc_engine = OnLogChunkPrcEngine;
    driver.on_log_chunk_dpr_eth    = OnLogChunkDprEth;
    driver.on_log_chunk_dpr_lox    = OnLogChunkDprLox;
    return pi::create_context(driver);
  }();
  return ctx;
}

} // namespace

void Fc_Can_ProcessRxMessage(uint32_t can_id, const uint8_t *data, uint32_t dlc) {
  pi::dispatch_frame(&Ctx(), static_cast<uint16_t>(can_id), data, dlc);
}

void Fc_Can_Init(FDCAN_HandleTypeDef *hfdcan) {
  g_hfdcan = hfdcan;
}

// Targets the engine board's own MO/ME valves, not a DPR board.
void Fc_Can_SendMainValveCmd(uint8_t is_ethanol, uint8_t open) {
  if (g_hfdcan == nullptr) return;

  pi::payload::cmd_valves cmd{};
  cmd.valve_id = is_ethanol ? pi::constants::VALVE_SOL4 : pi::constants::VALVE_SOL3;
  cmd.state    = open ? pi::constants::VALVE_STATE_OPEN : pi::constants::VALVE_STATE_CLOSED;

  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_prc_cmd_valves(&ctx, cmd);
}

void Fc_Can_SendPrcClearToIgnite(void) {
  if (g_hfdcan == nullptr) return;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_prc_clear_to_ignite(&ctx, pi::payload::empty{});
}

void Fc_Can_SendPrcIgnite(void) {
  if (g_hfdcan == nullptr) return;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_prc_ignite(&ctx, pi::payload::empty{});
}

void Fc_Can_SendPrcPassivate(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::safety_key key{};
  key.safety_key = pi::constants::SAFETY_KEY_PRC_PASSIVATE;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_prc_passivate(&ctx, key);
}

void Fc_Can_SendPrcReset(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::reset r{};
  r.magic = pi::constants::RESET_MAGIC;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_prc_reset(&ctx, r);
}

void Fc_Can_SendPrcColdflow(void) {
  if (g_hfdcan == nullptr) return;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_prc_coldflow(&ctx, pi::payload::empty{});
}

void Fc_Can_SendDprLoxPressurize(uint8_t open) {
  if (g_hfdcan == nullptr) return;
  pi::payload::on_off cmd{};
  cmd.state = open ? pi::constants::CMD_ON : pi::constants::CMD_OFF;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_lox_pressurize(&ctx, cmd);
}

void Fc_Can_SendDprLoxAbort(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::safety_key key{};
  key.safety_key = pi::constants::SAFETY_KEY_DPR_LOX_ABORT;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_lox_abort(&ctx, key);
}

void Fc_Can_SendDprLoxPassivate(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::safety_key key{};
  key.safety_key = pi::constants::SAFETY_KEY_DPR_LOX_PASSIVATE;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_lox_passivate(&ctx, key);
}

void Fc_Can_SendDprLoxReset(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::reset r{};
  r.magic = pi::constants::RESET_MAGIC;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_lox_reset(&ctx, r);
}

void Fc_Can_SendDprEthPressurize(uint8_t open) {
  if (g_hfdcan == nullptr) return;
  pi::payload::on_off cmd{};
  cmd.state = open ? pi::constants::CMD_ON : pi::constants::CMD_OFF;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_eth_pressurize(&ctx, cmd);
}

void Fc_Can_SendDprEthAbort(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::safety_key key{};
  key.safety_key = pi::constants::SAFETY_KEY_DPR_ETH_ABORT;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_eth_abort(&ctx, key);
}

void Fc_Can_SendDprEthPassivate(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::safety_key key{};
  key.safety_key = pi::constants::SAFETY_KEY_DPR_ETH_PASSIVATE;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_eth_passivate(&ctx, key);
}

void Fc_Can_SendDprEthReset(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::reset r{};
  r.magic = pi::constants::RESET_MAGIC;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_eth_reset(&ctx, r);
}

void Fc_Can_SendBroadcastAbort(void) {
  if (g_hfdcan == nullptr) return;
  pi::payload::safety_key key{};
  key.safety_key = pi::constants::SAFETY_KEY_BROADCAST_ABORT;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_broadcast_abort(&ctx, key);
}

static void SendCmdValves(void (*send)(pi::context*, pi::payload::cmd_valves), uint8_t valve_id, uint8_t open) {
  if (g_hfdcan == nullptr) return;
  pi::payload::cmd_valves cmd{};
  cmd.valve_id = valve_id;
  cmd.state    = open ? pi::constants::VALVE_STATE_OPEN : pi::constants::VALVE_STATE_CLOSED;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  send(&ctx, cmd);
}

void Fc_Can_SendDprLoxVent(uint8_t open) {
  SendCmdValves(pi::send_dpr_lox_cmd_valves, pi::constants::VALVE_VENT, open);
}
void Fc_Can_SendDprEthVent(uint8_t open) {
  SendCmdValves(pi::send_dpr_eth_cmd_valves, pi::constants::VALVE_VENT, open);
}
void Fc_Can_SendDprLoxSafety(uint8_t open) {
  SendCmdValves(pi::send_dpr_lox_cmd_valves, pi::constants::VALVE_SAFETY, open);
}
void Fc_Can_SendDprEthSafety(uint8_t open) {
  SendCmdValves(pi::send_dpr_eth_cmd_valves, pi::constants::VALVE_SAFETY, open);
}
void Fc_Can_SendDprLoxCopvVent(uint8_t open) {
  SendCmdValves(pi::send_dpr_lox_cmd_valves, pi::constants::VALVE_COPV_VENT, open);
}
void Fc_Can_SendDprEthCopvVent(uint8_t open) {
  SendCmdValves(pi::send_dpr_eth_cmd_valves, pi::constants::VALVE_COPV_VENT, open);
}

void Fc_Can_SendDprLoxBallValve(float percent_open) {
  if (g_hfdcan == nullptr) return;
  pi::payload::ball_valve_position pos{};
  pos.percent_open = percent_open;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_lox_ball_valve(&ctx, pos);
}

void Fc_Can_SendDprEthBallValve(float percent_open) {
  if (g_hfdcan == nullptr) return;
  pi::payload::ball_valve_position pos{};
  pos.percent_open = percent_open;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_dpr_eth_ball_valve(&ctx, pos);
}
