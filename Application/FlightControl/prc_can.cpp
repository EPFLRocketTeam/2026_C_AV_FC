#include "Application/FlightControl/prc_can.hpp"

#include <cstdio>
#include <cstring>

#include "Application/Data/data.hpp"
#include "log_aggregator/reassembler.hpp"
#include "prc_intranet/const.hpp"
#include "prc_intranet/dispatch.hpp"
#include "prc_intranet/transmit.hpp"
#include "Application/Config/config.hpp"

using namespace flight_computer;
namespace pi = prc_intranet;

namespace {

FDCAN_HandleTypeDef* g_hfdcan = nullptr;

uint32_t get_fdcan_dlc(uint8_t bytes) {
    switch (bytes) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return FDCAN_DLC_BYTES_8; // Fallback
    }
}

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

  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData);

  if (status != HAL_OK) {
      uint32_t error_code = HAL_FDCAN_GetError(hfdcan);
      uint32_t state      = HAL_FDCAN_GetState(hfdcan);
      uint32_t fifo_fill  = HAL_FDCAN_GetTxFifoFreeLevel(hfdcan);

      printf("[FC CAN] TX failed (Status: %d, State: 0x%X, Error: 0x%X, Free FIFO: %d)\r\n", 
            status, state, error_code, fifo_fill);
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
  GOATStore::get_instance().propSensorsStore.set_ETA_pressure(p.p_eta);
  GOATStore::get_instance().propSensorsStore.set_HPE_pressure(p.p_hpe);
}
void OnDprLoxPressures(void*, pi::payload::dpr_lox_pressures p) noexcept {
  GOATStore::get_instance().propSensorsStore.set_OTA_pressure(p.p_ota);
  GOATStore::get_instance().propSensorsStore.set_HPO_pressure(p.p_hpo);
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
  GOATStore::get_instance().valvesStore.set_main_LOX_open(mo_open);
  GOATStore::get_instance().valvesStore.set_main_fuel_open(me_open);
}

void OnDprLoxState (void*, pi::payload::dpr_state state) noexcept {
  const bool safety_open = (state.valve_mask & pi::constants::VALVE_MASK_BIT_SAFETY) != 0;
  const bool vent_open = (state.valve_mask & pi::constants::VALVE_MASK_BIT_VENT) != 0;
  GOATStore::get_instance().valvesStore.set_safety_LOX_open(safety_open);
  GOATStore::get_instance().valvesStore.set_vent_LOX_open(vent_open);
}
void OnDprEthState (void*, pi::payload::dpr_state state) noexcept {
  const bool safety_open = (state.valve_mask & pi::constants::VALVE_MASK_BIT_SAFETY) != 0;
  const bool vent_open = (state.valve_mask & pi::constants::VALVE_MASK_BIT_VENT) != 0;
  GOATStore::get_instance().valvesStore.set_safety_fuel_open(safety_open);
  GOATStore::get_instance().valvesStore.set_vent_fuel_open(vent_open);
}

void OnConfigCrcDprEth (void*, pi::payload::config_crc crc) noexcept {
  config::internal::On_Crc(BoardIds::FP_PRC_ETH, crc.crc_buffer, crc.crc_commited);
}
void OnConfigCrcDprLox (void*, pi::payload::config_crc crc) noexcept {
  config::internal::On_Crc(BoardIds::FP_PRC_LOX, crc.crc_buffer, crc.crc_commited);
}
void OnConfigCrcEngine (void*, pi::payload::config_crc crc) noexcept {
  config::internal::On_Crc(BoardIds::FP_ENGINE, crc.crc_buffer, crc.crc_commited);
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
    driver.on_dpr_lox_state     = OnDprLoxState;
    driver.on_dpr_eth_state     = OnDprEthState;
    driver.on_log_chunk_prc_engine  = OnLogChunkPrcEngine;
    driver.on_log_chunk_dpr_eth     = OnLogChunkDprEth;
    driver.on_log_chunk_dpr_lox     = OnLogChunkDprLox;
    driver.on_config_crc_dpr_eth    = OnConfigCrcDprEth;
    driver.on_config_crc_dpr_lox    = OnConfigCrcDprLox;
    driver.on_config_crc_prc_engine = OnConfigCrcEngine;
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

void Fc_Can_SendLogDprEth (bool can, bool enabled) {
  if (g_hfdcan == nullptr) return ;
  pi::payload::log_config conf;
  conf.board = pi::payload::board_id::DPR_ETH;
  conf.channel = can ? pi::payload::log_channel::CAN : pi::payload::log_channel::USB;
  conf.enabled = enabled;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_log_config(&ctx, conf);
}
void Fc_Can_SendLogDprLox (bool can, bool enabled) {
  if (g_hfdcan == nullptr) return ;
  pi::payload::log_config conf;
  conf.board = pi::payload::board_id::DPR_LOX;
  conf.channel = can ? pi::payload::log_channel::CAN : pi::payload::log_channel::USB;
  conf.enabled = enabled;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_log_config(&ctx, conf);
}
void Fc_Can_SendLogEngine (bool can, bool enabled) {
  if (g_hfdcan == nullptr) return ;
  pi::payload::log_config conf;
  conf.board = pi::payload::board_id::ENGINE;
  conf.channel = can ? pi::payload::log_channel::CAN : pi::payload::log_channel::USB;
  conf.enabled = enabled;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_log_config(&ctx, conf);
}

void config::internal::Fc_Can_SendChunkLog (prc_intranet::payload::config_chunk chunk) {
  if (g_hfdcan == nullptr) return ;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::send_config_send_data(&ctx, chunk);
}
void config::internal::Fc_Can_SendCommit (BoardIds ids) {
  if (g_hfdcan == nullptr) return ;
  pi::context& ctx = Ctx();
  ctx.driver.driver_ptr = g_hfdcan;
  pi::payload::config_commit c;
  if (ids == BoardIds::FP_ENGINE) c.board = pi::payload::board_id::ENGINE;
  else if (ids == BoardIds::FP_PRC_LOX) c.board = pi::payload::board_id::DPR_LOX;
  else if (ids == BoardIds::FP_PRC_ETH) c.board = pi::payload::board_id::DPR_ETH;
  else return ;
  pi::send_config_send_commit(&ctx, c);
}
