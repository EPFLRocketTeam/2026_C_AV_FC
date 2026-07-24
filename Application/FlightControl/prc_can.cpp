#include "Application/FlightControl/prc_can.hpp"

#include <cstdio>

#include "Application/Data/data.hpp"
#include "log_aggregator/reassembler.hpp"
#include "prc_intranet/dispatch.hpp"

using namespace flight_computer;
namespace pi = prc_intranet;

namespace {

// One reassembler per source board -- each owns its own 128-byte buffer,
// see log_aggregator/reassembler.hpp's warning against sharing one
// instance across sources.
log_aggregator::LineReassembler g_log_prc_engine;
log_aggregator::LineReassembler g_log_dpr_eth;
log_aggregator::LineReassembler g_log_dpr_lox;

// ctx is the source's tag string (e.g. "[PRC-ETH]"), passed straight
// through from whichever Feed* call below invoked feed(). Goes out FC's
// own local VCP via printf/_write, same path FC's own logs already use
// -- safe from recursion since receiving a CAN frame and printing here
// never triggers another CAN send on this board.
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
  auto& sensors = GOATStore::get_instance().propSensorsStore;
  sensors.set_fuel_pressure(p.p_eta);
  printf("[FC CAN] RX dpr_eth_pressures: tank=%.2f copv=%.2f\r\n", p.p_eta, p.p_hpe);
}
void OnDprLoxPressures(void*, pi::payload::dpr_lox_pressures p) noexcept {
  auto& sensors = GOATStore::get_instance().propSensorsStore;
  sensors.set_LOX_pressure(p.p_ota);
  printf("[FC CAN] RX dpr_lox_pressures: tank=%.2f copv=%.2f\r\n", p.p_ota, p.p_hpo);
}

pi::context& Ctx() {
  static pi::context ctx = [] {
    pi::prc_driver driver{};
    driver.on_dpr_eth_pressures = OnDprEthPressures;
    driver.on_dpr_lox_pressures = OnDprLoxPressures;
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
