#include "Application/FlightControl/prc_can.hpp"

#include "Application/Data/data.hpp"
#include "prc_intranet/dispatch.hpp"

using namespace flight_computer;
namespace pi = prc_intranet;

namespace {

void OnDprEthPressures(void*, pi::payload::dpr_pressures p) noexcept {
  auto& sensors = GOATStore::get_instance().propSensorsStore;
  sensors.set_fuel_pressure(p.p_xta);
  // p.p_nco (COPV) intentionally not written, see prc_can.hpp.
}
void OnDprEthTemps1(void*, pi::payload::dpr_temps_1 t) noexcept {
  auto& sensors = GOATStore::get_instance().propSensorsStore;
  sensors.set_fuel_temperature(t.t_xta);
  // t.t_nco (COPV) intentionally not written, see prc_can.hpp.
}
void OnDprLoxPressures(void*, pi::payload::dpr_pressures p) noexcept {
  auto& sensors = GOATStore::get_instance().propSensorsStore;
  sensors.set_LOX_pressure(p.p_xta);
  // p.p_nco (COPV) intentionally not written, see prc_can.hpp.
}
void OnDprLoxTemps1(void*, pi::payload::dpr_temps_1 t) noexcept {
  auto& sensors = GOATStore::get_instance().propSensorsStore;
  sensors.set_LOX_temperature(t.t_xta);
  // t.t_nco (COPV) intentionally not written, see prc_can.hpp.
}

pi::context& Ctx() {
  static pi::context ctx = [] {
    pi::prc_driver driver{};
    driver.on_dpr_eth_pressures = OnDprEthPressures;
    driver.on_dpr_eth_temps_1   = OnDprEthTemps1;
    driver.on_dpr_lox_pressures = OnDprLoxPressures;
    driver.on_dpr_lox_temps_1   = OnDprLoxTemps1;
    return pi::create_context(driver);
  }();
  return ctx;
}

} // namespace

void Fc_Can_ProcessRxMessage(uint32_t can_id, const uint8_t *data, uint32_t dlc) {
  pi::dispatch_frame(&Ctx(), static_cast<uint16_t>(can_id), data, dlc);
}
