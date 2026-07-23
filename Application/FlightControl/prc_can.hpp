//    FILE: prc_can.hpp
// PURPOSE: Decodes received FDCAN messages (dictionary defined in the
//          prc_intranet submodule's message_list.hpp) coming from the DPR
//          Ethanol / DPR LOX boards into PropSensorsStore. Called from
//          main.c's RX poll loop via the extern "C" shim below.

#ifndef APP_FC_PRC_CAN_H
#define APP_FC_PRC_CAN_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// can_id: 11-bit standard CAN arbitration id, as read from
// FDCAN_RxHeaderTypeDef::Identifier. data: up to 8 bytes, dlc: number of
// valid bytes in data (from FDCAN_RxHeaderTypeDef::DataLength, numerically
// equal to the byte count for classic frames).
//
// Only DPR_ETH_PRESSURES/TEMPS_1 and DPR_LOX_PRESSURES/TEMPS_1 are handled
// right now, feeding PropSensorsStore's fuel_*/LOX_* fields (the
// unambiguous per-board fields). N2_pressure/N2_temperature are NOT
// written here: DPR_ETH and DPR_LOX each have their own COPV, but
// PropSensors has only one shared N2_pressure/N2_temperature field, so
// writing both boards' readings into it would have one silently clobber
// the other. Left as a TODO until that's resolved. Engine PRC telemetry
// (PRC_STATE/P_CHAMBER/P_INJECTOR/T_CHAMBER/T_INJECTOR) and DPR_STATE
// (valve_mask has no defined bit layout) are also not decoded yet. Ids
// this function doesn't recognize are silently ignored, same as the CAN
// bus carrying traffic for other nodes.
void Fc_Can_ProcessRxMessage(uint32_t can_id, const uint8_t *data, uint32_t dlc);

#ifdef __cplusplus
}
#endif

#endif // APP_FC_PRC_CAN_H
