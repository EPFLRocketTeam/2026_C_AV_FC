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
// Handles: DPR_ETH_PRESSURES/DPR_LOX_PRESSURES, feeding PropSensorsStore's
// fuel_pressure/LOX_pressure (tank pressure only; COPV pressure isn't
// written, no field for it yet). Also LOG_CHUNK_PRC_ENGINE/DPR_ETH/DPR_LOX,
// reassembled via log_aggregator and printed out FC's own local VCP
// tagged [PRC-ENGINE]/[PRC-ETH]/[PRC-LOX] (see prc_can.cpp).
//
// Not decoded yet: DPR_LOX_TEMPS_OTA_1_2/3_4, DPR_STATE, engine PRC
// telemetry. Ids this function doesn't recognize are silently ignored,
// same as the CAN bus carrying traffic for other nodes.
void Fc_Can_ProcessRxMessage(uint32_t can_id, const uint8_t *data, uint32_t dlc);

#ifdef __cplusplus
}
#endif

#endif // APP_FC_PRC_CAN_H
