//    FILE: prc_can.hpp
// PURPOSE: Decodes received FDCAN messages (dictionary defined in the
//          prc_intranet submodule's message_list.hpp) coming from the DPR
//          Ethanol / DPR LOX boards into PropSensorsStore. Called from
//          main.c's RX poll loop via the extern "C" shim below.

#ifndef APP_FC_PRC_CAN_H
#define APP_FC_PRC_CAN_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Latches the FDCAN handle used by Fc_Can_SendMainValveCmd below. Call
// once, after MX_FDCAN1_Init(), before any command is sent.
void Fc_Can_Init(FDCAN_HandleTypeDef *hfdcan);

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

// Bench-test hookup: sends dpr_lox_cmd_valves for the bench board's spare
// Sol3/Sol4 channels, repurposed as "LOX main"/"Ethanol main" (see
// 2026_C_AV_PRC's prc_can.cpp ApplyCmdValves). Hardcoded to the DprLox
// message id since that's this bench board's current role -- not a real
// mission valve mapping yet. is_ethanol selects Sol4 (Ethanol main) vs
// Sol3 (LOX main); open selects VALVE_STATE_OPEN vs _CLOSED.
void Fc_Can_SendMainValveCmd(uint8_t is_ethanol, uint8_t open);

// Engine bay (PRC-P) ignition sequence commands -- see 2026_C_AV_PRC's
// engine_state.cpp/"Propulsion Computer FSM" diagram for what each one
// does on the receiving end. Manual/bench-test hookups for now (driven
// from fc_shell.cpp's "prc clear_to_ignite"/"prc ignite"/"prc
// passivate"/"prc reset" commands), not yet tied to AvState's own
// IGNITION state -- see av_state.cpp's fromIgnition().
void Fc_Can_SendPrcClearToIgnite(void);
void Fc_Can_SendPrcIgnite(void);
void Fc_Can_SendPrcPassivate(void);
void Fc_Can_SendPrcReset(void);

// DPR (LOX/ETH) bench-test commands -- exercises 2026_C_AV_PRC's
// prc_state.cpp PrcState FSM (dpr_lox_*/dpr_eth_* fromXxx() transitions),
// role-gated on the receiving board's own BoardIdentity. Manual/bench-test
// hookups driven from fc_shell.cpp's "dpr lox/eth ..." commands, not yet
// tied to any real ground-station sequence.
void Fc_Can_SendDprLoxPressurize(uint8_t open);
void Fc_Can_SendDprLoxAbort(void);
void Fc_Can_SendDprLoxPassivate(void);
void Fc_Can_SendDprLoxReset(void);
void Fc_Can_SendDprEthPressurize(uint8_t open);
void Fc_Can_SendDprEthAbort(void);
void Fc_Can_SendDprEthPassivate(void);
void Fc_Can_SendDprEthReset(void);
void Fc_Can_SendBroadcastAbort(void);

// Ground-station manual valve commands (VENT LOX/FUEL, PRESSURE LOX/FUEL,
// VENT_COPV) -- exercises the same cmd_valves message as
// Fc_Can_SendMainValveCmd, but targeting VALVE_SAFETY/VALVE_VENT/
// VALVE_COPV_VENT instead of the Sol3/Sol4 bench channels. See
// 2026_C_AV_PRC's prc_can.cpp ApplyCmdValves: only takes effect while the
// receiving DPR board's FSM is in State::MANUAL, so it can't fight the
// FSM's own valve control once a mission sequence is running. VENT_COPV
// has no dedicated valve -- N2/COPV venting is a bundled Vent+Safety+
// ball-valve sequence (see Prc_Fsm_ManualVentCopv), sent to both DPR
// boards from fc_shell.cpp's OnVentCopv since it isn't board-specific.
void Fc_Can_SendDprLoxVent(uint8_t open);
void Fc_Can_SendDprEthVent(uint8_t open);
void Fc_Can_SendDprLoxSafety(uint8_t open);
void Fc_Can_SendDprEthSafety(uint8_t open);
void Fc_Can_SendDprLoxCopvVent(uint8_t open);
void Fc_Can_SendDprEthCopvVent(uint8_t open);

#ifdef __cplusplus
}
#endif

#endif // APP_FC_PRC_CAN_H
