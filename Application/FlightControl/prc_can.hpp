#ifndef APP_FC_PRC_CAN_H
#define APP_FC_PRC_CAN_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void Fc_Can_Init(FDCAN_HandleTypeDef *hfdcan);

void Fc_Can_ProcessRxMessage(uint32_t can_id, const uint8_t *data, uint32_t dlc);

// Toggles the engine board's MO/ME valves directly, bypassing the FSM.
void Fc_Can_SendMainValveCmd(uint8_t is_ethanol, uint8_t open);

// Engine bay ignition sequence commands.
void Fc_Can_SendPrcClearToIgnite(void);
void Fc_Can_SendPrcIgnite(void);
void Fc_Can_SendPrcPassivate(void);
void Fc_Can_SendPrcReset(void);
void Fc_Can_SendPrcColdflow(void);

// DPR bench-test commands, role-gated on the receiving board.
void Fc_Can_SendDprLoxPressurize(uint8_t open);
void Fc_Can_SendDprLoxAbort(void);
void Fc_Can_SendDprLoxPassivate(void);
void Fc_Can_SendDprLoxReset(void);
void Fc_Can_SendDprEthPressurize(uint8_t open);
void Fc_Can_SendDprEthAbort(void);
void Fc_Can_SendDprEthPassivate(void);
void Fc_Can_SendDprEthReset(void);
void Fc_Can_SendBroadcastAbort(void);

// Manual valve overrides, only take effect while the DPR board is in State::MANUAL.
void Fc_Can_SendDprLoxVent(uint8_t open);
void Fc_Can_SendDprEthVent(uint8_t open);
void Fc_Can_SendDprLoxSafety(uint8_t open);
void Fc_Can_SendDprEthSafety(uint8_t open);
void Fc_Can_SendDprLoxCopvVent(uint8_t open);
void Fc_Can_SendDprEthCopvVent(uint8_t open);
void Fc_Can_SendDprLoxBallValve(float percent_open);
void Fc_Can_SendDprEthBallValve(float percent_open);

#ifdef __cplusplus
}
#endif

#endif // APP_FC_PRC_CAN_H
