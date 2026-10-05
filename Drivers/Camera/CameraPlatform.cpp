
#include "Drivers/Camera/Camera.hpp"
#include "Drivers/Camera/CameraPlatform.hpp"
#include "Application/app_timebase.h"
#include "stm32h7xx_hal.h"

extern FDCAN_HandleTypeDef hfdcan2;

namespace {

constexpr uint32_t kDlc[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8
};

uint8_t dlcToLength(uint32_t dlc) {
    for (uint8_t i = 0; i <= 8; i++) {
        if (kDlc[i] == dlc) return i;
    }
    return 0xFF;
}

}

uint32_t cameraGetTickMs() {
    return HAL_GetTick();
}

void cameraSendMessage(uint16_t messageId, uint8_t length, const uint8_t* data) {
    if (length > 8) return;

    FDCAN_TxHeaderTypeDef header = {};
    header.Identifier          = messageId;
    header.IdType              = FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = kDlc[length];
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0;

    (void)HAL_FDCAN_AddMessageToTxFifoQ(
        &hfdcan2, &header, const_cast<uint8_t*>(data));
}

bool cameraPollMessage() {
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) == 0) {
        return false;
    }

    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &header, data) != HAL_OK) {
        return false;
    }

    if (header.IdType == FDCAN_STANDARD_ID &&
        header.RxFrameType == FDCAN_DATA_FRAME) {
        uint8_t length = dlcToLength(header.DataLength);
        if (length <= 8) {
            cameraDriver.processMessage(
                static_cast<uint16_t>(header.Identifier), length, data);
        }
    }
    return true;
}

uint32_t cameraGetTick () {
    return HAL_GetTick();
}

void cameraSetup () {
    cameraDriver.init(cameraPollMessage, cameraSendMessage, cameraGetTick);
}
void cameraTick () {
    cameraPollMessage();
    
    RUN_EVERY(1000) {
        cameraDriver.tick();
    }
}
