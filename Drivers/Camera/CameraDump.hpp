#pragma once
#include "Drivers/Camera/Intranet.hpp"

struct SingleCameraDump {
    camera::State cameraState;

    /* Imposed State that is currently stored by camera */
    camera::AvionicsStateMachine imposedState;
    
    uint32_t timeSinceLastPacket;

    bool isDownlinkOn;
    bool isUplinkOn;
    bool isPowerOn;
    bool isRecording;
    bool isNominal;
};

struct CameraDump {
    SingleCameraDump aero_bot;
    SingleCameraDump aero_top;
    SingleCameraDump sepmech;

    camera::AvionicsStateMachine globalImposedState;
    uint32_t imposedSince;
};
