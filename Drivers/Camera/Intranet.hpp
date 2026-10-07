
#pragma once
#include <cstdint>
#include <cstddef>

namespace camera {
    enum State : uint8_t {
        // Init and setup power.
        INIT,
        TRY_POWER_ON,
        WAIT_FOR_ON,
        UART_INIT,

        // Power is on.
        POWER_ON,
        POWER_ON_MANUAL,

        // Start Recording.
        TRY_START,
        WAIT_FOR_START,

        // Recording.
        RECORDING,
        RECORDING_MANUAL,

        // Stop Recording.
        TRY_STOP,
        WAIT_FOR_STOP,

        // Ended (Power off)
        ENDED,

        // Aborts
        ABORT_ON_POWER_ON,
        ABORT_ON_START,
        ABORT_ON_STOP
    };

    enum AvionicsStateMachine : uint8_t {
        AV_INIT,
        AV_RECORD,
        AV_STOPPED,
        AV_ABORT
    };
};

struct HealthPacket {
    camera::State cameraState;
    camera::AvionicsStateMachine avState;
    uint8_t statusMask;

    bool isRecording () { return (statusMask & 1) != 0; }
    bool isPowerOn   () { return (statusMask & 2) != 0; }
};
static_assert(sizeof(HealthPacket) == 3);
