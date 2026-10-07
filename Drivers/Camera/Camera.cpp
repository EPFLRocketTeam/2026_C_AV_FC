
#include "Drivers/Camera/Camera.hpp"
#include "Application/Data/data.hpp"
#include "Application/app_printf.h"
#include "Application/app_logger.hpp"
#include <cstring>

uint32_t CameraInformation::lastPacketTime () {
    return lastPacketReceived_;
}
camera::State CameraInformation::getState () {
    return lastHealthPacket_.cameraState;
}
camera::AvionicsStateMachine CameraInformation::getImposedState () {
    return lastHealthPacket_.avState;
}

bool CameraInformation::isDownlinkOn (uint32_t currentTimeMs) {
    return (currentTimeMs - lastPacketReceived_) <= CameraMsgTimeoutMs && lastPacketReceived_ != 0;
}
bool CameraInformation::isUplinkOn (uint32_t currentTimeMs) {
    return isDownlinkOn(currentTimeMs) && isUplinkOn_;
}

bool CameraInformation::isPoweredOn () {
    return lastHealthPacket_.isPowerOn();
}
bool CameraInformation::isRecording () {
    return lastHealthPacket_.isRecording();
}

bool CameraInformation::isNominal (uint32_t currentTime) {
    return isDownlinkOn(currentTime) && isUplinkOn(currentTime) && isPoweredOn() && isRecording();
}

SingleCameraDump CameraInformation::makeDump (uint32_t currentTime) {
    SingleCameraDump dump;
    dump.cameraState  = lastHealthPacket_.cameraState;
    dump.imposedState = lastHealthPacket_.avState;

    dump.timeSinceLastPacket = currentTime - lastPacketReceived_;
    dump.isPowerOn    = isPoweredOn();
    dump.isRecording  = isRecording();
    dump.isDownlinkOn = isDownlinkOn(currentTime);
    dump.isUplinkOn   = isUplinkOn(currentTime);
    dump.isNominal    = isNominal(currentTime);

    return dump;
}
void CameraInformation::ingest (
    uint32_t timeSinceImposed,
    camera::AvionicsStateMachine stateImposed,

    uint32_t     currentTime,
    HealthPacket packet
) {
    if (timeSinceImposed >= CameraUplinkTimeoutMs
     && packet.avState != stateImposed) {
        isUplinkOn_ = false;
    }
    if (packet.avState == stateImposed
     && lastHealthPacket_.avState != stateImposed
     && lastPacketReceived_ != 0) {
        isUplinkOn_ = true;
    }

    if (lastHealthPacket_.avState != packet.avState && lastPacketReceived_ != 0) {
        app_printf("[%s] imposed state %s -> %s\n", 
            camera_to_string(whoAmI),
            camera_av_state_to_string(lastHealthPacket_.avState), 
            camera_av_state_to_string(packet.avState));
    }
    if (lastHealthPacket_.cameraState != packet.cameraState && lastPacketReceived_ != 0) {
        app_printf("[%s] camera state %s -> %s\n", 
            camera_to_string(whoAmI),
            camera_state_to_string(lastHealthPacket_.cameraState), 
            camera_state_to_string(packet.cameraState));
    }

    lastHealthPacket_   = packet;
    lastPacketReceived_ = currentTime;
}
bool CameraInformation::isSynchronized (camera::AvionicsStateMachine stateImposed) {
    return lastHealthPacket_.avState == stateImposed;
}

void CameraDriver::internalSendStart () {
    if (!did_init_) return ;

    sendMessage_(mask_flight_computer | mask_msg_start, sizeof(CameraStartSequence), CameraStartSequence);
}
void CameraDriver::internalSendStop () {
    if (!did_init_) return ;

    sendMessage_(mask_flight_computer | mask_msg_stop, sizeof(CameraStopSequence), CameraStopSequence);
}
void CameraDriver::internalSendAbort () {
    if (!did_init_) return ;

    sendMessage_(mask_flight_computer | mask_msg_abort, sizeof(CameraAbortSequence), CameraAbortSequence);
}
void CameraDriver::internalSendRecover () {
    if (!did_init_) return ;

    sendMessage_(mask_flight_computer | mask_msg_recover, sizeof(CameraRecoverSequence), CameraRecoverSequence);
}

CameraInformation &CameraDriver::getInformation (Camera camera) {
    return cameraInfo_[static_cast<uint8_t>(camera)];
}

void CameraDriver::processMessage (uint16_t messageId, uint8_t length, const uint8_t* data) {
    if (!did_init_) return ;
    if (length != 3) return ;

    Camera from;

    switch (messageId) {
        case (mask_cam_aero_bot | mask_msg_health):
            from = CAM_AERO_BOT;
            break ;
        case (mask_cam_aero_top | mask_msg_health):
            from = CAM_AERO_TOP;
            break ;
        case (mask_cam_sepmech | mask_msg_health):
            from = CAM_SEPMECH;
            break ;
        default:
            return ;
    }

    HealthPacket packet;
    memcpy(&packet, data, length);

    getInformation(from).ingest(
        stateImposedTickMs_ - getTick_(),
        stateImposed_,

        getTick_(),
        packet
    );
}

void CameraDriver::init (
    camera::PollMessage pollMessage,
    camera::SendMessage sendMessage,
    camera::GetTickMs   getTick
) {
    //app_printf("%p %p %p\n", pollMessage, sendMessage, getTick);
    if (!pollMessage || !sendMessage || !getTick) {
        //app_printf("Failed init.\n");
        while (1) {
            continue ;
        }
        return;
    }
    //app_printf("Did init .\n");

    did_init_ = true;

    pollMessage_ = pollMessage;
    sendMessage_ = sendMessage;
    getTick_     = getTick;
}
void CameraDriver::tick () {
    // app_printf("Did init %d\n", (int) did_init_);
    if (!did_init_) return ;

    CameraDump dump;
    dump.globalImposedState = stateImposed_;
    dump.imposedSince = getTick_() - stateImposedTickMs_;

    dump.aero_bot = getInformation(CAM_AERO_BOT).makeDump(getTick_());
    dump.aero_top = getInformation(CAM_AERO_TOP).makeDump(getTick_());
    dump.sepmech  = getInformation(CAM_SEPMECH).makeDump(getTick_());

    auto &storage = flight_computer::GOATStore::get_instance().camsRecordingStore;
    storage.set_cam_aero_bot(dump.aero_bot.isNominal);
    storage.set_cam_aero_top(dump.aero_top.isNominal);
    storage.set_cam_sepmech (dump.sepmech.isNominal);

    app_get_sd_logger().logCameraDump(dump);

    uint32_t numberPolls = 0;
    while (numberPolls < MaxCameraNumberPolls && pollMessage_()) {
        numberPolls ++;
    }

    // app_printf("Is synchronized: %d\n", (int) isSynchronized());
    if (isSynchronized()) return ;

    switch (stateImposed_) {
        case camera::AV_INIT:
            internalSendRecover(); 
            break;
        case camera::AV_RECORD:
            internalSendStart(); 
            break;
        case camera::AV_STOPPED:
            internalSendStop(); 
            break;
        case camera::AV_ABORT:
            internalSendAbort(); 
            break;
    }
}

void CameraDriver::start () {
    stateImposed_ = camera::AvionicsStateMachine::AV_RECORD;
    stateImposedTickMs_ = getTick_();
}
void CameraDriver::stop () {
    stateImposed_ = camera::AvionicsStateMachine::AV_STOPPED;
    stateImposedTickMs_ = getTick_();
}
void CameraDriver::abort () {
    stateImposed_ = camera::AvionicsStateMachine::AV_ABORT;
    stateImposedTickMs_ = getTick_();
}
void CameraDriver::recover () {
    stateImposed_ = camera::AvionicsStateMachine::AV_INIT;
    stateImposedTickMs_ = getTick_();
}

bool CameraDriver::isNominal (Camera camera) {
    if (!did_init_) {
        return false;
    }

    uint8_t uuid = static_cast<uint8_t>(camera);
    if (uuid >= NumCameras) {
        return false;
    }

    return getInformation(camera).isNominal(getTick_());
}
bool CameraDriver::isSynchronized () {
    for (uint32_t offset = 0; offset < NumCameras; offset ++) {
        CameraInformation &information = getInformation(static_cast<Camera>(offset));
        
        if (!information.isSynchronized(stateImposed_)) {
            return false;
        }
    }

    return true;
}




const char* camera_to_string (Camera camera) {
    switch (camera) {
        case CAM_AERO_BOT: return "CAM_AERO_BOT";
        case CAM_AERO_TOP: return "CAM_AERO_TOP";
        case CAM_SEPMECH: return "CAM_SEPMECH";
    }

    return "camera::???";
}
const char* camera_state_to_string (camera::State state) {
    switch (state) {
        case camera::State::INIT: return "INIT";
        case camera::State::TRY_POWER_ON: return "TRY_POWER_ON";
        case camera::State::WAIT_FOR_ON: return "WAIT_FOR_ON";
        case camera::State::UART_INIT: return "UART_INIT";
        case camera::State::POWER_ON: return "POWER_ON";
        case camera::State::POWER_ON_MANUAL: return "POWER_ON_MANUAL";
        case camera::State::TRY_START: return "TRY_START";
        case camera::State::WAIT_FOR_START: return "WAIT_FOR_START";
        case camera::State::RECORDING: return "RECORDING";
        case camera::State::RECORDING_MANUAL: return "RECORDING_MANUAL";
        case camera::State::TRY_STOP: return "TRY_STOP";
        case camera::State::WAIT_FOR_STOP: return "WAIT_FOR_STOP";
        case camera::State::ENDED: return "ENDED";
        case camera::State::ABORT_ON_POWER_ON: return "ABORT_ON_POWER_ON";
        case camera::State::ABORT_ON_START: return "ABORT_ON_START";
        case camera::State::ABORT_ON_STOP: return "ABORT_ON_STOP";
    }
    return "camera::State::???";
}
const char* camera_av_state_to_string (camera::AvionicsStateMachine avState) {
    switch (avState) {
        case camera::AvionicsStateMachine::AV_INIT: return "INIT";
        case camera::AvionicsStateMachine::AV_RECORD: return "RECORD";
        case camera::AvionicsStateMachine::AV_STOPPED: return "STOPPED";
        case camera::AvionicsStateMachine::AV_ABORT: return "ABORT";
    }

    return "camera::AvionicsStateMachine::???";
}

void CameraDriver::display (Camera camera, bool displayGlobalHeader) {
    if (!did_init_) {
        app_printf("Forgot to init camera driver.\n");
        return ;
    }

    if (displayGlobalHeader) {
        app_printf("Camera Driver Status\n");
        app_printf(" Target State = %s\n", camera_av_state_to_string(stateImposed_));
        app_printf(" Since = %d ms\n", (int) (getTick_() - stateImposedTickMs_));
        app_printf("\n");
    }

    uint32_t currentTime = getTick_();

    CameraInformation &information = getInformation(camera);

    app_printf("Camera %s\n", camera_to_string(camera));
    app_printf(" State   = %s\n", camera_state_to_string(information.getState()));
    app_printf(" Imposed = %s\n", camera_av_state_to_string(information.getImposedState()));
    app_printf(" Link: ");
    if (information.isDownlinkOn(currentTime)) app_printf("DOWNLINK ");
    if (information.isUplinkOn(currentTime)) app_printf("UPLINK");
    app_printf("\n");
    app_printf(" Power     = %d\n", information.isPoweredOn());
    app_printf(" Recording = %d\n", information.isRecording());
    app_printf(" Nominal   = %d\n", information.isNominal(currentTime));
    app_printf("\n");
}

void CameraDriver::displayAll () {
    display(CAM_AERO_BOT);
    display(CAM_AERO_TOP, false);
    display(CAM_SEPMECH, false);
}
