
#pragma once
#include <cstddef>
#include <cstdint>
#include "Drivers/Camera/Intranet.hpp"
#include "Drivers/Camera/CameraDump.hpp"

enum Camera : uint8_t {
    CAM_AERO_BOT = 0b00,
    CAM_AERO_TOP = 0b01,
    CAM_SEPMECH  = 0b10
};

// If no message has been received in the last 20 seconds
//  the camera will be assumed to be off. Originally was
//  5 seconds, but the payload has at most a 12 seconds
//  delay between two health sends
constexpr uint32_t CameraMsgTimeoutMs    = 20000;
constexpr uint32_t CameraUplinkTimeoutMs = 15000;

constexpr uint32_t mask_msg_start   = 0b111000000 << 2;
constexpr uint32_t mask_msg_stop    = 0b000111000 << 2;
constexpr uint32_t mask_msg_abort   = 0b101010000 << 2;
constexpr uint32_t mask_msg_recover = 0b010101111 << 2;
constexpr uint32_t mask_msg_health  = 0b000000111 << 2;

constexpr uint32_t mask_cam_aero_bot    = CAM_AERO_BOT;
constexpr uint32_t mask_cam_aero_top    = CAM_AERO_TOP;
constexpr uint32_t mask_cam_sepmech     = CAM_SEPMECH;
constexpr uint32_t mask_flight_computer = 0b11;

constexpr uint32_t NumCameras           = 3;
constexpr uint32_t MaxCameraNumberPolls = 8;
constexpr uint32_t MaxCameraMessageSize = 8;

constexpr uint8_t CameraStartSequence  [2] = { 0x42, 0xA5 };
constexpr uint8_t CameraStopSequence   [2] = { 0xA5, 0x42 };
constexpr uint8_t CameraAbortSequence  [2] = { 0xDE, 0xAD };
constexpr uint8_t CameraRecoverSequence[2] = { 0xCA, 0xFE };

namespace camera {
    using PollMessage = bool (*)();
    using SendMessage = void (*)(uint16_t messageId, uint8_t length, const uint8_t* data);
    using GetTickMs   = uint32_t (*)();
};

struct CameraInformation {
private:
    Camera whoAmI;

    uint32_t     lastPacketReceived_ = 0;
    HealthPacket lastHealthPacket_ {};

    /* Guess of whether the uplink looks ok
     * Based on the following estimate :
     *  - if more than 30 seconds after AV asked the camera
     *    for a new state (and multiple repings), nothing
     *    changes in the imposed state, turns to false.
     *  - if on the other hand at some point AV asks for a change
     *    and it becomes correct, the uplink switches back to true.
     * 
     * Used to determine whether the system looks nominal.
     */
    bool isUplinkOn_ = true;
public:
    CameraInformation (Camera whoami) : whoAmI(whoami) {}

    uint32_t                     lastPacketTime ();
    camera::State                getState ();
    camera::AvionicsStateMachine getImposedState ();

    bool isDownlinkOn (uint32_t currentTimeMs);
    bool isUplinkOn (uint32_t currentTimeMs);

    /* Returns whether the last packet said it was powered on */
    bool isPoweredOn ();
    /* Returns whether the last packet said it was recording */
    bool isRecording ();

    /* Returns whether the camera is in its nominal state,
       that is if its downlink is on, it is powered on,
       it is recording and its uplink is on. */
    bool isNominal (uint32_t currentTime);

    void ingest (
        uint32_t             timeSinceImposed,
        camera::AvionicsStateMachine stateImposed,

        uint32_t     currentTime,
        HealthPacket packet
    );

    bool isSynchronized (camera::AvionicsStateMachine stateImposed);

    SingleCameraDump makeDump (uint32_t currentTime);
};

struct CameraDriver {
private:
    uint32_t stateImposedTickMs_ = 0;
    camera::AvionicsStateMachine stateImposed_ = camera::AvionicsStateMachine::AV_INIT;

    CameraInformation cameraInfo_[NumCameras] = {
        CameraInformation(CAM_AERO_BOT),
        CameraInformation(CAM_AERO_TOP),
        CameraInformation(CAM_SEPMECH)
    };
    
    camera::PollMessage pollMessage_ = nullptr;
    camera::SendMessage sendMessage_ = nullptr;
    camera::GetTickMs   getTick_     = nullptr;
    bool did_init_ = false;

    void internalSendStart ();
    void internalSendStop ();
    void internalSendAbort ();
    void internalSendRecover ();

    /* Unsafe method if camera >= NumCameras */
    CameraInformation& getInformation (Camera camera);
public:
    CameraDriver () = default;
    
    void processMessage (uint16_t messageId, uint8_t length, const uint8_t* data);

    void init (
        camera::PollMessage pollMessage,
        camera::SendMessage sendMessage,
        camera::GetTickMs   getTick
    );
    void tick ();

    void start ();
    void stop ();
    void abort ();
    void recover ();

    bool isSynchronized ();
    bool isNominal (Camera camera);

    void display (Camera camera, bool displayGlobalHeader = true);
    void displayAll ();
};

const char* camera_to_string          (Camera camera);
const char* camera_state_to_string    (camera::State state);
const char* camera_av_state_to_string (camera::AvionicsStateMachine avState);

extern CameraDriver cameraDriver;
