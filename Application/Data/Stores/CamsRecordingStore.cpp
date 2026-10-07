#include "../data.hpp"

using namespace flight_computer;

CamsRecording::CamsRecording()
:   cam_sepmech(false),
    cam_aero_top(false),
    cam_aero_bot(false)
{}

CamsRecordingStore::CamsRecordingStore() {}

bool CamsRecordingStore::get_cam_sepmech() const { return data_.cam_sepmech; }
void CamsRecordingStore::set_cam_sepmech(bool value) { data_.cam_sepmech = value; }

bool CamsRecordingStore::get_cam_aero_top() const { return data_.cam_aero_top; }
void CamsRecordingStore::set_cam_aero_top(bool value) { data_.cam_aero_top = value; }

bool CamsRecordingStore::get_cam_aero_bot() const { return data_.cam_aero_bot; }
void CamsRecordingStore::set_cam_aero_bot(bool value) { data_.cam_aero_bot = value; }
