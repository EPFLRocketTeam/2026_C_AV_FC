#include "../data.hpp"

using namespace flight_computer;

CamsRecording::CamsRecording()
:   cam_sep(false),
    cam_up(false),
    cam_down(false)
{}

CamsRecordingStore::CamsRecordingStore() {}

bool CamsRecordingStore::get_cam_sep() const { return data_.cam_sep; }
void CamsRecordingStore::set_cam_sep(bool value) { data_.cam_sep = value; }

bool CamsRecordingStore::get_cam_up() const { return data_.cam_up; }
void CamsRecordingStore::set_cam_up(bool value) { data_.cam_up = value; }

bool CamsRecordingStore::get_cam_down() const { return data_.cam_down; }
void CamsRecordingStore::set_cam_down(bool value) { data_.cam_down = value; }
