#include "../data.hpp"

using namespace flight_computer;

Batteries::Batteries()
: lpb_voltage(0.0f),
  lpb_current(0.0f),
  vout_5v_voltage(0.0f),
  vout_5v_current(0.0f),
  hpb_main_voltage(0.0f),
  hpb_main_current(0.0f),
  hpb_backup_voltage(0.0f),
  hpb_backup_current(0.0f),
  vout_24v_voltage(0.0f),
  vout_24v_current(0.0f)
{}

BatteriesStore::BatteriesStore() {}

float BatteriesStore::get_lpb_voltage () const {
    return data_.lpb_voltage;
}
void BatteriesStore::set_lpb_voltage (float value) {
    data_.lpb_voltage = value;
}

float BatteriesStore::get_lpb_current () const {
    return data_.lpb_current;
}
void BatteriesStore::set_lpb_current (float value) {
    data_.lpb_current = value;
}

float BatteriesStore::get_vout_5v_voltage () const {
    return data_.vout_5v_voltage;
}
void BatteriesStore::set_vout_5v_voltage (float value) {
    data_.vout_5v_voltage = value;
}

float BatteriesStore::get_vout_5v_current () const {
    return data_.vout_5v_current;
}
void BatteriesStore::set_vout_5v_current (float value) {
    data_.vout_5v_current = value;
}

float BatteriesStore::get_hpb_main_voltage () const {
    return data_.hpb_main_voltage;
}
void BatteriesStore::set_hpb_main_voltage (float value) {
    data_.hpb_main_voltage = value;
}

float BatteriesStore::get_hpb_main_current () const {
    return data_.hpb_main_current;
}
void BatteriesStore::set_hpb_main_current (float value) {
    data_.hpb_main_current = value;
}

float BatteriesStore::get_hpb_backup_voltage () const {
    return data_.hpb_backup_voltage;
}
void BatteriesStore::set_hpb_backup_voltage (float value) {
    data_.hpb_backup_voltage = value;
}

float BatteriesStore::get_hpb_backup_current () const {
    return data_.hpb_backup_current;
}
void BatteriesStore::set_hpb_backup_current (float value) {
    data_.hpb_backup_current = value;
}

float BatteriesStore::get_vout_24v_voltage () const {
    return data_.vout_24v_voltage;
}
void BatteriesStore::set_vout_24v_voltage (float value) {
    data_.vout_24v_voltage = value;
}

float BatteriesStore::get_vout_24v_current () const {
    return data_.vout_24v_current;
}
void BatteriesStore::set_vout_24v_current (float value) {
    data_.vout_24v_current = value;
}