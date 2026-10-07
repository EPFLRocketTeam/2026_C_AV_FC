#include "../data.hpp"

using namespace flight_computer;

Batteries::Batteries()
: lpb1_voltage(0.0f),
  lpb1_current(0.0f),
  lpb2_voltage(0.0f),
  lpb2_current(0.0f),
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

float BatteriesStore::get_lpb1_voltage () const {
    return data_.lpb1_voltage;
}
void BatteriesStore::set_lpb1_voltage (float value) {
    data_.lpb1_voltage = value;
}

float BatteriesStore::get_lpb2_voltage () const {
    return data_.lpb2_voltage;
}
void BatteriesStore::set_lpb2_voltage (float value) {
    data_.lpb2_voltage = value;
}

float BatteriesStore::get_lpb1_current () const {
    return data_.lpb1_current;
}
void BatteriesStore::set_lpb1_current (float value) {
    data_.lpb1_current = value;
}

float BatteriesStore::get_lpb2_current () const {
    return data_.lpb2_current;
}
void BatteriesStore::set_lpb2_current (float value) {
    data_.lpb2_current = value;
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