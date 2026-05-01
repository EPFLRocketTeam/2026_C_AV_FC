#include "../data.hpp"

using namespace flight_computer;

Batteries::Batteries()
:   lpb_voltage(0.0f),
    hpb_voltage(0.0f)
{}

BatteriesStore::BatteriesStore() {}

float BatteriesStore::get_lpb_voltage() const { return data_.lpb_voltage; }
void BatteriesStore::set_lpb_voltage(float value) { data_.lpb_voltage = value; }

float BatteriesStore::get_hpb_voltage() const { return data_.hpb_voltage; }
void BatteriesStore::set_hpb_voltage(float value) { data_.hpb_voltage = value; }
