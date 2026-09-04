#include "Application/Data/data.hpp"

using namespace flight_computer;

PropSensors::PropSensors()
:   HPE_pressure(0.0f),
    ETA_pressure(0.0f),
    HPO_pressure(0.0f),
    OTA_pressure(0.0f),
    fls_OTA_temperature_1(0.0f),
    fls_OTA_temperature_2(0.0f),
    fls_OTA_temperature_3(0.0f),
    fls_OTA_temperature_4(0.0f),
    fls_OTA_temperature_5(0.0f),
    fuel_inj_pressure(0.0f),
    fuel_inj_temperature(0.0f),
    LOX_inj_pressure(0.0f),
    LOX_inj_temperature(0.0f),
    chamber_pressure(0.0f),
    chamber_temperature(0.0f),
    dpr_fuel_state(0),
    dpr_LOX_state(0),
    engine_state(0),
    timer_burn(0)
{}

PropSensorsStore::PropSensorsStore() {}

double PropSensorsStore::get_HPE_pressure () const { return data_.HPE_pressure; }
void PropSensorsStore::set_HPE_pressure (double value) { data_.HPE_pressure = value; }

double PropSensorsStore::get_ETA_pressure () const { return data_.ETA_pressure; }
void PropSensorsStore::set_ETA_pressure (double value) { data_.ETA_pressure = value; }

double PropSensorsStore::get_HPO_pressure () const { return data_.HPO_pressure; }
void PropSensorsStore::set_HPO_pressure (double value) { data_.HPO_pressure = value; }

double PropSensorsStore::get_OTA_pressure () const { return data_.OTA_pressure; }
void PropSensorsStore::set_OTA_pressure (double value) { data_.OTA_pressure = value; }

double PropSensorsStore::get_fls_OTA_temperature_1 () const { return data_.fls_OTA_temperature_1; }
void PropSensorsStore::set_fls_OTA_temperature_1 (double value) { data_.fls_OTA_temperature_1 = value; }

double PropSensorsStore::get_fls_OTA_temperature_2 () const { return data_.fls_OTA_temperature_2; }
void PropSensorsStore::set_fls_OTA_temperature_2 (double value) { data_.fls_OTA_temperature_2 = value; }

double PropSensorsStore::get_fls_OTA_temperature_3 () const { return data_.fls_OTA_temperature_3; }
void PropSensorsStore::set_fls_OTA_temperature_3 (double value) { data_.fls_OTA_temperature_3 = value; }

double PropSensorsStore::get_fls_OTA_temperature_4 () const { return data_.fls_OTA_temperature_4; }
void PropSensorsStore::set_fls_OTA_temperature_4 (double value) { data_.fls_OTA_temperature_4 = value; }

double PropSensorsStore::get_fls_OTA_temperature_5 () const { return data_.fls_OTA_temperature_5; }
void PropSensorsStore::set_fls_OTA_temperature_5 (double value) { data_.fls_OTA_temperature_5 = value; }

double PropSensorsStore::get_fuel_inj_pressure () const { return data_.fuel_inj_pressure; }
void PropSensorsStore::set_fuel_inj_pressure (double value) { data_.fuel_inj_pressure = value; }

double PropSensorsStore::get_fuel_inj_temperature () const { return data_.fuel_inj_temperature; }
void PropSensorsStore::set_fuel_inj_temperature (double value) { data_.fuel_inj_temperature = value; }

double PropSensorsStore::get_LOX_inj_pressure () const { return data_.LOX_inj_pressure; }
void PropSensorsStore::set_LOX_inj_pressure (double value) { data_.LOX_inj_pressure = value; }

double PropSensorsStore::get_LOX_inj_temperature () const { return data_.LOX_inj_temperature; }
void PropSensorsStore::set_LOX_inj_temperature (double value) { data_.LOX_inj_temperature = value; }

double PropSensorsStore::get_chamber_pressure () const { return data_.chamber_pressure; }
void PropSensorsStore::set_chamber_pressure (double value) { data_.chamber_pressure = value; }

double PropSensorsStore::get_chamber_temperature () const { return data_.chamber_temperature; }
void PropSensorsStore::set_chamber_temperature (double value) { data_.chamber_temperature = value; }

uint8_t PropSensorsStore::get_dpr_fuel_state () const { return data_.dpr_fuel_state; }
void PropSensorsStore::set_dpr_fuel_state (uint8_t value) { data_.dpr_fuel_state = value; }

uint8_t PropSensorsStore::get_dpr_LOX_state () const { return data_.dpr_LOX_state; }
void PropSensorsStore::set_dpr_LOX_state (uint8_t value) { data_.dpr_LOX_state = value; }

uint8_t PropSensorsStore::get_engine_state () const { return data_.engine_state; }
void PropSensorsStore::set_engine_state (uint8_t value) { data_.engine_state = value; }

uint32_t PropSensorsStore::get_timer_burn () const { return data_.timer_burn; }
void PropSensorsStore::set_timer_burn (uint32_t value) { data_.timer_burn = value; }