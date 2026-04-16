#include "Application/Data/data.hpp"

using namespace flight_computer;

PropSensors::PropSensors()
:   N2_pressure(0.0),
    fuel_pressure(0.0),
    LOX_pressure(0.0),
    igniter_pressure(0.0),
    LOX_inj_pressure(0.0),
    fuel_inj_pressure(0.0),
    chamber_pressure(0.0),
    fuel_level(0.0),
    LOX_level(0.0),
    N2_temperature(0.0),
    fuel_temperature(0.0),
    LOX_temperature(0.0),
    igniter_temperature(0.0),
    fuel_inj_temperature(0.0),
    fuel_inj_cooling_temperature(0.0),
    LOX_inj_temperature(0.0),
    chamber_temperature(0.0),
    PR_state(0)
{}

PropSensorsStore::PropSensorsStore() {}

double PropSensorsStore::get_N2_pressure() const { return data_.N2_pressure; }
void PropSensorsStore::set_N2_pressure(double value) { data_.N2_pressure = value; }

double PropSensorsStore::get_fuel_pressure() const { return data_.fuel_pressure; }
void PropSensorsStore::set_fuel_pressure(double value) { data_.fuel_pressure = value; }

double PropSensorsStore::get_LOX_pressure() const { return data_.LOX_pressure; }
void PropSensorsStore::set_LOX_pressure(double value) { data_.LOX_pressure = value; }

double PropSensorsStore::get_igniter_pressure() const { return data_.igniter_pressure; }
void PropSensorsStore::set_igniter_pressure(double value) { data_.igniter_pressure = value; }

double PropSensorsStore::get_LOX_inj_pressure() const { return data_.LOX_inj_pressure; }
void PropSensorsStore::set_LOX_inj_pressure(double value) { data_.LOX_inj_pressure = value; }

double PropSensorsStore::get_fuel_inj_pressure() const { return data_.fuel_inj_pressure; }
void PropSensorsStore::set_fuel_inj_pressure(double value) { data_.fuel_inj_pressure = value; }

double PropSensorsStore::get_chamber_pressure() const { return data_.chamber_pressure; }
void PropSensorsStore::set_chamber_pressure(double value) { data_.chamber_pressure = value; }

double PropSensorsStore::get_fuel_level() const { return data_.fuel_level; }
void PropSensorsStore::set_fuel_level(double value) { data_.fuel_level = value; }

double PropSensorsStore::get_LOX_level() const { return data_.LOX_level; }
void PropSensorsStore::set_LOX_level(double value) { data_.LOX_level = value; }

double PropSensorsStore::get_N2_temperature() const { return data_.N2_temperature; }
void PropSensorsStore::set_N2_temperature(double value) { data_.N2_temperature = value; }

double PropSensorsStore::get_fuel_temperature() const { return data_.fuel_temperature; }
void PropSensorsStore::set_fuel_temperature(double value) { data_.fuel_temperature = value; }

double PropSensorsStore::get_LOX_temperature() const { return data_.LOX_temperature; }
void PropSensorsStore::set_LOX_temperature(double value) { data_.LOX_temperature = value; }

double PropSensorsStore::get_igniter_temperature() const { return data_.igniter_temperature; }
void PropSensorsStore::set_igniter_temperature(double value) { data_.igniter_temperature = value; }

double PropSensorsStore::get_fuel_inj_temperature() const { return data_.fuel_inj_temperature; }
void PropSensorsStore::set_fuel_inj_temperature(double value) { data_.fuel_inj_temperature = value; }

double PropSensorsStore::get_fuel_inj_cooling_temperature() const { return data_.fuel_inj_cooling_temperature; }
void PropSensorsStore::set_fuel_inj_cooling_temperature(double value) { data_.fuel_inj_cooling_temperature = value; }

double PropSensorsStore::get_LOX_inj_temperature() const { return data_.LOX_inj_temperature; }
void PropSensorsStore::set_LOX_inj_temperature(double value) { data_.LOX_inj_temperature = value; }

double PropSensorsStore::get_chamber_temperature() const { return data_.chamber_temperature; }
void PropSensorsStore::set_chamber_temperature(double value) { data_.chamber_temperature = value; }

uint32_t PropSensorsStore::get_PR_state() const { return data_.PR_state; }
void PropSensorsStore::set_PR_state(uint32_t value) { data_.PR_state = value; }
