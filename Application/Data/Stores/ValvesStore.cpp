#include "Application/Data/data.hpp"

using namespace flight_computer;

Valves::Valves()
:   valve_dpr_pressure_lox(false),
    valve_dpr_pressure_fuel(false),
    valve_dpr_vent_copv(false),
    valve_dpr_vent_lox(false),
    valve_dpr_vent_fuel(false),
    valve_prb_main_lox(false),
    valve_prb_main_fuel(false)
{}

ValvesStore::ValvesStore() {}

bool ValvesStore::get_valve_dpr_pressure_lox() const { return data_.valve_dpr_pressure_lox; }
void ValvesStore::set_valve_dpr_pressure_lox(bool value) { data_.valve_dpr_pressure_lox = value; }

bool ValvesStore::get_valve_dpr_pressure_fuel() const { return data_.valve_dpr_pressure_fuel; }
void ValvesStore::set_valve_dpr_pressure_fuel(bool value) { data_.valve_dpr_pressure_fuel = value; }

bool ValvesStore::get_valve_dpr_vent_copv() const { return data_.valve_dpr_vent_copv; }
void ValvesStore::set_valve_dpr_vent_copv(bool value) { data_.valve_dpr_vent_copv = value; }

bool ValvesStore::get_valve_dpr_vent_lox() const { return data_.valve_dpr_vent_lox; }
void ValvesStore::set_valve_dpr_vent_lox(bool value) { data_.valve_dpr_vent_lox = value; }

bool ValvesStore::get_valve_dpr_vent_fuel() const { return data_.valve_dpr_vent_fuel; }
void ValvesStore::set_valve_dpr_vent_fuel(bool value) { data_.valve_dpr_vent_fuel = value; }

bool ValvesStore::get_valve_prb_main_lox() const { return data_.valve_prb_main_lox; }
void ValvesStore::set_valve_prb_main_lox(bool value) { data_.valve_prb_main_lox = value; }

bool ValvesStore::get_valve_prb_main_fuel() const { return data_.valve_prb_main_fuel; }
void ValvesStore::set_valve_prb_main_fuel(bool value) { data_.valve_prb_main_fuel = value; }
