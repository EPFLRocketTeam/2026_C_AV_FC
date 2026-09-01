#include "Application/Data/data.hpp"

using namespace flight_computer;

Valves::Valves()
: main_LOX_open(false),
  main_fuel_open(false),
  vent_LOX_open(false),
  vent_fuel_open(false),
  safety_LOX_open(false),
  safety_fuel_open(false),
  ball_valve_LOX(0.0f),
  ball_valve_fuel(0.0f)
{}

ValvesStore::ValvesStore() {}

bool  ValvesStore::get_main_LOX_open () const { return data_.main_LOX_open; } 
void  ValvesStore::set_main_LOX_open (bool value) { data_.main_LOX_open = value; }

bool  ValvesStore::get_main_fuel_open () const { return data_.main_fuel_open; } 
void  ValvesStore::set_main_fuel_open (bool value) { data_.main_fuel_open = value; }

bool  ValvesStore::get_vent_LOX_open () const { return data_.vent_LOX_open; } 
void  ValvesStore::set_vent_LOX_open (bool value) { data_.vent_LOX_open = value; }

bool  ValvesStore::get_vent_fuel_open () const { return data_.vent_fuel_open; } 
void  ValvesStore::set_vent_fuel_open (bool value) { data_.vent_fuel_open = value; }

bool  ValvesStore::get_safety_LOX_open () const { return data_.safety_LOX_open; } 
void  ValvesStore::set_safety_LOX_open (bool value) { data_.safety_LOX_open = value; }

bool  ValvesStore::get_safety_fuel_open () const { return data_.safety_fuel_open; } 
void  ValvesStore::set_safety_fuel_open (bool value) { data_.safety_fuel_open = value; }

float ValvesStore::get_ball_valve_LOX () const { return data_.ball_valve_LOX; } 
void  ValvesStore::set_ball_valve_LOX (float value) { data_.ball_valve_LOX = value; }

float ValvesStore::get_ball_valve_fuel () const { return data_.ball_valve_fuel; } 
void  ValvesStore::set_ball_valve_fuel (float value) { data_.ball_valve_fuel = value; }
