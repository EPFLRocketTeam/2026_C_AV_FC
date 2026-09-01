#include "../data.hpp"
#include <cstdint>

using namespace flight_computer;

VehiculeOverview::VehiculeOverview()
    : no_cable_continuity(0),
      pyros_on({ false, false, false, false }) {}

VehiculeOverviewStore::VehiculeOverviewStore() {}

uint8_t VehiculeOverviewStore::get_no_cable_continuity() const { return data_.no_cable_continuity; }
void VehiculeOverviewStore::set_no_cable_continuity(uint8_t value) { data_.no_cable_continuity = value; }

bool VehiculeOverviewStore::get_pyro_ch1_on () const { return data_.pyros_on[0]; }
bool VehiculeOverviewStore::get_pyro_ch2_on () const { return data_.pyros_on[1]; }
bool VehiculeOverviewStore::get_pyro_ch3_on () const { return data_.pyros_on[2]; }
bool VehiculeOverviewStore::get_pyro_ch4_on () const { return data_.pyros_on[3]; }

void VehiculeOverviewStore::set_pyro_ch1_on (bool value) { data_.pyros_on[0] = value; }
void VehiculeOverviewStore::set_pyro_ch2_on (bool value) { data_.pyros_on[1] = value; }
void VehiculeOverviewStore::set_pyro_ch3_on (bool value) { data_.pyros_on[2] = value; }
void VehiculeOverviewStore::set_pyro_ch4_on (bool value) { data_.pyros_on[3] = value; }
