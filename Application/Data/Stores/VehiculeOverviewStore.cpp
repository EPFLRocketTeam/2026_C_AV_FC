#include "../data.hpp"
#include <cstdint>

using namespace flight_computer;

VehiculeOverview::VehiculeOverview()
    : no_cable_continuity_lox(0), no_cable_continuity_eth(0), pyros_activated(false) {
    pyros_on[0] = false;
    pyros_on[1] = false;
    pyros_on[2] = false;
    pyros_on[3] = false;
}

VehiculeOverviewStore::VehiculeOverviewStore() {}

bool VehiculeOverviewStore::get_pyros_activated () const {
    return data_.pyros_activated;
}
void VehiculeOverviewStore::set_pyros_activated (bool value) {
    data_.pyros_activated = value;
}

bool VehiculeOverviewStore::get_no_cable_continuity_eth () const {
    return data_.no_cable_continuity_eth;
}
void VehiculeOverviewStore::set_no_cable_continuity_eth (bool value) {
    data_.no_cable_continuity_eth = value;
}

bool VehiculeOverviewStore::get_no_cable_continuity_lox () const {
    return data_.no_cable_continuity_lox;
}
void VehiculeOverviewStore::set_no_cable_continuity_lox (bool value) {
    data_.no_cable_continuity_lox = value;
}

bool VehiculeOverviewStore::get_pyro_ch1_on () const { return data_.pyros_on[0]; }
bool VehiculeOverviewStore::get_pyro_ch2_on () const { return data_.pyros_on[1]; }
bool VehiculeOverviewStore::get_pyro_ch3_on () const { return data_.pyros_on[2]; }
bool VehiculeOverviewStore::get_pyro_ch4_on () const { return data_.pyros_on[3]; }

void VehiculeOverviewStore::set_pyro_ch1_on (bool value) { data_.pyros_on[0] = value; }
void VehiculeOverviewStore::set_pyro_ch2_on (bool value) { data_.pyros_on[1] = value; }
void VehiculeOverviewStore::set_pyro_ch3_on (bool value) { data_.pyros_on[2] = value; }
void VehiculeOverviewStore::set_pyro_ch4_on (bool value) { data_.pyros_on[3] = value; }
