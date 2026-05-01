#include "../data.hpp"
#include <cstdint>

using namespace flight_computer;

VehiculeOverview::VehiculeOverview()
    : no_cable_continuity(0) {}

VehiculeOverviewStore::VehiculeOverviewStore() {}

uint8_t VehiculeOverviewStore::get_no_cable_continuity() const { return data_.no_cable_continuity; }
void VehiculeOverviewStore::set_no_cable_continuity(uint8_t value) { data_.no_cable_continuity = value; }
