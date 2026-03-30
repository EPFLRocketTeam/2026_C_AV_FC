#include "../data.hpp"
#include <cstdint>

using namespace flight_computer;

FlightEventTimers::FlightEventTimers()
    : flight_duration(0), descent_duration(0) {}

FlightEventTimersStore::FlightEventTimersStore() {}

uint32_t FlightEventTimersStore::get_flight_timer() const { return data_.flight_duration; }
void FlightEventTimersStore::set_flight_timer(uint32_t value) { data_.flight_duration = value; }

uint32_t FlightEventTimersStore::get_descent_timer() const { return data_.descent_duration; }
void FlightEventTimersStore::set_descent_timer(uint32_t value) { data_.descent_duration = value; }


