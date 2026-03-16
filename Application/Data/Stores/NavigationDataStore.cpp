#include "Application/Data/data.hpp"

using namespace flight_computer;

NavigationData::NavigationData()
:   position_kalman{},
    speed{},
    accel{},
    attitude{},
    course(0.0),
    altitude(0.0),
    baro{}
{}

NavigationDataStore::NavigationDataStore() {}

Vector3 NavigationDataStore::get_position_kalman() const { return data_.position_kalman; }
void NavigationDataStore::set_position_kalman(const Vector3& value) { data_.position_kalman = value; }

Vector3 NavigationDataStore::get_speed() const { return data_.speed; }
void NavigationDataStore::set_speed(const Vector3& value) { data_.speed = value; }

Vector3 NavigationDataStore::get_accel() const { return data_.accel; }
void NavigationDataStore::set_accel(const Vector3& value) { data_.accel = value; }

Vector3 NavigationDataStore::get_attitude() const { return data_.attitude; }
void NavigationDataStore::set_attitude(const Vector3& value) { data_.attitude = value; }

double NavigationDataStore::get_course() const { return data_.course; }
void NavigationDataStore::set_course(double value) { data_.course = value; }

double NavigationDataStore::get_altitude() const { return data_.altitude; }
void NavigationDataStore::set_altitude(double value) { data_.altitude = value; }

bmp3_data NavigationDataStore::get_baro() const { return data_.baro; }
void NavigationDataStore::set_baro(const bmp3_data& value) { data_.baro = value; }
