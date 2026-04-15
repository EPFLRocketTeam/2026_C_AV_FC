#include "../data.hpp"
#include "Drivers/InvIMU/InvIMU.h"

using namespace flight_computer;

NavSensors::NavSensors()
:   adxl{0, 0, 0},
    adxl_aux{0, 0, 0},
    imu{},
    bmp{0, 0},
    bmp_aux{0, 0}
{}

NavSensorsStore::NavSensorsStore() {}

adxl375_data NavSensorsStore::get_adxl() const { return data_.adxl; }
void NavSensorsStore::set_adxl(const adxl375_data& d) { data_.adxl = d; }

adxl375_data NavSensorsStore::get_adxl_aux() const { return data_.adxl_aux; }
void NavSensorsStore::set_adxl_aux(const adxl375_data& d) { data_.adxl_aux = d; }

Drivers::InvIMU::IMUData NavSensorsStore::get_imu(size_t index) const {
    if (index >= 3) return Drivers::InvIMU::IMUData{};
    return data_.imu[index];
}

void NavSensorsStore::set_imu(size_t index, const Drivers::InvIMU::IMUData& d) {
    if (index >= 3) return;
    data_.imu[index] = d;
}

bmp3_data NavSensorsStore::get_bmp() const { return data_.bmp; }
void NavSensorsStore::set_bmp(const bmp3_data& d) { data_.bmp = d; }

bmp3_data NavSensorsStore::get_bmp_aux() const { return data_.bmp_aux; }
void NavSensorsStore::set_bmp_aux(const bmp3_data& d) { data_.bmp_aux = d; }
