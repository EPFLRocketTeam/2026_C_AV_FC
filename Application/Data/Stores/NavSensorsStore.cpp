#include "../data.hpp"
#include "Drivers/InvIMU/InvIMU.h"

using namespace flight_computer;

NavSensors::NavSensors()
:   adxl{0, 0, 0},
    adxl_aux{0, 0, 0},
    imu{},
    bmp{0, 0},
    bmp_aux{0, 0},
    bmp_3{0, 0},
    bmp_4{0, 0}
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

bmp3_data NavSensorsStore::get_bmp() const { return get_bmp(0); }
void NavSensorsStore::set_bmp(const bmp3_data& d) { set_bmp(0, d); }

bmp3_data NavSensorsStore::get_bmp_aux() const { return get_bmp(1); }
void NavSensorsStore::set_bmp_aux(const bmp3_data& d) { set_bmp(1, d); }

bmp3_data NavSensorsStore::get_bmp(size_t index) const {
    switch (index) {
    case 0: return data_.bmp;
    case 1: return data_.bmp_aux;
    case 2: return data_.bmp_3;
    case 3: return data_.bmp_4;
    default: return bmp3_data{};
    }
}

void NavSensorsStore::set_bmp(size_t index, const bmp3_data& d) {
    switch (index) {
    case 0:
        data_.bmp = d;
        break;
    case 1:
        data_.bmp_aux = d;
        break;
    case 2:
        data_.bmp_3 = d;
        break;
    case 3:
        data_.bmp_4 = d;
        break;
    default:
        break;
    }
}
