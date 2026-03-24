#include "../data.hpp"

using namespace flight_computer;

NavSensors::NavSensors()
:   adxl{0, 0, 0},
    adxl_aux{0, 0, 0},
    bmi_accel{0, 0, 0},
    bmi_gyro{0, 0, 0},
    bmi_aux_accel{0, 0, 0},
    bmi_aux_gyro{0, 0, 0},
    bmp{0, 0},
    bmp_aux{0, 0}
{}

NavSensorsStore::NavSensorsStore() {}

adxl375_data NavSensorsStore::get_adxl() const { return data_.adxl; }
void NavSensorsStore::set_adxl(const adxl375_data& d) { data_.adxl = d; }

adxl375_data NavSensorsStore::get_adxl_aux() const { return data_.adxl_aux; }
void NavSensorsStore::set_adxl_aux(const adxl375_data& d) { data_.adxl_aux = d; }

bmi08_sensor_data_f NavSensorsStore::get_bmi_accel() const { return data_.bmi_accel; }
void NavSensorsStore::set_bmi_accel(const bmi08_sensor_data_f& d) { data_.bmi_accel = d; }

bmi08_sensor_data_f NavSensorsStore::get_bmi_gyro() const { return data_.bmi_gyro; }
void NavSensorsStore::set_bmi_gyro(const bmi08_sensor_data_f& d) { data_.bmi_gyro = d; }

bmi08_sensor_data_f NavSensorsStore::get_bmi_aux_accel() const { return data_.bmi_aux_accel; }
void NavSensorsStore::set_bmi_aux_accel(const bmi08_sensor_data_f& d) { data_.bmi_aux_accel = d; }

bmi08_sensor_data_f NavSensorsStore::get_bmi_aux_gyro() const { return data_.bmi_aux_gyro; }
void NavSensorsStore::set_bmi_aux_gyro(const bmi08_sensor_data_f& d) { data_.bmi_aux_gyro = d; }

bmp3_data NavSensorsStore::get_bmp() const { return data_.bmp; }
void NavSensorsStore::set_bmp(const bmp3_data& d) { data_.bmp = d; }

bmp3_data NavSensorsStore::get_bmp_aux() const { return data_.bmp_aux; }
void NavSensorsStore::set_bmp_aux(const bmp3_data& d) { data_.bmp_aux = d; }
