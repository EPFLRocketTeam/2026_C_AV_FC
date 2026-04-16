#include "Application/Data/data.hpp"

using namespace flight_computer;

SensStatus::SensStatus()
:   adxl_status(0),
    adxl_aux_status(0),
    bmi_accel_status(0),
    bmi_aux_accel_status(0),
    bmi_gyro_status(0),
    bmi_aux_gyro_status(0),
    bmp_status{},
    bmp_aux_status{}
{}

SensStatusStore::SensStatusStore() {}

uint8_t SensStatusStore::get_adxl_status() const { return data_.adxl_status; }
void SensStatusStore::set_adxl_status(uint8_t value) { data_.adxl_status = value; }

uint8_t SensStatusStore::get_adxl_aux_status() const { return data_.adxl_aux_status; }
void SensStatusStore::set_adxl_aux_status(uint8_t value) { data_.adxl_aux_status = value; }

uint8_t SensStatusStore::get_bmi_accel_status() const { return data_.bmi_accel_status; }
void SensStatusStore::set_bmi_accel_status(uint8_t value) { data_.bmi_accel_status = value; }

uint8_t SensStatusStore::get_bmi_aux_accel_status() const { return data_.bmi_aux_accel_status; }
void SensStatusStore::set_bmi_aux_accel_status(uint8_t value) { data_.bmi_aux_accel_status = value; }

uint8_t SensStatusStore::get_bmi_gyro_status() const { return data_.bmi_gyro_status; }
void SensStatusStore::set_bmi_gyro_status(uint8_t value) { data_.bmi_gyro_status = value; }

uint8_t SensStatusStore::get_bmi_aux_gyro_status() const { return data_.bmi_aux_gyro_status; }
void SensStatusStore::set_bmi_aux_gyro_status(uint8_t value) { data_.bmi_aux_gyro_status = value; }

bmp3_status SensStatusStore::get_bmp_status() const { return data_.bmp_status; }
void SensStatusStore::set_bmp_status(const bmp3_status& value) { data_.bmp_status = value; }

bmp3_status SensStatusStore::get_bmp_aux_status() const { return data_.bmp_aux_status; }
void SensStatusStore::set_bmp_aux_status(const bmp3_status& value) { data_.bmp_aux_status = value; }
