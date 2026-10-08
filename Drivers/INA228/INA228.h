#pragma once
//  Ported from Rob Tillaart's Arduino INA228 library (v0.4.1) to STM32 HAL.
//  https://github.com/RobTillaart/INA228
//  I2C, 20-bit voltage/current/power/energy/charge monitor.
//  Read the datasheet for register-level details.

#include "stm32h7xx_hal.h"
#include <cstdint>

namespace Drivers {
namespace INA228 {

//  for setMode() and getMode()
enum ina228_mode_enum {
  INA228_MODE_SHUTDOWN            = 0x00,
  INA228_MODE_TRIG_BUS            = 0x01,
  INA228_MODE_TRIG_SHUNT          = 0x02,
  INA228_MODE_TRIG_BUS_SHUNT      = 0x03,
  INA228_MODE_TRIG_TEMP           = 0x04,
  INA228_MODE_TRIG_TEMP_BUS       = 0x05,
  INA228_MODE_TRIG_TEMP_SHUNT     = 0x06,
  INA228_MODE_TRIG_TEMP_BUS_SHUNT = 0x07,

  INA228_MODE_SHUTDOWN2           = 0x08,
  INA228_MODE_CONT_BUS            = 0x09,
  INA228_MODE_CONT_SHUNT          = 0x0A,
  INA228_MODE_CONT_BUS_SHUNT      = 0x0B,
  INA228_MODE_CONT_TEMP           = 0x0C,
  INA228_MODE_CONT_TEMP_BUS       = 0x0D,
  INA228_MODE_CONT_TEMP_SHUNT     = 0x0E,
  INA228_MODE_CONT_TEMP_BUS_SHUNT = 0x0F
};

//  for setAverage() and getAverage()
enum ina228_average_enum {
    INA228_1_SAMPLE     = 0,
    INA228_4_SAMPLES    = 1,
    INA228_16_SAMPLES   = 2,
    INA228_64_SAMPLES   = 3,
    INA228_128_SAMPLES  = 4,
    INA228_256_SAMPLES  = 5,
    INA228_512_SAMPLES  = 6,
    INA228_1024_SAMPLES = 7
};

//  for Bus, shunt and temperature conversion timing.
enum ina228_timing_enum {
    INA228_50_us   = 0,
    INA228_84_us   = 1,
    INA228_150_us  = 2,
    INA228_280_us  = 3,
    INA228_540_us  = 4,
    INA228_1052_us = 5,
    INA228_2074_us = 6,
    INA228_4120_us = 7
};

//  for diagnose/alert() bit fields.
enum ina228_diag_enum {
  INA228_DIAG_MEMORY_STATUS      = 0,
  INA228_DIAG_CONVERT_COMPLETE   = 1,
  INA228_DIAG_POWER_OVER_LIMIT   = 2,
  INA228_DIAG_BUS_UNDER_LIMIT    = 3,
  INA228_DIAG_BUS_OVER_LIMIT     = 4,
  INA228_DIAG_SHUNT_UNDER_LIMIT  = 5,
  INA228_DIAG_SHUNT_OVER_LIMIT   = 6,
  INA228_DIAG_TEMP_OVER_LIMIT    = 7,
  INA228_DIAG_RESERVED           = 8,
  INA228_DIAG_MATH_OVERFLOW      = 9,
  INA228_DIAG_CHARGE_OVERFLOW    = 10,
  INA228_DIAG_ENERGY_OVERFLOW    = 11,
  INA228_DIAG_ALERT_POLARITY     = 12,
  INA228_DIAG_SLOW_ALERT         = 13,
  INA228_DIAG_CONVERT_READY      = 14,
  INA228_DIAG_ALERT_LATCH        = 15
};

class INA228_Driver {
public:
  //  address between 0x40 and 0x4F
  explicit INA228_Driver(uint8_t address, I2C_HandleTypeDef *i2c);

  bool     begin();
  bool     isConnected();
  uint8_t  getAddress();

  //       BUS VOLTAGE
  float    getBusVoltage();     //  Volt

  //       SHUNT VOLTAGE
  float    getShuntVoltage();   //  Volt
  int32_t  getShuntVoltageRAW();

  //       SHUNT CURRENT
  float    getCurrent();        //  Ampere

  //       POWER
  float    getPower();          //  Watt

  //       TEMPERATURE
  float    getTemperature();    //  Celsius

  //       ENERGY / CHARGE (higher range -> double)
  double   getEnergy();         //  Joule
  double   getCharge();         //  Coulomb

  //
  //  CONFIG REGISTER 0
  //
  void     reset();
  bool     setAccumulation(uint8_t value);
  bool     getAccumulation();
  void     setConversionDelay(uint8_t steps);
  uint8_t  getConversionDelay();
  void     setTemperatureCompensation(bool on);
  bool     getTemperatureCompensation();
  bool     setADCRange(bool flag);   //  false => 164 mV, true => 41 mV
  bool     getADCRange();

  //
  //  CONFIG ADC REGISTER 1
  //
  bool     setMode(uint8_t mode = INA228_MODE_CONT_TEMP_BUS_SHUNT);
  uint8_t  getMode();
  bool     setBusVoltageConversionTime(uint8_t bvct = INA228_1052_us);
  uint8_t  getBusVoltageConversionTime();
  bool     setShuntVoltageConversionTime(uint8_t svct = INA228_1052_us);
  uint8_t  getShuntVoltageConversionTime();
  bool     setTemperatureConversionTime(uint8_t tct = INA228_1052_us);
  uint8_t  getTemperatureConversionTime();
  bool     setAverage(uint8_t avg = INA228_1_SAMPLE);
  uint8_t  getAverage();

  //
  //  SHUNT CALIBRATION REGISTER 2
  //  maxCurrent <= 204 (in fact no limit), shunt >= 0.0001. returns 0 == OK.
  //
  int      setMaxCurrentShunt(float maxCurrent, float shunt);
  bool     isCalibrated()    { return _current_LSB > 0.0f; };
  float    getMaxCurrent();
  float    getShunt();
  float    getCurrentLSB();

  //
  //  SHUNT TEMPERATURE COEFFICIENT REGISTER 3 (ppm = 0..16383)
  //
  bool     setShuntTemperatureCoefficent(uint16_t ppm = 0);
  uint16_t getShuntTemperatureCoefficent();

  //
  //  DIAGNOSE ALERT REGISTER 11
  //
  void     setDiagnoseAlert(uint16_t flags);
  uint16_t getDiagnoseAlert();
  void     setDiagnoseAlertBit(uint8_t bit);
  void     clearDiagnoseAlertBit(uint8_t bit);
  uint16_t getDiagnoseAlertBit(uint8_t bit);

  //
  //  MANUFACTURER and ID REGISTER 3E and 3F
  //
  uint16_t getManufacturer();  //  0x5449 ("TI" in ASCII)
  uint16_t getDieID();         //  0x0228
  uint16_t getRevision();      //  0x0001

  //
  //  ERROR HANDLING
  //
  int      getLastError();

private:
  uint32_t _readRegister(uint8_t reg, uint8_t bytes);
  double   _readRegisterF(uint8_t reg, char mode);
  uint16_t _writeRegister(uint8_t reg, uint16_t value);

  float _current_LSB;
  float _shunt;
  float _maxCurrent;
  bool  _ADCRange;

  uint8_t _address;
  I2C_HandleTypeDef *_i2c;

  int _error;
};

} // namespace INA228
} // namespace Drivers
