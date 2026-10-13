#include "../INA228.h"

namespace Drivers {
namespace INA228 {

//      REGISTERS                   ADDRESS    BITS  RW
#define INA228_CONFIG               0x00    //  16   RW
#define INA228_ADC_CONFIG           0x01    //  16   RW
#define INA228_SHUNT_CAL            0x02    //  16   RW
#define INA228_SHUNT_TEMP_CO        0x03    //  16   RW
#define INA228_SHUNT_VOLTAGE        0x04    //  24   R-
#define INA228_BUS_VOLTAGE          0x05    //  24   R-
#define INA228_TEMPERATURE          0x06    //  16   R-
#define INA228_CURRENT              0x07    //  24   R-
#define INA228_POWER                0x08    //  24   R-
#define INA228_ENERGY               0x09    //  40   R-
#define INA228_CHARGE               0x0A    //  40   R-
#define INA228_DIAG_ALERT           0x0B    //  16   RW
#define INA228_SOVL                 0x0C    //  16   RW
#define INA228_SUVL                 0x0D    //  16   RW
#define INA228_BOVL                 0x0E    //  16   RW
#define INA228_BUVL                 0x0F    //  16   RW
#define INA228_TEMP_LIMIT           0x10    //  16   RW
#define INA228_POWER_LIMIT          0x11    //  16   RW
#define INA228_MANUFACTURER         0x3E    //  16   R-
#define INA228_DEVICE_ID            0x3F    //  16   R-

//  CONFIG MASKS (register 0)
#define INA228_CFG_RST              0x8000
#define INA228_CFG_RSTACC           0x4000
#define INA228_CFG_CONVDLY          0x3FC0
#define INA228_CFG_TEMPCOMP         0x0020
#define INA228_CFG_ADCRANGE         0x0010

//  ADC MASKS (register 1)
#define INA228_ADC_MODE             0xF000
#define INA228_ADC_VBUSCT           0x0E00
#define INA228_ADC_VSHCT            0x01C0
#define INA228_ADC_VTCT             0x0038
#define INA228_ADC_AVG              0x0007

////////////////////////////////////////////////////////
//
//  CONSTRUCTOR
//
INA228_Driver::INA228_Driver(uint8_t address, I2C_HandleTypeDef *i2c) {
  _address     = address;
  _i2c         = i2c;
  //  no calibrated values by default.
  _shunt       = 0.015f;
  _maxCurrent  = 10.0f;
  _current_LSB = _maxCurrent * 1.9073486328125e-6f;  //  pow(2, -19)
  _ADCRange    = false;
  _error       = 0;
}

bool INA228_Driver::begin() {
  if (!isConnected()) return false;
  getADCRange();
  return true;
}

bool INA228_Driver::isConnected() {
  return HAL_I2C_IsDeviceReady(_i2c, _address << 1, 3, 1000) == HAL_OK;
}

uint8_t INA228_Driver::getAddress() {
  return _address;
}

////////////////////////////////////////////////////////
//
//  CORE FUNCTIONS
//
float INA228_Driver::getBusVoltage() {
  //  always positive, remove reserved bits.
  int32_t value = _readRegister(INA228_BUS_VOLTAGE, 3) >> 4;
  float bus_LSB = 195.3125e-6f;  //  195.3125 uV
  return value * bus_LSB;
}

float INA228_Driver::getShuntVoltage() {
  //  shunt_LSB depends on ADCRANGE in INA228_CONFIG register.
  float shunt_LSB = 312.5e-9f;  //  312.5 nV
  if (_ADCRange) shunt_LSB = 78.125e-9f;  //  78.125 nV

  //  remove reserved bits.
  int32_t value = _readRegister(INA228_SHUNT_VOLTAGE, 3) >> 4;
  //  handle negative values (20 bit)
  if (value & 0x00080000) value |= 0xFFF00000;
  return value * shunt_LSB;
}

int32_t INA228_Driver::getShuntVoltageRAW() {
  uint32_t value = _readRegister(INA228_SHUNT_VOLTAGE, 3) >> 4;
  if (value & 0x00080000) value |= 0xFFF00000;
  return (int32_t)value;
}

float INA228_Driver::getCurrent() {
  int32_t value = _readRegister(INA228_CURRENT, 3) >> 4;
  if (value & 0x00080000) value |= 0xFFF00000;
  return value * _current_LSB;
}

float INA228_Driver::getPower() {
  uint32_t value = _readRegister(INA228_POWER, 3);
  return value * 3.2f * _current_LSB;
}

float INA228_Driver::getTemperature() {
  uint32_t value = _readRegister(INA228_TEMPERATURE, 2);
  float LSB = 7.8125e-3f;  //  milli degree Celsius
  return value * LSB;
}

double INA228_Driver::getEnergy() {
  double value = _readRegisterF(INA228_ENERGY, 'U');
  return value * (16 * 3.2) * _current_LSB;
}

double INA228_Driver::getCharge() {
  double value = _readRegisterF(INA228_CHARGE, 'S');
  return value * _current_LSB;
}

////////////////////////////////////////////////////////
//
//  CONFIG REGISTER 0
//
void INA228_Driver::reset() {
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  value |= INA228_CFG_RST;
  _writeRegister(INA228_CONFIG, value);
}

bool INA228_Driver::setAccumulation(uint8_t value) {
  if (value > 1) return false;
  uint16_t reg = _readRegister(INA228_CONFIG, 2);
  if (value == 1) reg |= INA228_CFG_RSTACC;
  else            reg &= ~INA228_CFG_RSTACC;
  _writeRegister(INA228_CONFIG, reg);
  return true;
}

bool INA228_Driver::getAccumulation() {
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value & INA228_CFG_RSTACC) > 0;
}

void INA228_Driver::setConversionDelay(uint8_t steps) {
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  value &= ~INA228_CFG_CONVDLY;
  value |= (steps << 6);
  _writeRegister(INA228_CONFIG, value);
}

uint8_t INA228_Driver::getConversionDelay() {
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value >> 6) & 0xFF;
}

void INA228_Driver::setTemperatureCompensation(bool on) {
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  if (on) value |= INA228_CFG_TEMPCOMP;
  else    value &= ~INA228_CFG_TEMPCOMP;
  _writeRegister(INA228_CONFIG, value);
}

bool INA228_Driver::getTemperatureCompensation() {
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  return (value & INA228_CFG_TEMPCOMP) > 0;
}

bool INA228_Driver::setADCRange(bool flag) {
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  _ADCRange = (value & INA228_CFG_ADCRANGE) > 0;
  if (flag == _ADCRange) return true;

  _ADCRange = flag;
  if (flag) value |= INA228_CFG_ADCRANGE;
  else      value &= ~INA228_CFG_ADCRANGE;
  _writeRegister(INA228_CONFIG, value);
  //  ADCRANGE affects shunt_cal scaling -> recompute it.
  return setMaxCurrentShunt(getMaxCurrent(), getShunt()) == 0;
}

bool INA228_Driver::getADCRange() {
  uint16_t value = _readRegister(INA228_CONFIG, 2);
  _ADCRange = (value & INA228_CFG_ADCRANGE) > 0;
  return _ADCRange;
}

////////////////////////////////////////////////////////
//
//  CONFIG ADC REGISTER 1
//
bool INA228_Driver::setMode(uint8_t mode) {
  if (mode > 0x0F) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_MODE;
  value |= (mode << 12);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228_Driver::getMode() {
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_MODE) >> 12;
}

bool INA228_Driver::setBusVoltageConversionTime(uint8_t bvct) {
  if (bvct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VBUSCT;
  value |= (bvct << 9);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228_Driver::getBusVoltageConversionTime() {
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VBUSCT) >> 9;
}

bool INA228_Driver::setShuntVoltageConversionTime(uint8_t svct) {
  if (svct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VSHCT;
  value |= (svct << 6);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228_Driver::getShuntVoltageConversionTime() {
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VSHCT) >> 6;
}

bool INA228_Driver::setTemperatureConversionTime(uint8_t tct) {
  if (tct > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_VTCT;
  value |= (tct << 3);
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228_Driver::getTemperatureConversionTime() {
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_VTCT) >> 3;
}

bool INA228_Driver::setAverage(uint8_t avg) {
  if (avg > 7) return false;
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  value &= ~INA228_ADC_AVG;
  value |= avg;
  _writeRegister(INA228_ADC_CONFIG, value);
  return true;
}

uint8_t INA228_Driver::getAverage() {
  uint16_t value = _readRegister(INA228_ADC_CONFIG, 2);
  return (value & INA228_ADC_AVG);
}

////////////////////////////////////////////////////////
//
//  SHUNT CALIBRATION REGISTER 2
//
int INA228_Driver::setMaxCurrentShunt(float maxCurrent, float shunt) {
  if (shunt < 0.0001f) return -2;
  if (maxCurrent < 0.0f) return -3;
  _maxCurrent = maxCurrent;
  _shunt = shunt;
  _current_LSB = _maxCurrent * 1.9073486328125e-6f;  //  pow(2, -19)

  float shunt_cal = 13107.2e6f * _current_LSB * _shunt;
  if (_ADCRange) shunt_cal *= 4;

  _writeRegister(INA228_SHUNT_CAL, (uint16_t)shunt_cal);
  return 0;
}

float INA228_Driver::getMaxCurrent() { return _maxCurrent; }
float INA228_Driver::getShunt()      { return _shunt; }
float INA228_Driver::getCurrentLSB() { return _current_LSB; }

////////////////////////////////////////////////////////
//
//  SHUNT TEMPERATURE COEFFICIENT REGISTER 3
//
bool INA228_Driver::setShuntTemperatureCoefficent(uint16_t ppm) {
  if (ppm > 16383) return false;
  _writeRegister(INA228_SHUNT_TEMP_CO, ppm);
  return true;
}

uint16_t INA228_Driver::getShuntTemperatureCoefficent() {
  return _readRegister(INA228_SHUNT_TEMP_CO, 2);
}

////////////////////////////////////////////////////////
//
//  DIAGNOSE ALERT REGISTER 11
//
void INA228_Driver::setDiagnoseAlert(uint16_t flags) {
  _writeRegister(INA228_DIAG_ALERT, flags);
}

uint16_t INA228_Driver::getDiagnoseAlert() {
  return _readRegister(INA228_DIAG_ALERT, 2);
}

void INA228_Driver::setDiagnoseAlertBit(uint8_t bit) {
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  uint16_t mask = (1 << bit);
  if ((value & mask) == 0) {
    value |= mask;
    _writeRegister(INA228_DIAG_ALERT, value);
  }
}

void INA228_Driver::clearDiagnoseAlertBit(uint8_t bit) {
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  uint16_t mask = (1 << bit);
  if ((value & mask) != 0) {
    value &= ~mask;
    _writeRegister(INA228_DIAG_ALERT, value);
  }
}

uint16_t INA228_Driver::getDiagnoseAlertBit(uint8_t bit) {
  uint16_t value = _readRegister(INA228_DIAG_ALERT, 2);
  return (value >> bit) & 0x01;
}

////////////////////////////////////////////////////////
//
//  MANUFACTURER and ID REGISTER 3E/3F
//
uint16_t INA228_Driver::getManufacturer() {
  return _readRegister(INA228_MANUFACTURER, 2);
}

uint16_t INA228_Driver::getDieID() {
  uint16_t value = _readRegister(INA228_DEVICE_ID, 2);
  return (value >> 4) & 0x0FFF;
}

uint16_t INA228_Driver::getRevision() {
  uint16_t value = _readRegister(INA228_DEVICE_ID, 2);
  return value & 0x000F;
}

////////////////////////////////////////////////////////
//
//  ERROR HANDLING
//
int INA228_Driver::getLastError() {
  int e = _error;
  _error = 0;
  return e;
}

////////////////////////////////////////////////////////
//
//  PRIVATE
//
uint32_t INA228_Driver::_readRegister(uint8_t reg, uint8_t bytes) {
  _error = 0;
  uint8_t buf[4] = {0};

  if (HAL_I2C_Master_Transmit(_i2c, _address << 1, &reg, 1, 10) != HAL_OK) {
    _error = -1;
    return 0;
  }
  if (HAL_I2C_Master_Receive(_i2c, (_address << 1) | 1, buf, bytes, 10) != HAL_OK) {
    _error = -2;
    return 0;
  }

  uint32_t value = 0;
  for (int i = 0; i < bytes; i++) {
    value = (value << 8) | buf[i];
  }
  return value;
}

//  always 5 bytes
double INA228_Driver::_readRegisterF(uint8_t reg, char mode) {
  _error = 0;
  uint8_t buf[5] = {0};

  if (HAL_I2C_Master_Transmit(_i2c, _address << 1, &reg, 1, 10) != HAL_OK) {
    _error = -1;
    return 0;
  }
  if (HAL_I2C_Master_Receive(_i2c, (_address << 1) | 1, buf, 5, 10) != HAL_OK) {
    _error = -2;
    return 0;
  }

  uint32_t val = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
               | ((uint32_t)buf[2] << 8)  | buf[3];

  double value = (mode == 'U') ? (double)val : (double)(int32_t)val;
  value = value * 256.0 + buf[4];
  return value;
}

uint16_t INA228_Driver::_writeRegister(uint8_t reg, uint16_t value) {
  _error = 0;
  uint8_t buf[3] = { reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };

  if (HAL_I2C_Master_Transmit(_i2c, _address << 1, buf, 3, 10) != HAL_OK) {
    _error = -1;
    return 1;
  }
  return 0;
}

} // namespace INA228
} // namespace Drivers
