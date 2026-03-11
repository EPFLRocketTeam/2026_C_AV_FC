
#ifndef APP_DATA_H
#define APP_DATA_H

#include <cmath>

#include "Application/Data/fsm.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"

namespace flight_computer {

struct bmp3_int_status {
    uint8_t fifo_wm;
    uint8_t fifo_full;
    uint8_t drdy;

    bmp3_int_status() : fifo_wm(0), fifo_full(0), drdy(0) {}
};

struct bmp3_sens_status {
    uint8_t cmd_rdy;
    uint8_t drdy_press;
    uint8_t drdy_temp;

    bmp3_sens_status() : cmd_rdy(0), drdy_press(0), drdy_temp(0) {}
};

struct bmp3_err_status {
    uint8_t fatal;
    uint8_t cmd;
    uint8_t conf;

    bmp3_err_status() : fatal(0), cmd(0), conf(0) {}
};

struct bmp3_status {
    struct bmp3_int_status intr;
    struct bmp3_sens_status sensor;
    struct bmp3_err_status err;
    uint8_t pwr_on_rst;

    bmp3_status() : intr{}, sensor{}, err{}, pwr_on_rst(0) {}
};

struct bmp3_data {
    double temperature;
    double pressure;

    bmp3_data() : temperature(0.0), pressure(0.0) {}
    bmp3_data(double temp, double press) : temperature(temp), pressure(press) {}
};

struct SensStatus {
    uint8_t     adxl_status;
    uint8_t     adxl_aux_status;
    uint8_t     bmi_accel_status;
    uint8_t     bmi_aux_accel_status;
    uint8_t     bmi_gyro_status;
    uint8_t     bmi_aux_gyro_status;
    bmp3_status bmp_status;
    bmp3_status bmp_aux_status;

    SensStatus();
};

struct adxl375_data {
	float x;
	float y;
	float z;

	adxl375_data() : x(0.0f), y(0.0f), z(0.0f) {}
	adxl375_data(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct bmi08_sensor_data_f
{
    float x;
    float y;
    float z;

    bmi08_sensor_data_f() : x(0.0f), y(0.0f), z(0.0f) {}
    bmi08_sensor_data_f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct NavSensors {
    adxl375_data        adxl;
    adxl375_data        adxl_aux;
    bmi08_sensor_data_f bmi_accel;
    bmi08_sensor_data_f bmi_gyro;
    bmi08_sensor_data_f bmi_aux_accel;
    bmi08_sensor_data_f bmi_aux_gyro;
    bmp3_data           bmp;
    bmp3_data           bmp_aux;

    NavSensors();
};

struct PropSensors {
    double    N2_pressure;
    double    fuel_pressure;
    double    LOX_pressure;
    double    igniter_pressure;
    double    LOX_inj_pressure;
    double    fuel_inj_pressure;
    double    chamber_pressure;
    double    fuel_level;
    double    LOX_level;
    double    N2_temperature;
    double    fuel_temperature;
    double    LOX_temperature;
    double    igniter_temperature;
    double    fuel_inj_temperature;
    double    fuel_inj_cooling_temperature;
    double    LOX_inj_temperature;
    double    chamber_temperature;
    uint32_t  PR_state;

    PropSensors();
};

struct Valves {
    bool valve_dpr_pressure_lox;
    bool valve_dpr_pressure_fuel;
    bool valve_dpr_vent_copv;
    bool valve_dpr_vent_lox;
    bool valve_dpr_vent_fuel;
    bool valve_prb_main_lox;
    bool valve_prb_main_fuel;

    Valves();
};

struct Vector3 {
    double x;
    double y;
    double z;

    Vector3() : x(0.0), y(0.0), z(0.0) {}
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    inline double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }
};

struct NavigationData {
    Vector3 position_kalman;
    Vector3 speed;
    Vector3 accel;
    Vector3 attitude;
    double  course;
    double  altitude;
    bmp3_data baro;

    NavigationData();
};

struct Batteries {
    float lpb_voltage;
    float hpb_voltage;

    Batteries();
};

struct CamsRecording {
    bool cam_sep;
    bool cam_up;
    bool cam_down;

    CamsRecording();
};

struct UplinkCmd {
    uint8_t id;
    uint8_t value;

    UplinkCmd();
};

template <typename T> class IStore {
public:
  virtual ~IStore() = default;

  inline void set(const T &value) {
	data_ = value;
    };
  inline const T &get() const {
	return data_;
    };
  inline T *get_ref() {
	return &data_;
    };

protected:
    T data_;
};

class BatteriesStore : public IStore<Batteries> {
public:
    BatteriesStore();

    float get_lpb_voltage() const;
    void set_lpb_voltage(float value);

    float get_hpb_voltage() const;
    void set_hpb_voltage(float value);
};

class CamsRecordingStore : public IStore<CamsRecording> {
public:
    CamsRecordingStore();

    bool get_cam_sep() const;
    void set_cam_sep(bool value);

    bool get_cam_up() const;
    void set_cam_up(bool value);

    bool get_cam_down() const;
    void set_cam_down(bool value);
};

class UplinkCmdStore : public IStore<UplinkCmd> {
public:
    UplinkCmdStore();

    uint8_t get_id() const;
    void set_id(uint8_t value);

    uint8_t get_value() const;
    void set_value(uint8_t value);
};

class SensStatusStore : public IStore<SensStatus> {
public:
    SensStatusStore();

    uint8_t get_adxl_status() const;
    void set_adxl_status(uint8_t value);

    uint8_t get_adxl_aux_status() const;
    void set_adxl_aux_status(uint8_t value);

    uint8_t get_bmi_accel_status() const;
    void set_bmi_accel_status(uint8_t value);

    uint8_t get_bmi_aux_accel_status() const;
    void set_bmi_aux_accel_status(uint8_t value);

    uint8_t get_bmi_gyro_status() const;
    void set_bmi_gyro_status(uint8_t value);

    uint8_t get_bmi_aux_gyro_status() const;
    void set_bmi_aux_gyro_status(uint8_t value);

    bmp3_status get_bmp_status() const;
    void set_bmp_status(const bmp3_status& value);

    bmp3_status get_bmp_aux_status() const;
    void set_bmp_aux_status(const bmp3_status& value);
};

class NavSensorsStore : public IStore<NavSensors> {
public:
    NavSensorsStore();

    adxl375_data get_adxl() const;
    void set_adxl(const adxl375_data& data);

    adxl375_data get_adxl_aux() const;
    void set_adxl_aux(const adxl375_data& data);

    bmi08_sensor_data_f get_bmi_accel() const;
    void set_bmi_accel(const bmi08_sensor_data_f& data);

    bmi08_sensor_data_f get_bmi_gyro() const;
    void set_bmi_gyro(const bmi08_sensor_data_f& data);

    bmi08_sensor_data_f get_bmi_aux_accel() const;
    void set_bmi_aux_accel(const bmi08_sensor_data_f& data);

    bmi08_sensor_data_f get_bmi_aux_gyro() const;
    void set_bmi_aux_gyro(const bmi08_sensor_data_f& data);

    bmp3_data get_bmp() const;
    void set_bmp(const bmp3_data& data);

    bmp3_data get_bmp_aux() const;
    void set_bmp_aux(const bmp3_data& data);
};

class PropSensorsStore : public IStore<PropSensors> {
public:
    PropSensorsStore();

    double get_N2_pressure() const;
    void set_N2_pressure(double value);

    double get_fuel_pressure() const;
    void set_fuel_pressure(double value);

    double get_LOX_pressure() const;
    void set_LOX_pressure(double value);

    double get_igniter_pressure() const;
    void set_igniter_pressure(double value);

    double get_LOX_inj_pressure() const;
    void set_LOX_inj_pressure(double value);

    double get_fuel_inj_pressure() const;
    void set_fuel_inj_pressure(double value);

    double get_chamber_pressure() const;
    void set_chamber_pressure(double value);

    double get_fuel_level() const;
    void set_fuel_level(double value);

    double get_LOX_level() const;
    void set_LOX_level(double value);

    double get_N2_temperature() const;
    void set_N2_temperature(double value);

    double get_fuel_temperature() const;
    void set_fuel_temperature(double value);

    double get_LOX_temperature() const;
    void set_LOX_temperature(double value);

    double get_igniter_temperature() const;
    void set_igniter_temperature(double value);

    double get_fuel_inj_temperature() const;
    void set_fuel_inj_temperature(double value);

    double get_fuel_inj_cooling_temperature() const;
    void set_fuel_inj_cooling_temperature(double value);

    double get_LOX_inj_temperature() const;
    void set_LOX_inj_temperature(double value);

    double get_chamber_temperature() const;
    void set_chamber_temperature(double value);

    uint32_t get_PR_state() const;
    void set_PR_state(uint32_t value);
};

class ValvesStore : public IStore<Valves> {
public:
    ValvesStore();

    bool get_valve_dpr_pressure_lox() const;
    void set_valve_dpr_pressure_lox(bool value);

    bool get_valve_dpr_pressure_fuel() const;
    void set_valve_dpr_pressure_fuel(bool value);

    bool get_valve_dpr_vent_copv() const;
    void set_valve_dpr_vent_copv(bool value);

    bool get_valve_dpr_vent_lox() const;
    void set_valve_dpr_vent_lox(bool value);

    bool get_valve_dpr_vent_fuel() const;
    void set_valve_dpr_vent_fuel(bool value);

    bool get_valve_prb_main_lox() const;
    void set_valve_prb_main_lox(bool value);

    bool get_valve_prb_main_fuel() const;
    void set_valve_prb_main_fuel(bool value);
};

class NavigationDataStore : public IStore<NavigationData> {
public:
    NavigationDataStore();

    Vector3 get_position_kalman() const;
    void set_position_kalman(const Vector3& value);

    Vector3 get_speed() const;
    void set_speed(const Vector3& value);

    Vector3 get_accel() const;
    void set_accel(const Vector3& value);

    Vector3 get_attitude() const;
    void set_attitude(const Vector3& value);

    double get_course() const;
    void set_course(double value);

    double get_altitude() const;
    void set_altitude(double value);

    bmp3_data get_baro() const;
    void set_baro(const bmp3_data& value);
};

struct Event {
    bool command_updated;
    bool calibrated;
    bool dpr_eth_ready;
    bool dpr_eth_pressure_ok;
    bool dpr_lox_ready;
    bool dpr_lox_pressure_ok;
    bool prb_ready;
    bool trb_ready;
    bool ignited;
    bool seperated;
    bool chute_unreefed;
    bool ignition_failed;
    bool catastrophic_failure;

    Event();
};

class EventStore : public IStore<Event> {
public:
    EventStore();

    bool get_command_updated() const;
    void set_command_updated(bool value);

    bool get_calibrated() const;
    void set_calibrated(bool value);

    bool get_dpr_eth_ready() const;
    void set_dpr_eth_ready(bool value);

    bool get_dpr_eth_pressure_ok() const;
    void set_dpr_eth_pressure_ok(bool value);

    bool get_dpr_lox_ready() const;
    void set_dpr_lox_ready(bool value);

    bool get_dpr_lox_pressure_ok() const;
    void set_dpr_lox_pressure_ok(bool value);

    bool get_prb_ready() const;
    void set_prb_ready(bool value);

    bool get_trb_ready() const;
    void set_trb_ready(bool value);

    bool get_ignited() const;
    void set_ignited(bool value);

    bool get_seperated() const;
    void set_seperated(bool value);

    bool get_chute_unreefed() const;
    void set_chute_unreefed(bool value);

    bool get_ignition_failed() const;
    void set_ignition_failed(bool value);

    bool get_catastrophic_failure() const;
    void set_catastrophic_failure(bool value);
};

class StateStore : public IStore<State> {
public:
  StateStore();
};

struct DataDump {
  State av_state;
  uint32_t av_timestamp;
  float av_fc_temp;
  GpsBasicFixData gps_state;
  SensStatus sensStatus;
  PropSensors propSensors;
  Valves valves;
  NavigationData navigationData;
  Event event;
  Batteries batteries;
  CamsRecording camsRecording;
  UplinkCmd uplinkCmd;
};

class GpsStore : public IStore<GpsBasicFixData> {
public:
  GpsStore();

  void setLon(int32_t lon);
  int32_t getLon() const;

  void setLat(int32_t lat);
  int32_t getLat() const;

  void setHAcc(uint32_t hAcc);
  uint32_t getHAcc() const;

  void setNumSV(uint8_t numSV);
  uint8_t getNumSV() const;

  void setFixType(GpsFixType type);
  GpsFixType getFixType() const;

  void setValidity(GpsValidityFlags valid);
  GpsValidityFlags getValidity() const;

  void setStatusFlags(GpsStatusFlags flags);
  GpsStatusFlags getStatusFlags() const;

};

class GOATStore : public IStore<GpsBasicFixData> {
public:
  GOATStore();

  StateStore stateStore;
  GpsStore gpsStore;
  SensStatusStore sensStatusStore;
  PropSensorsStore propSensorsStore;
  ValvesStore valvesStore;
  NavigationDataStore navigationDataStore;
  EventStore eventStore;
  BatteriesStore batteriesStore;
  CamsRecordingStore camsRecordingStore;
  UplinkCmdStore uplinkCmdStore;

  void set(const DataDump &value);
  const DataDump &get() const;
  DataDump *get_ref();

private:
  mutable DataDump data_;
};

class Data {
public:
  State state;

  Data();
};

class DataManager {
public:
  static Data read();

  static void setState(State state);
};
}; // namespace flight_computer

#endif /* APP_DATA_H */
