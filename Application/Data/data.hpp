
#ifndef APP_DATA_H
#define APP_DATA_H

#include <cmath>
#include <cstdint>

#include "Application/Data/fsm.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"
#include "Drivers/InvIMU/InvIMU.h"

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
  uint8_t adxl_status;
  uint8_t adxl_aux_status;
  uint8_t bmi_accel_status;
  uint8_t bmi_aux_accel_status;
  uint8_t bmi_gyro_status;
  uint8_t bmi_aux_gyro_status;
  bmp3_status bmp_status;
  bmp3_status bmp_aux_status;

  // uint8_t no_cable_continuity; // TODO: Properly update the store
  // uint8_t flight_duration;     // TODO: Properly update the store
  // uint8_t descent_duration;    // TODO: Properly update the store

  SensStatus();
};

struct FlightEventTimers {// TODO: Properly update the store test
  uint32_t flight_duration;
  uint32_t ascent_duration;
  uint32_t descent_duration;

  FlightEventTimers();
};

struct VehiculeOverview {// TODO: Properly update the store test
  uint8_t no_cable_continuity;

  bool pyros_on[4];

  VehiculeOverview();
};

struct adxl375_data {
  float x;
  float y;
  float z;

  adxl375_data() : x(0.0f), y(0.0f), z(0.0f) {}
  adxl375_data(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};



struct NavSensors {
  adxl375_data adxl;
  adxl375_data adxl_aux;
  Drivers::InvIMU::IMUData imu[4];  // imu[0]=imu_1, imu[1]=imu_2, imu[2]=imu_3, imu[3]=imu_4
  bmp3_data bmp;      // bmp_1
  bmp3_data bmp_aux;  // bmp_2, retained for telemetry compatibility
  bmp3_data bmp_3;
  bmp3_data bmp_4;

  NavSensors();
};

struct PropSensors {
  // === FUEL ===
  double HPE_pressure;
  double ETA_pressure;

  // === LOX ===
  double HPO_pressure;
  double OTA_pressure;

  // on lox tank, connected to PRC-lox
  //  1 -> highest on rocket
  //  4 -> lowest 
  double fls_OTA_temperature_1;
  double fls_OTA_temperature_2;
  double fls_OTA_temperature_3;
  double fls_OTA_temperature_4;

  // === ENGINE ===
  
  // 5 -> lowest
  double fls_OTA_temperature_5;

  double fuel_inj_pressure;
  double fuel_inj_temperature;
  double LOX_inj_pressure;
  double LOX_inj_temperature;
  double chamber_pressure;
  double chamber_temperature;

  // === STATE ===
  uint8_t dpr_fuel_state;
  uint8_t dpr_LOX_state;
  uint8_t engine_state;

  uint32_t timer_burn;

  PropSensors();
};

struct Valves {
  bool main_LOX_open, main_fuel_open;

  bool vent_LOX_open, vent_fuel_open;
  bool safety_LOX_open, safety_fuel_open;

  float ball_valve_LOX, ball_valve_fuel;

  Valves();
};

struct Vector3 {
  double x;
  double y;
  double z;

  Vector3() : x(0.0), y(0.0), z(0.0) {}
  Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

  inline double norm() const { return std::sqrt(x * x + y * y + z * z); }
};

struct NavigationData {
  Vector3 position_kalman;
  Vector3 speed;
  Vector3 accel;
  Vector3 attitude;
  // Heading from attitude yaw (degrees), not ground-track.
  double course;
  double altitude;
  bmp3_data baro;

  NavigationData();
};

struct Batteries {
  float lpb_voltage;
	float lpb_current;
	float vout_5v_voltage;
	float vout_5v_current;
	float hpb_main_voltage;
	float hpb_main_current;
	float hpb_backup_voltage;
	float hpb_backup_current;
	float vout_24v_voltage;
	float vout_24v_current;

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

/**
 * @brief Generic base class for all data stores.
 *
 * Provides a simple, uniform interface to store, retrieve, and reference
 * a single value of type T. Concrete store classes derive from this
 * template to add domain-specific accessors on top of these primitives.
 *
 * @tparam T The type of the value held by this store.
 */
template <typename T> class IStore {
public:
  virtual ~IStore() = default;

  /**
   * @brief Overwrite the stored value with a copy of @p value.
   * @param value New value to store.
   */
  inline void set(const T &value) { data_ = value; };

  /**
   * @brief Return a const reference to the currently stored value.
   * @return Const reference to the internal data.
   */
  inline const T &get() const { return data_; };

  /**
   * @brief Return a mutable pointer to the internally stored value.
   *
   * Useful when an external subsystem needs to write directly into the
   * store without going through a copy (e.g. passing the address to a
   * driver that fills the struct in-place).
   *
   * @return Pointer to the internal data.
   */
  inline T *get_ref() { return &data_; };

protected:
  T data_;
};

class BatteriesStore : public IStore<Batteries> {
public:
  BatteriesStore();

  float get_lpb_voltage () const;
  void set_lpb_voltage (float value);

  float get_lpb_current () const;
  void set_lpb_current (float value);

  float get_vout_5v_voltage () const;
  void set_vout_5v_voltage (float value);

  float get_vout_5v_current () const;
  void set_vout_5v_current (float value);

  float get_hpb_main_voltage () const;
  void set_hpb_main_voltage (float value);

  float get_hpb_main_current () const;
  void set_hpb_main_current (float value);

  float get_hpb_backup_voltage () const;
  void set_hpb_backup_voltage (float value);

  float get_hpb_backup_current () const;
  void set_hpb_backup_current (float value);

  float get_vout_24v_voltage () const;
  void set_vout_24v_voltage (float value);

  float get_vout_24v_current () const;
  void set_vout_24v_current (float value);
};

class FlightEventTimersStore : public IStore<FlightEventTimers> {
public:
  FlightEventTimersStore();

  uint32_t get_flight_timer() const;
  void set_flight_timer(uint32_t value);

  uint32_t get_ascent_timer() const;
  void set_ascent_timer(uint32_t value);

  uint32_t get_descent_timer() const;
  void set_descent_timer(uint32_t value);
};

class VehiculeOverviewStore : public IStore<VehiculeOverview> {
public:
  VehiculeOverviewStore();

  uint8_t get_no_cable_continuity() const;
  void set_no_cable_continuity(uint8_t value);

  bool get_pyro_ch1_on () const;
  bool get_pyro_ch2_on () const;
  bool get_pyro_ch3_on () const;
  bool get_pyro_ch4_on () const;

  void set_pyro_ch1_on (bool value);
  void set_pyro_ch2_on (bool value);
  void set_pyro_ch3_on (bool value);
  void set_pyro_ch4_on (bool value);
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
  void set_bmp_status(const bmp3_status &value);

  bmp3_status get_bmp_aux_status() const;
  void set_bmp_aux_status(const bmp3_status &value);
};

class NavSensorsStore : public IStore<NavSensors> {
public:
  NavSensorsStore();

  adxl375_data get_adxl() const;
  void set_adxl(const adxl375_data &data);

  adxl375_data get_adxl_aux() const;
  void set_adxl_aux(const adxl375_data &data);

  // index 0 = imu_1, 1 = imu_2, 2 = imu_3. Out-of-bounds: get returns IMUData{}, set is a no-op.
  Drivers::InvIMU::IMUData get_imu(size_t index) const;
  void set_imu(size_t index, const Drivers::InvIMU::IMUData &data);

  bmp3_data get_bmp() const;
  void set_bmp(const bmp3_data &data);

  bmp3_data get_bmp_aux() const;
  void set_bmp_aux(const bmp3_data &data);

  // index 0 = bmp_1, ..., 3 = bmp_4. Out-of-bounds: get returns bmp3_data{}, set is a no-op.
  bmp3_data get_bmp(size_t index) const;
  void set_bmp(size_t index, const bmp3_data &data);
};

class PropSensorsStore : public IStore<PropSensors> {
public:
  PropSensorsStore();

  double get_HPE_pressure () const;
  void set_HPE_pressure (double value);

  double get_ETA_pressure () const;
  void set_ETA_pressure (double value);

  double get_HPO_pressure () const;
  void set_HPO_pressure (double value);

  double get_OTA_pressure () const;
  void set_OTA_pressure (double value);

  double get_fls_OTA_temperature_1 () const;
  void set_fls_OTA_temperature_1 (double value);

  double get_fls_OTA_temperature_2 () const;
  void set_fls_OTA_temperature_2 (double value);

  double get_fls_OTA_temperature_3 () const;
  void set_fls_OTA_temperature_3 (double value);

  double get_fls_OTA_temperature_4 () const;
  void set_fls_OTA_temperature_4 (double value);

  double get_fls_OTA_temperature_5 () const;
  void set_fls_OTA_temperature_5 (double value);

  double get_fuel_inj_pressure () const;
  void set_fuel_inj_pressure (double value);

  double get_fuel_inj_temperature () const;
  void set_fuel_inj_temperature (double value);

  double get_LOX_inj_pressure () const;
  void set_LOX_inj_pressure (double value);

  double get_LOX_inj_temperature () const;
  void set_LOX_inj_temperature (double value);

  double get_chamber_pressure () const;
  void set_chamber_pressure (double value);

  double get_chamber_temperature () const;
  void set_chamber_temperature (double value);

  uint8_t get_dpr_fuel_state () const;
  void set_dpr_fuel_state (uint8_t value);

  uint8_t get_dpr_LOX_state () const;
  void set_dpr_LOX_state (uint8_t value);

  uint8_t get_engine_state () const;
  void set_engine_state (uint8_t value);
  
  uint32_t get_timer_burn () const;
  void set_timer_burn (uint32_t value_ms);
};

class ValvesStore : public IStore<Valves> {
public:
  ValvesStore();

  bool  get_main_LOX_open () const;
  void  set_main_LOX_open (bool value);

  bool  get_main_fuel_open () const;
  void  set_main_fuel_open (bool value);

  bool  get_vent_LOX_open () const;
  void  set_vent_LOX_open (bool value);

  bool  get_vent_fuel_open () const;
  void  set_vent_fuel_open (bool value);

  bool  get_safety_LOX_open () const;
  void  set_safety_LOX_open (bool value);

  bool  get_safety_fuel_open () const;
  void  set_safety_fuel_open (bool value);

  float get_ball_valve_LOX () const;
  void  set_ball_valve_LOX (float value);

  float get_ball_valve_fuel () const;
  void  set_ball_valve_fuel (float value);
};

class NavigationDataStore : public IStore<NavigationData> {
public:
  NavigationDataStore();

  Vector3 get_position_kalman() const;
  void set_position_kalman(const Vector3 &value);

  Vector3 get_speed() const;
  void set_speed(const Vector3 &value);

  Vector3 get_accel() const;
  void set_accel(const Vector3 &value);

  Vector3 get_attitude() const;
  void set_attitude(const Vector3 &value);

  double get_course() const;
  void set_course(double value);

  double get_altitude() const;
  void set_altitude(double value);

  bmp3_data get_baro() const;
  void set_baro(const bmp3_data &value);
};

/// Tri-state encoding for the vertical-acceleration-hold liftoff event.
/// Used during IGNITION to decide whether the motor actually ignited:
///   NOT_ELAPSED      – the evaluation window has not finished yet; no decision.
///   ACC_DID_HOLD     – mean vertical accel exceeded the threshold for the
///                      required duration → transition to BURN.
///   ACC_DID_NOT_HOLD – the window elapsed but acceleration was insufficient
///                      → transition to ABORT_ON_GROUND.
enum AccHoldStatus : uint8_t {
  ACC_HOLD_NOT_ELAPSED     = 0,
  ACC_HOLD_DID_HOLD        = 1,
  ACC_HOLD_DID_NOT_HOLD    = 2,
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

  bool timer_launch_delay; // TODO: update the store
  bool cut_off_detected;   // TODO: update the store
  bool apogee_detected;    // TODO: update the store
  bool touchdown_detected; // TODO: update the store

  /// Liftoff acceleration-hold evaluation result.
  /// Written by the Kalman subsystem once the FSM enters IGNITION.
  /// @see AccHoldStatus
  uint8_t vertical_acc_hold;

  /// IMU-based dual-window liftoff detection (set by LiftoffDetector).
  bool imu_liftoff_detected;

  // bool ascent_max_duration;  // TODO: update the store
  // bool descent_max_duration; // TODO: update the store

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

  bool get_imu_liftoff_detected() const;
  void set_imu_liftoff_detected(bool value);

  bool get_cut_off_detected() const;
  void set_cut_off_detected(bool value);
};

class StateStore : public IStore<State> {
public:
  StateStore();
};

struct DataDump {
  State av_state; // Y
  // Timestamp contract: GOATStore::get() stamps this field from the shared
  // AV monotonic timebase in milliseconds before AvState::update consumes it.
  uint32_t av_timestamp; // Y
  float av_fc_temp; // X
  GpsBasicFixData gps_state; // X
  SensStatus sensStatus; // X
  VehiculeOverview vehiculeOverview; // X
  FlightEventTimers flightEventTimers; // X
  NavSensors navSensors; // Y
  PropSensors propSensors; // X
  Valves valves; // X
  NavigationData navigationData; // Y
  Event event; // Y
  Batteries batteries; // X
  CamsRecording camsRecording; // X
  UplinkCmd uplinkCmd; // X
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
  VehiculeOverviewStore vehiculeOverviewStore;
  FlightEventTimersStore flightEventTimersStore;
  PropSensorsStore propSensorsStore;
  NavSensorsStore navSensorStore;
  ValvesStore valvesStore;
  NavigationDataStore navigationDataStore;
  EventStore eventStore;
  BatteriesStore batteriesStore;
  CamsRecordingStore camsRecordingStore;
  UplinkCmdStore uplinkCmdStore;

  void set(const DataDump &value);
  const DataDump &get() const;
  DataDump *get_ref();
  static inline GOATStore &get_instance() {
    static GOATStore instance;
    return instance;
  }

private:
  mutable DataDump data_;
};

}; // namespace flight_computer

#endif /* APP_DATA_H */
