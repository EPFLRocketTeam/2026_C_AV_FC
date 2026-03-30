
#include "Application/Data/data.hpp"

using namespace flight_computer;

StateStore::StateStore() { data_ = State::INIT; }

GOATStore::GOATStore() {
  stateStore = StateStore{};
  gpsStore = GpsStore{};
  sensStatusStore = SensStatusStore{};
  vehiculeOverviewStore = VehiculeOverviewStore{};
  flightEventTimersStore = FlightEventTimersStore{};
  propSensorsStore = PropSensorsStore{};
  valvesStore = ValvesStore{};
  navigationDataStore = NavigationDataStore{};
  eventStore = EventStore{};
  batteriesStore = BatteriesStore{};
  camsRecordingStore = CamsRecordingStore{};
  uplinkCmdStore = UplinkCmdStore{};
}

const DataDump &GOATStore::get() const {
  data_.av_state = stateStore.get();
  data_.gps_state = gpsStore.get();
  data_.sensStatus = sensStatusStore.get();
  data_.flightEventTimers = flightEventTimersStore.get();
  data_.vehiculeOverview = vehiculeOverviewStore.get();
  data_.propSensors = propSensorsStore.get();
  data_.valves = valvesStore.get();
  data_.navigationData = navigationDataStore.get();
  data_.event = eventStore.get();
  data_.batteries = batteriesStore.get();
  data_.camsRecording = camsRecordingStore.get();
  data_.uplinkCmd = uplinkCmdStore.get();
  return data_;
}

void GOATStore::set(const DataDump &value) {
  stateStore.set(value.av_state);
  gpsStore.set(value.gps_state);
  sensStatusStore.set(value.sensStatus);
  flightEventTimersStore.set(value.flightEventTimers);
  propSensorsStore.set(value.propSensors);
  valvesStore.set(value.valves);
  navigationDataStore.set(value.navigationData);
  eventStore.set(value.event);
  batteriesStore.set(value.batteries);
  camsRecordingStore.set(value.camsRecording);
  uplinkCmdStore.set(value.uplinkCmd);
}

DataDump *GOATStore::get_ref() { return &data_; }
