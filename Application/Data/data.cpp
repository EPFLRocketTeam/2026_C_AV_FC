
#include "Application/Data/data.hpp"
#include "Application/app_timebase.h"
#include "Drivers/STM32HAL/stm32hal.h"

#if !defined(UNIT_TEST_ENV)
#include "cmsis_os.h"
#else
using osMutexId_t = void *;
constexpr uint32_t osWaitForever = 0xFFFFFFFFu;
inline int32_t osMutexAcquire(osMutexId_t, uint32_t) { return 0; }
inline int32_t osMutexRelease(osMutexId_t) { return 0; }
#endif

#if defined(UNIT_TEST_ENV)
osMutexId_t eventStoreMutexHandle = nullptr;
osMutexId_t navigationDataMutexHandle = nullptr;
#else
extern osMutexId_t eventStoreMutexHandle;
extern osMutexId_t navigationDataMutexHandle;
#endif

namespace {

struct AppTimebaseState {
  bool initialized = false;
  bool use_dwt = false;
  uint32_t last_tick_ms = 0;
  uint32_t last_cycle = 0;
  uint32_t cycle_remainder = 0;
  uint64_t acc_us = 0;
};

AppTimebaseState g_app_timebase{};

inline void lock_event_store() {
  if (eventStoreMutexHandle != nullptr) {
    osMutexAcquire(eventStoreMutexHandle, osWaitForever);
  }
}

inline void unlock_event_store() {
  if (eventStoreMutexHandle != nullptr) {
    osMutexRelease(eventStoreMutexHandle);
  }
}

inline void lock_navigation_data() {
  if (navigationDataMutexHandle != nullptr) {
    osMutexAcquire(navigationDataMutexHandle, osWaitForever);
  }
}

inline void unlock_navigation_data() {
  if (navigationDataMutexHandle != nullptr) {
    osMutexRelease(navigationDataMutexHandle);
  }
}

#if !defined(UNIT_TEST_ENV)
inline void app_timebase_enable_dwt() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

inline bool app_timebase_is_dwt_running() {
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) {
    return false;
  }
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
    return false;
  }

  const uint32_t a = DWT->CYCCNT;
  __NOP();
  const uint32_t b = DWT->CYCCNT;
  return (a != b);
}
#endif

inline uint32_t app_timebase_enter_critical() {
#if !defined(UNIT_TEST_ENV)
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
#else
  return 0u;
#endif
}

inline void app_timebase_exit_critical(uint32_t primask) {
#if !defined(UNIT_TEST_ENV)
  if ((primask & 0x1u) == 0u) {
    __enable_irq();
  }
#else
  (void)primask;
#endif
}

void app_timebase_init_locked() {
  if (g_app_timebase.initialized) {
    return;
  }

  g_app_timebase.last_tick_ms = HAL_GetTick();
  g_app_timebase.acc_us = static_cast<uint64_t>(g_app_timebase.last_tick_ms) * 1000ULL;
  g_app_timebase.last_cycle = 0u;
  g_app_timebase.cycle_remainder = 0u;

#if !defined(UNIT_TEST_ENV)
  app_timebase_enable_dwt();
  g_app_timebase.use_dwt = app_timebase_is_dwt_running();
  if (g_app_timebase.use_dwt) {
    g_app_timebase.last_cycle = DWT->CYCCNT;
  }
#else
  g_app_timebase.use_dwt = false;
#endif

  g_app_timebase.initialized = true;
}

uint64_t app_timebase_now_us_locked() {
  app_timebase_init_locked();

  if (g_app_timebase.use_dwt) {
#if !defined(UNIT_TEST_ENV)
    const uint32_t cycles_per_us = SystemCoreClock / 1000000u;
    if (cycles_per_us > 0u) {
      const uint32_t now_cycle = DWT->CYCCNT;
      const uint32_t delta_cycles =
          static_cast<uint32_t>(now_cycle - g_app_timebase.last_cycle);
      g_app_timebase.last_cycle = now_cycle;

      const uint64_t total_cycles =
          static_cast<uint64_t>(g_app_timebase.cycle_remainder) +
          static_cast<uint64_t>(delta_cycles);
      g_app_timebase.acc_us += total_cycles / static_cast<uint64_t>(cycles_per_us);
      g_app_timebase.cycle_remainder =
          static_cast<uint32_t>(total_cycles % static_cast<uint64_t>(cycles_per_us));
      return g_app_timebase.acc_us;
    }
#endif
    g_app_timebase.use_dwt = false;
    g_app_timebase.last_tick_ms = HAL_GetTick();
    g_app_timebase.cycle_remainder = 0u;
  }

  const uint32_t now_ms = HAL_GetTick();
  const uint32_t delta_ms =
      static_cast<uint32_t>(now_ms - g_app_timebase.last_tick_ms);
  g_app_timebase.last_tick_ms = now_ms;
  g_app_timebase.acc_us += static_cast<uint64_t>(delta_ms) * 1000ULL;
  return g_app_timebase.acc_us;
}

} // namespace

extern "C" void app_timebase_init(void) {
  const uint32_t primask = app_timebase_enter_critical();
  app_timebase_init_locked();
  app_timebase_exit_critical(primask);
}

extern "C" uint64_t app_timebase_now_us(void) {
  const uint32_t primask = app_timebase_enter_critical();
  const uint64_t now_us = app_timebase_now_us_locked();
  app_timebase_exit_critical(primask);
  return now_us;
}

extern "C" uint32_t app_timebase_now_ms(void) {
  return static_cast<uint32_t>(app_timebase_now_us() / 1000ULL);
}

using namespace flight_computer;

StateStore::StateStore() { data_ = State::INIT; }

GOATStore::GOATStore() {
  stateStore = StateStore{};
  gpsStore = GpsStore{};
  sensStatusStore = SensStatusStore{};
  vehiculeOverviewStore = VehiculeOverviewStore{};
  flightEventTimersStore = FlightEventTimersStore{};
  navSensorStore = NavSensorsStore{};
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
  data_.av_timestamp = app_timebase_now_ms();
  data_.gps_state = gpsStore.get();
  data_.sensStatus = sensStatusStore.get();
  data_.flightEventTimers = flightEventTimersStore.get();
  data_.vehiculeOverview = vehiculeOverviewStore.get();
  data_.navSensors = navSensorStore.get();
  data_.propSensors = propSensorsStore.get();
  data_.valves = valvesStore.get();

  lock_navigation_data();
  data_.navigationData = navigationDataStore.get();
  unlock_navigation_data();

  lock_event_store();
  data_.event = eventStore.get();
  unlock_event_store();

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
  navSensorStore.set(value.navSensors);
  propSensorsStore.set(value.propSensors);
  valvesStore.set(value.valves);

  lock_navigation_data();
  navigationDataStore.set(value.navigationData);
  unlock_navigation_data();

  lock_event_store();
  eventStore.set(value.event);
  unlock_event_store();

  batteriesStore.set(value.batteries);
  camsRecordingStore.set(value.camsRecording);
  uplinkCmdStore.set(value.uplinkCmd);
}

DataDump *GOATStore::get_ref() { return &data_; }
