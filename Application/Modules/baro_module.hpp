#pragma once

#include "Application/Data/data.hpp"
#include "Application/Data/ring_buffer.hpp"
#include "Application/Modules/module.hpp"
#include "Application/app_timebase.h"
#include "Drivers/BMP390/BMP390.h"
#include <cmath>
#include <cstdio>
#include <limits>

extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData1;
extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData2;
extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData3;
extern RingBuffer<Drivers::BMP390::BaroData, 100> baroData4;

#ifndef APP_BARO_COMMAND_TIMEOUT_US
#define APP_BARO_COMMAND_TIMEOUT_US 50000u
#endif

#ifndef APP_BARO_STALE_TIMEOUT_MS
#define APP_BARO_STALE_TIMEOUT_MS 200u
#endif

template <size_t NumSensors = 1>
class BaroModule
    : public modules::Module<Drivers::BMP390::BMP390_Interface,
                             RingBuffer<Drivers::BMP390::BaroData, 100>, NumSensors> {
public:
  using BaroData = Drivers::BMP390::BaroData;
  using BMP390_Interface = Drivers::BMP390::BMP390_Interface;
  using OsrPressure = Drivers::BMP390::OsrPressure;
  using OsrTemp = Drivers::BMP390::OsrTemp;
  using IIRFilter = Drivers::BMP390::IIRFilter;
  using Base = modules::Module<BMP390_Interface, RingBuffer<BaroData, 100>, NumSensors>;
  using Base::drivers_;
  using Base::buffers_;
  static constexpr size_t kNumSensors = NumSensors;


  bool sensorInit(size_t sensor_index) const {
    if (sensor_index >= kNumSensors) {
      return false;
    }
    return sensor_state_[sensor_index].initialized;
  }


  /// Callback type for full-rate raw baro logging.
  /// Called with (sensor_index, sample) when a conversion completes.
  using RawLogCallback = void (*)(size_t sensor_index, const BaroData& sample);

  explicit BaroModule(BMP390_Interface *(&drivers)[NumSensors],
                      RingBuffer<BaroData, 100> *(&buffers)[NumSensors])
      : Base(drivers, buffers) {}

  /// Set a callback to receive every raw baro sample as it's read.
  void setRawLogCallback(RawLogCallback cb) { raw_log_callback_ = cb; }

  bool init() override {
    bool any_initialized = false;
    printf("[BARO-MOD] init: %u sensors\r\n", (unsigned)kNumSensors);
    for (size_t i = 0; i < kNumSensors; ++i) {
      if (drivers_[i] == nullptr) {
        printf("[BARO-MOD] sensor %u: driver is NULL, skip\r\n", (unsigned)i);
        continue;
      }
      printf("[BARO-MOD] sensor %u: calling init()...\r\n", (unsigned)i);
      if (!drivers_[i]->init()) {
        printf("[BARO-MOD] sensor %u: init() FAILED, status=0x%lX\r\n",
               (unsigned)i, (unsigned long)drivers_[i]->getStatus());
        sensor_state_[i].status_flags |= Drivers::BMP390::BMP390_STATUS_SPI_ERROR;
        continue;
      }
      printf("[BARO-MOD] sensor %u: init() OK, calling ping()...\r\n", (unsigned)i);
      if (!drivers_[i]->ping()) {
        printf("[BARO-MOD] sensor %u: ping() FAILED, status=0x%lX\r\n",
               (unsigned)i, (unsigned long)drivers_[i]->getStatus());
        sensor_state_[i].status_flags |=
            Drivers::BMP390::BMP390_STATUS_WHOAMI_MISMATCH;
        continue;
      }
      // TODO(baro-hw): Verify the final BMP390 oversampling/IIR tuple keeps
      // four forced-mode conversions comfortably below the 10 ms control-loop
      // budget on flight hardware.
      drivers_[i]->configure(OsrPressure::x4, OsrTemp::x1, IIRFilter::OFF);
      sensor_state_[i].initialized = true;
      any_initialized = true;
      printf("[BARO-MOD] sensor %u: READY\r\n", (unsigned)i);
    }
    printf("[BARO-MOD] init done: any_initialized=%d\r\n", any_initialized);
    return any_initialized;
  }

  void update(uint32_t tick_ms) override {
    produced_since_last_update_ = 0;
    flight_computer::GOATStore &g = flight_computer::GOATStore::get_instance();

    if (cmd_state_ == CommandState::PENDING) {
      drainPendingConversions(tick_ms, g);
      return;
    }

    triggerConversions(tick_ms, g);
  }

  void setTriggerCallback(void (*callback)(uint64_t)) {
    trigger_callback_ = callback;
  }

  size_t takeProducedCount() {
    const size_t produced = produced_since_last_update_;
    produced_since_last_update_ = 0;
    return produced;
  }

  bool sensorHealthy(size_t sensor_index) const {
    if (sensor_index >= kNumSensors) {
      return false;
    }
    return sensor_state_[sensor_index].healthy;
  }

  uint32_t sensorStatusFlags(size_t sensor_index) const {
    if (sensor_index >= kNumSensors) {
      return Drivers::BMP390::BMP390_STATUS_WHOAMI_MISMATCH;
    }
    return sensor_state_[sensor_index].status_flags;
  }

  uint32_t sensorDropCount(size_t sensor_index) const {
    if (sensor_index >= kNumSensors) {
      return 0u;
    }
    return sensor_state_[sensor_index].drop_count;
  }

private:
  enum class CommandState : uint8_t { IDLE, PENDING };

  struct SensorRuntimeState {
    uint64_t last_timestamp_us = 0;
    uint32_t last_data_tick_ms = 0;
    uint32_t status_flags = Drivers::BMP390::BMP390_STATUS_OK;
    uint32_t drop_count = 0;
    bool initialized = false;
    bool pending = false;
    bool healthy = false;
    bool last_sample_valid = false;
  };

  void triggerConversions(uint32_t tick_ms, flight_computer::GOATStore &g) {
    const uint64_t trigger_ts = app_timebase_now_us();
    bool any_pending = false;

    for (size_t i = 0; i < kNumSensors; ++i) {
      auto &state = sensor_state_[i];
      if (!state.initialized || drivers_[i] == nullptr) {
        continue;
      }
      drivers_[i]->triggerMeasurement();
      state.pending = true;
      state.status_flags = drivers_[i]->getStatus();
      state.healthy = isHealthyStatus(i, tick_ms);
      any_pending = true;
    }

    if (!any_pending) {
      return;
    }

    trigger_ts_us_ = trigger_ts;
    pending_started_us_ = trigger_ts;
    for (size_t i = 0; i < kNumSensors; ++i) {
      if (!sensor_state_[i].initialized) {
        publishInvalidSample(i, g, tick_ms,
                             Drivers::BMP390::BMP390_STATUS_WHOAMI_MISMATCH);
      }
    }

    cmd_state_ = CommandState::PENDING;
    if (trigger_callback_ != nullptr) {
      trigger_callback_(trigger_ts);
    }
  }

  void drainPendingConversions(uint32_t tick_ms, flight_computer::GOATStore &g) {
    bool any_pending = false;
    for (size_t i = 0; i < kNumSensors; ++i) {
      auto &state = sensor_state_[i];
      if (!state.pending || drivers_[i] == nullptr) {
        continue;
      }

      BaroData sample{};
      if (drivers_[i]->getFrame(sample)) {
        sample.timestamp_us = trigger_ts_us_;
        buffers_[i]->append(sample);
        g.navSensorStore.set_bmp(i, {sample.temperature_c, sample.pressure_pa});
        state.pending = false;
        state.last_timestamp_us = sample.timestamp_us;
        state.last_data_tick_ms = tick_ms;
        state.last_sample_valid = true;
        ++produced_since_last_update_;
        if (raw_log_callback_) {
          raw_log_callback_(i, sample);
        }
      } else {
        any_pending = true;
      }

      state.status_flags = drivers_[i]->getStatus();
      state.healthy = isHealthyStatus(i, tick_ms);
    }

    if (!any_pending) {
      clearPendingState();
      return;
    }

    const uint64_t now_us = app_timebase_now_us();
    if (pending_started_us_ > 0u &&
        now_us >= pending_started_us_ &&
        (now_us - pending_started_us_) >= APP_BARO_COMMAND_TIMEOUT_US) {
      publishTimedOutSensors(g, tick_ms);
      clearPendingState();
    }
  }

  void publishTimedOutSensors(flight_computer::GOATStore &g, uint32_t tick_ms) {
    for (size_t i = 0; i < kNumSensors; ++i) {
      auto &state = sensor_state_[i];
      if (!state.pending) {
        continue;
      }

      state.pending = false;
      // Match ktp-soft behavior: drop the sample silently on timeout instead
      // of pushing NaN into the ring buffer.  Only update status/health so
      // downstream consumers see the failure without having to filter NaN.
      state.status_flags |= Drivers::BMP390::BMP390_STATUS_NOT_READY;
      state.last_sample_valid = false;
      state.healthy = isHealthyStatus(i, tick_ms);
      ++state.drop_count;
    }
  }

  void publishInvalidSample(size_t sensor_index,
                            flight_computer::GOATStore &g,
                            uint32_t tick_ms,
                            uint32_t status_flag) {
    if (sensor_index >= kNumSensors) {
      return;
    }
    auto &state = sensor_state_[sensor_index];
    // Don't append NaN to the ring buffer — uninitialised sensors are
    // permanently unavailable; there is no sample to push.  Update the
    // GOATStore with zeros so telemetry reflects the missing sensor and
    // increment produced count so takeProducedCount bookkeeping stays
    // consistent.
    g.navSensorStore.set_bmp(sensor_index, {0.0, 0.0});
    state.status_flags |= status_flag;
    state.last_timestamp_us = trigger_ts_us_;
    state.last_data_tick_ms = tick_ms;
    state.last_sample_valid = false;
    ++state.drop_count;
  }

  void clearPendingState() {
    for (auto &state : sensor_state_) {
      state.pending = false;
    }
    cmd_state_ = CommandState::IDLE;
    trigger_ts_us_ = 0;
    pending_started_us_ = 0;
  }

  bool isHealthyStatus(size_t sensor_index, uint32_t tick_ms) const {
    const auto &state = sensor_state_[sensor_index];
    constexpr uint32_t kCriticalFlags =
        Drivers::BMP390::BMP390_STATUS_SPI_ERROR |
        Drivers::BMP390::BMP390_STATUS_WHOAMI_MISMATCH |
        Drivers::BMP390::BMP390_STATUS_CALIB_ERROR;
    const bool has_critical = (state.status_flags & kCriticalFlags) != 0u;
    const bool stale = (state.last_data_tick_ms == 0u) ||
                       ((tick_ms - state.last_data_tick_ms) >
                        APP_BARO_STALE_TIMEOUT_MS);
    return state.initialized && state.last_sample_valid && !has_critical && !stale;
  }

  SensorRuntimeState sensor_state_[kNumSensors] = {};
  CommandState cmd_state_ = CommandState::IDLE;
  uint64_t trigger_ts_us_ = 0;
  uint64_t pending_started_us_ = 0;
  size_t produced_since_last_update_ = 0;
  void (*trigger_callback_)(uint64_t) = nullptr;
  RawLogCallback raw_log_callback_ = nullptr;
};
