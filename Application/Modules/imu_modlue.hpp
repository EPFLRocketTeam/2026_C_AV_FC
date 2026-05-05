#pragma once

#include "Application/Data/ring_buffer.hpp"
#include "Application/Modules/module.hpp"
#include "Drivers/InvIMU/InvIMU.h"
#include "Drivers/STM32HAL/stm32hal.h"
#include "Application/Data/data.hpp"
#include <cstdio>

using namespace Drivers::InvIMU;

#ifndef APP_IMU_RING_CAPACITY
#define APP_IMU_RING_CAPACITY 128u
#endif

using AppImuRingBuffer = RingBuffer<IMUData, APP_IMU_RING_CAPACITY>;

extern AppImuRingBuffer imuData1;
extern AppImuRingBuffer imuData2;
extern AppImuRingBuffer imuData3;

#ifndef APP_IMU_ENABLE_FSYNC
#define APP_IMU_ENABLE_FSYNC 1u
#endif

#ifndef APP_IMU_SOFT_TRIGGER_SECONDARIES
#define APP_IMU_SOFT_TRIGGER_SECONDARIES 1u
#endif

#ifndef APP_IMU_SOFT_TRIGGER_MASTER_FALLBACK
#define APP_IMU_SOFT_TRIGGER_MASTER_FALLBACK 1u
#endif

#ifndef APP_IMU_ENABLE_FAILOVER
#define APP_IMU_ENABLE_FAILOVER 1u
#endif

#ifndef APP_IMU_MASTER_TIMEOUT_MS
#define APP_IMU_MASTER_TIMEOUT_MS 20u
#endif

#ifndef APP_IMU_ALIGNMENT_SPIN_TRIES
#define APP_IMU_ALIGNMENT_SPIN_TRIES 2u
#endif

#ifndef APP_IMU_STALE_TIMEOUT_MS
#define APP_IMU_STALE_TIMEOUT_MS 50u
#endif

template <size_t NumSensors = 1u,
          size_t BufferCapacity = APP_IMU_RING_CAPACITY>
class ImuModule
    : public modules::Module<InvIMU_Interface,
                             RingBuffer<IMUData, BufferCapacity>,
                             NumSensors> {
public:
  using BufferT = RingBuffer<IMUData, BufferCapacity>;
  using Base = modules::Module<InvIMU_Interface, BufferT, NumSensors>;
  using Base::buffers_;
  using Base::drivers_;
  static constexpr size_t kNumSensors = NumSensors;

  explicit ImuModule(InvIMU_Interface *(&drivers)[NumSensors],
                     BufferT *(&buffers)[NumSensors])
      : Base(drivers, buffers) {}

  bool init() override {
    for (size_t i = 0; i < kNumSensors; ++i) {
      if (!drivers_[i]->init()) {
        printf("Error starting imu %zu\n", i);
        return false;
      }
      if (!drivers_[i]->ping()) {
        printf("Error pinging imu %zu\n", i);
        return false;
      }
      drivers_[i]->configure(AccelRange::_32G, GyroRange::_4000DPS, ODR::_6_4kHz);
      drivers_[i]->configureFifo();
#if (APP_IMU_ENABLE_FSYNC != 0u)
      drivers_[i]->enableFsync();
#endif
      sensor_state_[i] = SensorRuntimeState{};
    }
    last_master_irq_ms_ = HAL_GetTick();
    return true;
  }

  void onImuInterrupt(size_t sensor_index, uint32_t irq_tick_ms) {
    if (sensor_index >= kNumSensors) {
      return;
    }
    if (sensor_index != master_index_) {
      return;
    }

    master_irq_pending_ = true;
    last_master_irq_ms_ = irq_tick_ms;
  }

  void onSpiRxComplete(SPI_HandleTypeDef *hspi) {
    (void)hspi;
    for (size_t i = 0; i < kNumSensors; ++i) {
      drivers_[i]->onDmaComplete();
    }
  }

  void update(uint32_t tick_ms) override {
    flight_computer::GOATStore& g = flight_computer::GOATStore::get_instance();
    produced_since_last_update_ = 0;

#if (APP_IMU_ENABLE_FAILOVER != 0u) && (APP_IMU_SOFT_TRIGGER_MASTER_FALLBACK == 0u)
    if ((tick_ms - last_master_irq_ms_) > APP_IMU_MASTER_TIMEOUT_MS) {
      master_index_ = (master_index_ + 1u) % kNumSensors;
      master_irq_pending_ = false;
      last_master_irq_ms_ = tick_ms;
      failover_occurred_ = true;
      printf("ImuModule: master failover to imu %zu\n", master_index_);
    }
#endif

    if (master_irq_pending_) {
      drivers_[master_index_]->onInterrupt();
      master_irq_pending_ = false;
    }
#if (APP_IMU_SOFT_TRIGGER_MASTER_FALLBACK != 0u)
    else {
      drivers_[master_index_]->onInterrupt();
    }
#endif

#if (APP_IMU_SOFT_TRIGGER_SECONDARIES != 0u)
    for (size_t i = 0; i < kNumSensors; ++i) {
      if (i == master_index_) {
        continue;
      }
      drivers_[i]->onInterrupt();
    }
#endif

    const size_t master_produced = drainSensor(master_index_, g, tick_ms);
    if (master_produced > 0u) {
      last_master_irq_ms_ = tick_ms;
    }

    for (size_t i = 0; i < kNumSensors; ++i) {
      if (i == master_index_) {
        continue;
      }
      (void)drainSensor(i, g, tick_ms);
      alignSecondaryToMaster(i, tick_ms, g);
    }

    for (size_t i = 0; i < kNumSensors; ++i) {
      updateSensorHealth(i, tick_ms);
    }
  }

  size_t takeProducedCount() {
    const size_t produced = produced_since_last_update_;
    produced_since_last_update_ = 0;
    return produced;
  }

  const BufferT &getBuffer(size_t i) const {
    return *buffers_[i];
  }

  bool hasFailoverOccurred() const {
    return failover_occurred_;
  }

  size_t masterIndex() const {
    return master_index_;
  }

  bool sensorHealthy(size_t sensor_index) const {
    if (sensor_index >= kNumSensors) {
      return false;
    }
    return sensor_state_[sensor_index].healthy;
  }

  uint32_t sensorStatusFlags(size_t sensor_index) const {
    if (sensor_index >= kNumSensors) {
      return IMU_STATUS_OK;
    }
    return sensor_state_[sensor_index].status_flags;
  }

  uint32_t sensorDropCount(size_t sensor_index) const {
    if (sensor_index >= kNumSensors) {
      return 0u;
    }
    return sensor_state_[sensor_index].drop_count;
  }

  uint32_t sensorAlignmentMismatches(size_t sensor_index) const {
    if (sensor_index >= kNumSensors) {
      return 0u;
    }
    return sensor_state_[sensor_index].alignment_mismatch_count;
  }

private:
  struct SensorRuntimeState {
    uint64_t last_timestamp_us = 0;
    uint32_t last_data_tick_ms = 0;
    uint32_t status_flags = IMU_STATUS_OK;
    uint32_t drop_count = 0;
    uint32_t alignment_mismatch_count = 0;
    bool healthy = false;
  };

  size_t drainSensor(size_t sensor_index, flight_computer::GOATStore &g,
                     uint32_t tick_ms) {
    drivers_[sensor_index]->tick();

    IMUData frame{};
    size_t produced = 0;
    while (drivers_[sensor_index]->getFrame(frame)) {
      buffers_[sensor_index]->append(frame);
      g.navSensorStore.set_imu(sensor_index, frame);
      sensor_state_[sensor_index].last_timestamp_us = frame.timestamp_us;
      ++produced_since_last_update_;
      ++produced;
    }
    sensor_state_[sensor_index].status_flags = drivers_[sensor_index]->statusFlags();
    sensor_state_[sensor_index].drop_count = drivers_[sensor_index]->dropCount();
    if (produced > 0u) {
      sensor_state_[sensor_index].last_data_tick_ms = tick_ms;
    }

    return produced;
  }

  void alignSecondaryToMaster(size_t secondary_index, uint32_t tick_ms,
                              flight_computer::GOATStore &g) {
    if (secondary_index >= kNumSensors || secondary_index == master_index_) {
      return;
    }

    const uint64_t master_ts = sensor_state_[master_index_].last_timestamp_us;
    if (master_ts == 0u) {
      return;
    }
    if (sensor_state_[secondary_index].last_timestamp_us >= master_ts) {
      return;
    }

    for (uint32_t tries = 0; tries < APP_IMU_ALIGNMENT_SPIN_TRIES; ++tries) {
      drivers_[secondary_index]->onInterrupt();
      (void)drainSensor(secondary_index, g, tick_ms);
      if (sensor_state_[secondary_index].last_timestamp_us >= master_ts) {
        return;
      }
    }

    ++sensor_state_[secondary_index].alignment_mismatch_count;
  }

  void updateSensorHealth(size_t sensor_index, uint32_t tick_ms) {
    if (sensor_index >= kNumSensors) {
      return;
    }

    auto &state = sensor_state_[sensor_index];
    const uint32_t critical_flags = IMU_STATUS_SPI_ERROR | IMU_STATUS_WHOAMI_MISMATCH;
    const bool has_critical = (state.status_flags & critical_flags) != 0u;
    const bool stale = (state.last_data_tick_ms == 0u) ||
                       ((tick_ms - state.last_data_tick_ms) > APP_IMU_STALE_TIMEOUT_MS);
    state.healthy = (!has_critical) && (!stale);
  }

  size_t produced_since_last_update_ = 0;
  size_t master_index_ = 0;
  bool master_irq_pending_ = false;
  uint32_t last_master_irq_ms_ = 0;
  bool failover_occurred_ = false;
  SensorRuntimeState sensor_state_[kNumSensors] = {};
};
