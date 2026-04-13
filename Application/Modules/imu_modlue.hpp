#pragma once

#include "cmsis_os.h"
#include "Application/Data/ring_buffer.hpp"
#include "Application/Modules/module.hpp"
#include "Drivers/InvIMU/InvIMU.h"
#include <cstdio>

extern osMutexId_t imuData1MutexHandle;
extern osMutexId_t imuData2MutexHandle;
extern osMutexId_t imuData3MutexHandle;

using namespace Drivers::InvIMU;

extern RingBuffer<IMUData, 100> imuData1;
extern RingBuffer<IMUData, 100> imuData2;
extern RingBuffer<IMUData, 100> imuData3;

inline osMutexId_t getImuLock(size_t index) {
  switch (index) {
    case 0:
      return imuData1MutexHandle;
    case 1:
      return imuData2MutexHandle;
    case 2:
      return imuData3MutexHandle;
    default:
      return nullptr;
  }
}

class ImuModule
    : public modules::Module<InvIMU_Interface, RingBuffer<IMUData, 100>, 3> {
public:
  explicit ImuModule(InvIMU_Interface *(&drivers)[3],
                     RingBuffer<IMUData, 100> *(&buffers)[3])
      : Module(drivers, buffers) {}

  bool init() override {
    for (size_t i = 0; i < kNumSensors; ++i) {
      if (drivers_[i]->init()) {
        printf("Error starting imu %zu \n", i);
        return false;
      }
    }
    return true;
  }
  void update(uint32_t tick_ms) override {
    for (size_t i = 0; i < kNumSensors; ++i) {
      drivers_[i]->tick();
      IMUData frame{};
      osMutexId_t lock = getImuLock(i);
      osMutexAcquire(lock, osWaitForever);
      while (drivers_[i]->getFrame(frame))
        buffers_[i]->append(frame);
      osMutexRelease(lock);
    }
  }
  const RingBuffer<IMUData, 100> &getBuffer(size_t i) const {
    return *buffers_[i];
  }
};
