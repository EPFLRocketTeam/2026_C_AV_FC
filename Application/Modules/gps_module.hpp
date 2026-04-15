#pragma once

#include "cmsis_os.h"
#include "Application/Data/ring_buffer.hpp"
#include "Application/Modules/module.hpp"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"
#include "Application/Data/data.hpp"
#include <cstdio>

extern osMutexId_t gpsDataMutexHandle;

extern RingBuffer<GpsBasicFixData, 100> gpsData;

class GpsModule
    : public modules::Module<UbxGpsInterface, RingBuffer<GpsBasicFixData, 100>, 1> {
public:
  explicit GpsModule(UbxGpsInterface *(&drivers)[1],
                     RingBuffer<GpsBasicFixData, 100> *(&buffers)[1])
      : Module(drivers, buffers) {}

  bool init() override {
	 GpsStatus status = drivers_[0]->init();
	 if (status == GpsStatus::OK) {
	     printf("Error starting GPS : got %d \n", (int)status);
	     return false;
	 }
    return true;
  }
  void update(uint32_t tick_ms) override {
    flight_computer::GOATStore& g = flight_computer::GOATStore::get_instance();
	GpsBasicFixData gpsFix{};
	osMutexAcquire(gpsDataMutexHandle, osWaitForever);
	GpsStatus status = drivers_[0]->getPvt(&gpsFix, 150);
	if (status != GpsStatus::OK) {
		printf("Error with GPS data fetch : got %d \n", (int)status);
	}
	buffers_[0]->append(gpsFix);
	g.gpsStore.set(gpsFix);
	osMutexRelease(gpsDataMutexHandle);

  }
  const RingBuffer<GpsBasicFixData, 100> &getBuffer(size_t __unused_variable__) const {
    return *buffers_[0];
  }
};
