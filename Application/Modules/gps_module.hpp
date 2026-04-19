#pragma once

#include "cmsis_os.h"
#include "Application/Data/ring_buffer.hpp"
#include "Application/Modules/module.hpp"
#include "Application/app_timebase.h"
#include "Drivers/UBX_GPS/ubx_gps_interface.h"
#include "Application/Data/data.hpp"
#include <cstdio>

extern osMutexId_t gpsDataMutexHandle;

extern RingBuffer<GpsBasicFixData, 100> gpsData;

#ifndef APP_GPS_POLL_TIMEOUT_MS
#define APP_GPS_POLL_TIMEOUT_MS 0u
#endif

#ifndef APP_GPS_STALE_TIMEOUT_MS
#define APP_GPS_STALE_TIMEOUT_MS 2000u
#endif

class GpsModule
    : public modules::Module<UbxGpsInterface, RingBuffer<GpsBasicFixData, 100>, 1> {
public:
  explicit GpsModule(UbxGpsInterface *(&drivers)[1],
                     RingBuffer<GpsBasicFixData, 100> *(&buffers)[1])
      : Module(drivers, buffers) {}

  bool init() override {
    GpsStatus status = drivers_[0]->init();
    if (status != GpsStatus::OK) {
      printf("Error starting GPS : got %d \n", (int)status);
      return false;
    }
    return true;
  }
  void update(uint32_t tick_ms) override {
    flight_computer::GOATStore& g = flight_computer::GOATStore::get_instance();
    GpsBasicFixData gpsFix{};
    const GpsStatus status =
      drivers_[0]->getPvt(&gpsFix, APP_GPS_POLL_TIMEOUT_MS);
    if (status == GpsStatus::ERROR_TIMEOUT) {
      publishStaleNoFixIfNeeded(tick_ms, g);
      return;
    }
    if (status != GpsStatus::OK) {
      printf("Error with GPS data fetch : got %d \n", (int)status);
      return;
    }

    if (gpsFix.timestamp_us == 0u) {
      gpsFix.timestamp_us = app_timebase_now_us();
    }

    if (gpsDataMutexHandle != nullptr) {
      osMutexAcquire(gpsDataMutexHandle, osWaitForever);
    }
    buffers_[0]->append(gpsFix);
    g.gpsStore.set(gpsFix);
    if (gpsDataMutexHandle != nullptr) {
      osMutexRelease(gpsDataMutexHandle);
    }

    last_fix_tick_ms_ = tick_ms;
    has_fix_ = true;
    stale_no_fix_emitted_ = false;

  }
  const RingBuffer<GpsBasicFixData, 100> &getBuffer(size_t __unused_variable__) const {
    return *buffers_[0];
  }

private:
  void publishStaleNoFixIfNeeded(uint32_t tick_ms,
                                 flight_computer::GOATStore& g) {
    if (!has_fix_ || stale_no_fix_emitted_) {
      return;
    }
    if ((tick_ms - last_fix_tick_ms_) < APP_GPS_STALE_TIMEOUT_MS) {
      return;
    }

    GpsBasicFixData stale_fix = g.gpsStore.get();
    stale_fix.timestamp_us = app_timebase_now_us();
    stale_fix.pps_timestamp_us = 0ULL;
    stale_fix.fixType = GpsFixType::NO_FIX;
    stale_fix.valid.validDate = false;
    stale_fix.valid.validTime = false;
    stale_fix.valid.fullyResolved = false;
    stale_fix.valid.validMag = false;
    stale_fix.flags.gnssFixOK = false;
    stale_fix.flags.diffSoln = false;

    if (gpsDataMutexHandle != nullptr) {
      osMutexAcquire(gpsDataMutexHandle, osWaitForever);
    }
    buffers_[0]->append(stale_fix);
    g.gpsStore.set(stale_fix);
    if (gpsDataMutexHandle != nullptr) {
      osMutexRelease(gpsDataMutexHandle);
    }

    stale_no_fix_emitted_ = true;
  }

  uint32_t last_fix_tick_ms_ = 0u;
  bool has_fix_ = false;
  bool stale_no_fix_emitted_ = false;
};
