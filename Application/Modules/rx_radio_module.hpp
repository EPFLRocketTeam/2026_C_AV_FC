#pragma once

#include "Application/Data/ring_buffer.hpp"
#include "Application/Modules/module.hpp"
#include "Application/app_timebase.h"
#include "Drivers/SX127X/SX127X_capsule.hpp"
#include "Application/Data/data.hpp"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"
#include <cstdio>

extern RingBuffer<GpsBasicFixData, 100> gpsData;

#ifndef APP_GPS_POLL_TIMEOUT_MS
#define APP_GPS_POLL_TIMEOUT_MS 0u
#endif

#ifndef APP_GPS_STALE_TIMEOUT_MS
#define APP_GPS_STALE_TIMEOUT_MS 2000u
#endif

class RxRadioModule
    : public modules::Module<SX127XCapsule, RingBuffer<av_uplink_t, 10>, 1> {
public:
  explicit RxRadioModule(SX127XCapsule *(&drivers)[1],
                     RingBuffer<av_uplink_t, 10> *(&buffers)[1])
      : Module(drivers, buffers) {}

  bool init() override {
	  drivers_[0]->init(864.34e6, SX127X_POWER_11DBM, SX127X_LORA_SF_8,
	  	SX127X_LORA_BW_125KHZ, SX127X_LORA_CR_4_7, SX127X_LORA_CRC_EN,
	  	av_uplink_size);

	  if (! drivers_[0]->receive(1000)) {
	  		printf("Failed to enter in reception mode\r\n");
	  		return false;
	  }

	  return true;
  }

  void update(uint32_t tick_ms) override {

    if (drivers_[0]->available()) {
    	drivers_[0]->read();
    }
  }

  const RingBuffer<av_uplink_t, 10> &getBuffer(size_t __unused_variable__) const {
    return *buffers_[0];
  }
};
