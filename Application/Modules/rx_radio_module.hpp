#pragma once

#include "Application/Data/ring_buffer.hpp"
#include "Application/Modules/module.hpp"
#include "Application/app_timebase.h"
#include "Drivers/SX127X/SX127X_capsule.hpp"
#include "Application/Data/data.hpp"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"
#include <cstdio>

#include "Application/app_printf.h"

extern RingBuffer<GpsBasicFixData, 100> gpsData;

#ifndef APP_GPS_POLL_TIMEOUT_MS
#define APP_GPS_POLL_TIMEOUT_MS 0u
#endif

#ifndef APP_GPS_STALE_TIMEOUT_MS
#define APP_GPS_STALE_TIMEOUT_MS 2000u
#endif

/// Uplink counters since the last takeStats().
struct RxRadioStats {
  uint32_t packets = 0;    // radio packets read (before Capsule decoding)
  uint32_t reconfigs = 0;  // radio found again and put back in RX
};

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

	  next_check_ms_ = app_timebase_now_ms() + kCheckPeriodMs;
	  present_ = drivers_[0]->isPresent();
	  if (!present_) {
		  app_printf("[RADIO] RX radio not detected, retrying every %lu ms\r\n",
				  (unsigned long) kCheckPeriodMs);
		  return false;
	  }

	  if (! drivers_[0]->receive(1000)) {
	  		app_printf("Failed to enter in reception mode\r\n");
	  		return false;
	  }

	  return true;
  }

  void update(uint32_t tick_ms) override {
    (void) tick_ms;
    const uint32_t now = app_timebase_now_ms();
    if ((int32_t) (now - next_check_ms_) >= 0) {
      // Two register reads per period: stops polling a missing radio, whose
      // floating MISO can look like received packets, and puts a radio that
      // came back or lost its configuration (reset, brownout) into RX again
      // without the blocking receive().
      next_check_ms_ = now + kCheckPeriodMs;
      const bool present = drivers_[0]->isPresent();
      if (present && (!present_ || !drivers_[0]->isInLoRaRx())) {
        drivers_[0]->reconfigure();
        drivers_[0]->startReceive();
        ++stats_.reconfigs;
      }
      present_ = present;
    }
    if (!present_) {
      return;
    }

    if (drivers_[0]->available()) {
    	drivers_[0]->read();
    	++stats_.packets;
    }
  }

  const RingBuffer<av_uplink_t, 10> &getBuffer(size_t __unused_variable__) const {
    return *buffers_[0];
  }

  bool present() const {
    return present_;
  }

  RxRadioStats takeStats() {
    const RxRadioStats s = stats_;
    stats_ = RxRadioStats{};
    return s;
  }

private:
  static constexpr uint32_t kCheckPeriodMs = 1000;

  bool present_ = false;
  uint32_t next_check_ms_ = 0;
  RxRadioStats stats_{};
};
