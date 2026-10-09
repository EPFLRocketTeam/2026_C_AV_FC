#pragma once

#include "Application/main.h"
#include "Application/app_timebase.h"
#include "Application/Data/data.hpp"
#include "Drivers/SX127X/SX127X_capsule.hpp"
#include "Drivers/ERT_RF_Protocol_Interface/DownlinkCompression_Firehorn2.h"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Common.h"
#include "Drivers/ERT_RF_Protocol_Interface/ParameterDefinition_Firehorn2.h"

// Bench switch: 1 sends every packet with the former blocking driver call
// (reconfigure + wait for TxDone, ~92 ms per packet, ~154 ms with no radio),
// to compare against the non-blocking path on the same build.
#ifndef APP_RADIO_BLOCKING_TX
#define APP_RADIO_BLOCKING_TX 0
#endif

#ifndef APP_RADIO_TEST_UPLINK
#define APP_RADIO_TEST_UPLINK 0
#endif

#define PREPARE_DOWNLINK(type) \
    inline void prepare_downlink_packet (av_downlink_unpacked_t &packet, const type &dump)

PREPARE_DOWNLINK(GpsBasicFixData) {
    packet.gnss_alt = dump.height;
    packet.gnss_lat = dump.lat;
    packet.gnss_lon = dump.lon;
}
PREPARE_DOWNLINK(flight_computer::SensStatus) {
    (void) packet; (void) dump;
}
PREPARE_DOWNLINK(flight_computer::VehiculeOverview) {
    packet.rail_cable_status = 0;
    if (dump.no_cable_continuity_engine) packet.rail_cable_status |= AV_CABLE_EXT1;
    if (dump.no_cable_continuity_lox) packet.rail_cable_status |= AV_CABLE_EXT2;

    packet.pyro_status = 0;

    if (dump.pyros_on[0]) packet.pyro_status |= AV_PYRO_CH1;
    if (dump.pyros_on[1]) packet.pyro_status |= AV_PYRO_CH2;
    if (dump.pyros_on[2]) packet.pyro_status |= AV_PYRO_CH3;
    if (dump.pyros_on[3]) packet.pyro_status |= AV_PYRO_CH4;
}
PREPARE_DOWNLINK(flight_computer::FlightEventTimers) {
    (void) packet; (void) dump;
}
PREPARE_DOWNLINK(flight_computer::NavSensors) {
    (void) packet; (void) dump;
}
PREPARE_DOWNLINK(flight_computer::PropSensors) {
	// TODO HPO, HPE temperature
    packet.HPO_pressure = dump.HPO_pressure;
   	packet.HPE_pressure = dump.HPE_pressure;
	packet.fuel_pressure = dump.ETA_pressure;
	packet.LOX_pressure = dump.OTA_pressure;
	packet.LOX_fls_temp_1 = dump.fls_OTA_temperature_1;
	packet.LOX_fls_temp_2 = dump.fls_OTA_temperature_2;
	packet.LOX_fls_temp_3 = dump.fls_OTA_temperature_3;
	packet.LOX_fls_temp_4 = dump.fls_OTA_temperature_4;
	packet.LOX_fls_temp_5 = dump.fls_OTA_temperature_5;
	packet.LOX_fls_temp_6 = dump.fls_OTA_temperature_6;
    packet.fuel_inj_pressure = dump.fuel_inj_pressure;
	packet.LOX_inj_pressure = dump.LOX_inj_pressure;
	packet.chamber_pressure = dump.chamber_pressure;
	packet.chamber_temp = dump.chamber_temperature;
}
PREPARE_DOWNLINK(flight_computer::Valves) {
    // 100% -> degree of opening
    packet.valve_dpr_fuel = dump.ball_valve_fuel * 0.9;
    packet.valve_dpr_LOX  = dump.ball_valve_LOX * 0.9;

    packet.valves_state = 0;
    if (!dump.main_LOX_open)    packet.valves_state |= AV_VALVE_MAIN_LOX;
    if (!dump.main_fuel_open)   packet.valves_state |= AV_VALVE_MAIN_FUEL;
    if (!dump.vent_LOX_open)    packet.valves_state |= AV_VALVE_VENT_LOX;
    if (!dump.vent_fuel_open)   packet.valves_state |= AV_VALVE_VENT_FUEL;
    if (!dump.safety_LOX_open)  packet.valves_state |= AV_VALVE_SDPR_LOX;
    if (!dump.safety_fuel_open) packet.valves_state |= AV_VALVE_SDPR_FUEL;
}
PREPARE_DOWNLINK(flight_computer::NavigationData) {
    packet.agl_altitude   = - dump.position_kalman.z;
    packet.vertical_speed = - dump.speed.z;
    packet.absolute_speed = dump.speed.norm();
}
PREPARE_DOWNLINK(flight_computer::Event) {
    (void) packet; (void) dump;
}
PREPARE_DOWNLINK(flight_computer::Batteries) {
    packet.lpb1_voltage = dump.lpb1_voltage;
	packet.lpb1_current = dump.lpb1_current;
    packet.lpb2_voltage = dump.lpb2_voltage;
	packet.lpb2_current = dump.lpb2_current;
	packet.vout1_5v_voltage = dump.vout1_5v_voltage;
	packet.vout1_5v_current = dump.vout1_5v_current;
	packet.vout2_5v_voltage = dump.vout2_5v_voltage;
	packet.vout2_5v_current = dump.vout2_5v_current;
	packet.hpb_main_voltage = dump.hpb_main_voltage;
	packet.hpb_main_current = dump.hpb_main_current;
	packet.hpb_backup_voltage = dump.hpb_backup_voltage;
	packet.hpb_backup_current = dump.hpb_backup_current;
	packet.vout_24v_voltage = dump.vout_24v_voltage;
	packet.vout_24v_current = dump.vout_24v_current;
}
PREPARE_DOWNLINK(flight_computer::CamsRecording) {
    packet.cam_rec = 0;
    
    if (dump.cam_aero_bot) packet.cam_rec |= AV_CAMERA_AERO_BOT;
    if (dump.cam_aero_top)   packet.cam_rec |= AV_CAMERA_AERO_TOP;
    if (dump.cam_sepmech)  packet.cam_rec |= AV_CAMERA_SEPMEC;
}
PREPARE_DOWNLINK(flight_computer::UplinkCmd) {
    (void) packet; (void) dump;
}

PREPARE_DOWNLINK(flight_computer::DataDump) {
    packet.av_state     = static_cast<uint8_t>(dump.av_state);
    packet.av_fc_temp   = dump.av_fc_temp;
    packet.av_timestamp = dump.av_timestamp;

    packet.sd_fail_count = app_get_sd_fail_count();
    packet.remaining_disk_size = app_get_remaining_disk_size();
    packet.average_imu_rate = app_get_current_imu_rate();

    packet.ambient_temp = dump.ambient_temperature;
    
    prepare_downlink_packet(packet, dump.gps_state);
    prepare_downlink_packet(packet, dump.sensStatus);
    prepare_downlink_packet(packet, dump.vehiculeOverview);
    prepare_downlink_packet(packet, dump.flightEventTimers);
    prepare_downlink_packet(packet, dump.navSensors);
    prepare_downlink_packet(packet, dump.propSensors);
    prepare_downlink_packet(packet, dump.valves);
    prepare_downlink_packet(packet, dump.navigationData);
    prepare_downlink_packet(packet, dump.event);
    prepare_downlink_packet(packet, dump.batteries);
    prepare_downlink_packet(packet, dump.camsRecording);
    prepare_downlink_packet(packet, dump.uplinkCmd);
}

/// Downlink counters since the last takeStats().
struct TxRadioStats {
    uint32_t started = 0;         // transmissions put on air
    uint32_t sent = 0;            // TxDone seen
    uint32_t skipped_busy = 0;    // send() while the previous packet was on air
    uint32_t skipped_absent = 0;  // send() with the radio missing or failing
    uint32_t timeouts = 0;        // no TxDone within kTxTimeoutMs
    uint32_t reconfigs = 0;       // full reconfigurations after a fault
    uint32_t max_airtime_ms = 0;  // longest start-to-TxDone time
};

/// Downlink over the TX radio without blocking the super loop: send() loads
/// the packet and starts the transmission, tick() picks up TxDone. A missing
/// or failing radio is retried every kRetryPeriodMs instead of being reset
/// (100 ms) and reconfigured (15 ms) on every packet.
class TxRadioModule {
private:
    // A downlink is ~75 ms on air (59 B, SF7, BW250, CR4/7).
    static constexpr uint32_t kTxTimeoutMs = 500;
    static constexpr uint32_t kRetryPeriodMs = 5000;
    static constexpr uint32_t kMaxConsecutiveTimeouts = 3;

    SX127XCapsule *driver_;

    uint32_t packet_nbr = 0;
    uint32_t next_time = 0;
    uint32_t ms_between_send;

    bool present_ = false;
    bool tx_busy_ = false;
    uint32_t tx_start_ms_ = 0;
    uint32_t consecutive_timeouts_ = 0;
    uint32_t next_retry_ms_ = 0;
    TxRadioStats stats_{};

    bool usable () const {
        return present_ && consecutive_timeouts_ < kMaxConsecutiveTimeouts;
    }

public:
    explicit TxRadioModule(SX127XCapsule *driver, uint32_t ms_between_send)
      : driver_(driver), ms_between_send(ms_between_send) {}

    bool init () {
    	//app_printf("driver: %p\n", driver_);
#if APP_RADIO_TEST_UPLINK
      driver_->init(864.34e6, SX127X_POWER_11DBM, SX127X_LORA_SF_8,
        SX127X_LORA_BW_125KHZ, SX127X_LORA_CR_4_7, SX127X_LORA_CRC_EN,
        av_uplink_size);
#else
	  driver_->init(866.34e6, SX127X_POWER_20DBM, SX127X_LORA_SF_7,
	  	SX127X_LORA_BW_250KHZ, SX127X_LORA_CR_4_7, SX127X_LORA_CRC_EN,
	  	av_downlink_size);
#endif
	  //app_printf("init is ok.\n");
	  present_ = driver_->isPresent();
	  if (present_) {
	  	driver_->prepareTransmit();
	  } else {
	  	app_printf("[RADIO] TX radio not detected, retrying every %lu ms\r\n",
	  			(unsigned long) kRetryPeriodMs);
	  }
	  next_retry_ms_ = app_timebase_now_ms() + kRetryPeriodMs;
	  return present_;
    }

    /// Call every loop iteration: completes the transmission in progress and
    /// retries a missing or failing radio.
    void tick () {
        const uint32_t now = app_timebase_now_ms();

        if (!usable()) {
            if ((int32_t) (now - next_retry_ms_) < 0) {
                return;
            }
            next_retry_ms_ = now + kRetryPeriodMs;
            present_ = driver_->isPresent();
            if (!present_) {
                return;
            }
            driver_->reconfigure();
            driver_->prepareTransmit();
            ++stats_.reconfigs;
            consecutive_timeouts_ = 0;
            tx_busy_ = false;
            return;
        }

        if (!tx_busy_) {
            return;
        }
        switch (driver_->pollTransmit()) {
        case SX127X_TX_DONE: {
            const uint32_t airtime = now - tx_start_ms_;
            if (airtime > stats_.max_airtime_ms) stats_.max_airtime_ms = airtime;
            ++stats_.sent;
            consecutive_timeouts_ = 0;
            tx_busy_ = false;
            break;
        }
        case SX127X_TX_BUSY:
            if (now - tx_start_ms_ > kTxTimeoutMs) {
                // Back to standby with the TX registers rewritten; a radio
                // that lost its whole configuration keeps timing out and is
                // then fully reconfigured by the retry path above.
                driver_->abortTransmit();
                ++stats_.timeouts;
                ++consecutive_timeouts_;
                tx_busy_ = false;
                next_retry_ms_ = now + kRetryPeriodMs;
            }
            break;
        case SX127X_TX_IDLE:
            tx_busy_ = false;
            break;
        }
    }

    bool should_send () {
        return next_time <= app_timebase_now_ms();
    }

    bool send (const flight_computer::DataDump &dump) {
        const uint32_t now = app_timebase_now_ms();
#if APP_RADIO_TEST_UPLINK
        (void)dump;
        next_time = now + ms_between_send;
        if (tx_busy_) { ++stats_.skipped_busy; return false; }
        if (!usable()) { ++stats_.skipped_absent; return false; }
        // Order 0 is not an actuator or FSM command: the dispatcher ignores it.
        const av_uplink_t noop = {0, 0};
        if (!driver_->startTransmit(GSC_CMD, (uint8_t*)&noop, av_uplink_size)) return false;
        tx_busy_ = true;
        tx_start_ms_ = now;
        ++stats_.started;
        return true;
#endif
        next_time = now + ms_between_send;

        av_downlink_unpacked_t packet;
        packet.packet_nbr = packet_nbr ++;

#if APP_RADIO_BLOCKING_TX
        {
            prepare_downlink_packet(packet, dump);
            av_downlink_t compressed_packet;
            encode_downlink(&compressed_packet, packet);
            ++stats_.started;
            const bool ok = driver_->transmit(CAPSULE_ID::AV_TELEMETRY, (uint8_t*) &compressed_packet, av_downlink_size);
            if (ok) ++stats_.sent; else ++stats_.timeouts;
            return ok;
        }
#endif

        if (!usable()) {
            ++stats_.skipped_absent;
            return false;
        }
        if (tx_busy_) {
            ++stats_.skipped_busy;
            return false;
        }
        // One register read: catches a board unplugged since the last packet,
        // whose floating MISO would otherwise read back as TxDone.
        if (!driver_->isPresent()) {
            present_ = false;
            next_retry_ms_ = now + kRetryPeriodMs;
            ++stats_.skipped_absent;
            return false;
        }

        prepare_downlink_packet(packet, dump);

        struct __attribute__((packed)) Guard {
			volatile uint32_t canary_before = 0xAABBCCDD;
			av_downlink_t     compressed_packet;
			volatile uint32_t canary_after  = 0xDDCCBBAA;
		} guard;

        encode_downlink(&guard.compressed_packet, packet);

        if (!driver_->startTransmit(CAPSULE_ID::AV_TELEMETRY, (uint8_t*) &guard.compressed_packet, av_downlink_size)) {
            ++stats_.skipped_busy;
            return false;
        }
        tx_busy_ = true;
        tx_start_ms_ = now;
        ++stats_.started;
        return true;
    }

    bool present () const {
        return present_;
    }

    TxRadioStats takeStats () {
        const TxRadioStats s = stats_;
        stats_ = TxRadioStats{};
        return s;
    }
};
