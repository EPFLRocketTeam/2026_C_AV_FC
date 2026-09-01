#pragma once

#include "Application/Data/data.hpp"
#include "Drivers/SX127X/SX127X_capsule.hpp"
#include "Drivers/ERT_RF_Protocol_Interface/DownlinkCompression_Firehorn2.h"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Common.h"
#include "Drivers/ERT_RF_Protocol_Interface/ParameterDefinition_Firehorn2.h"

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
    // TODO no_cable_continuity ?
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
    packet.HPO_pressure = dump.HPO_pressure;
   	packet.HPE_pressure = dump.HPE_pressure;
	packet.fuel_pressure = dump.ETA_pressure;
	packet.LOX_pressure = dump.OTA_pressure;
	packet.LOX_fls_temp_1 = dump.fls_OTA_temperature_1;
	packet.LOX_fls_temp_2 = dump.fls_OTA_temperature_2;
	packet.LOX_fls_temp_3 = dump.fls_OTA_temperature_3;
	packet.LOX_fls_temp_4 = dump.fls_OTA_temperature_4;
	packet.LOX_fls_temp_5 = dump.fls_OTA_temperature_5;
    packet.fuel_inj_pressure = dump.fuel_inj_pressure;
	packet.LOX_inj_pressure = dump.LOX_inj_pressure;
	packet.chamber_pressure = dump.chamber_pressure;
	packet.chamber_temp = dump.chamber_temperature;
}
PREPARE_DOWNLINK(flight_computer::Valves) {
    packet.valve_dpr_fuel = dump.ball_valve_fuel;
    packet.valve_dpr_LOX  = dump.ball_valve_LOX;

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
    packet.lpb_voltage = dump.lpb_voltage;
	packet.lpb_current = dump.lpb_current;
	packet.vout_5v_voltage = dump.vout_5v_voltage;
	packet.vout_5v_current = dump.vout_5v_current;
	packet.hpb_main_voltage = dump.hpb_main_voltage;
	packet.hpb_main_current = dump.hpb_main_current;
	packet.hpb_backup_voltage = dump.hpb_backup_voltage;
	packet.hpb_backup_current = dump.hpb_backup_current;
	packet.vout_24v_voltage = dump.vout_24v_voltage;
	packet.vout_24v_current = dump.vout_24v_current;
}
PREPARE_DOWNLINK(flight_computer::CamsRecording) {
    packet.cam_rec = 0;
    
    if (dump.cam_down) packet.cam_rec |= AV_CAMERA_AERO_BOT;
    if (dump.cam_up)   packet.cam_rec |= AV_CAMERA_AERO_TOP;
    if (dump.cam_sep)  packet.cam_rec |= AV_CAMERA_SEPMEC;
}
PREPARE_DOWNLINK(flight_computer::UplinkCmd) {
    (void) packet; (void) dump;
}

PREPARE_DOWNLINK(flight_computer::DataDump) {
    packet.av_state     = static_cast<uint8_t>(dump.av_state);
    packet.av_fc_temp   = dump.av_fc_temp;
    packet.av_timestamp = dump.av_timestamp;
    
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

class TxRadioModule {
private:
    SX127XCapsule *driver_;

    uint32_t packet_nbr = 0;
public:
    explicit TxRadioModule(SX127XCapsule *driver)
      : driver_(driver) {}

    bool init () {
	  driver_->init(864.34e6, SX127X_POWER_11DBM, SX127X_LORA_SF_8,
	  	SX127X_LORA_BW_125KHZ, SX127X_LORA_CR_4_7, SX127X_LORA_CRC_EN,
	  	av_downlink_size);
    }

    bool send (const flight_computer::DataDump &dump) {
        av_downlink_unpacked_t packet;
        packet.packet_nbr = packet_nbr ++;
        
        prepare_downlink_packet(packet, dump);

        av_downlink_t compressed_packet = encode_downlink(packet);

        return driver_->transmit(CAPSULE_ID::AV_TELEMETRY, (uint8_t*) &compressed_packet, av_downlink_size);
    }
};
