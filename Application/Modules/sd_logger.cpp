#include "sd_logger.hpp"
#include <string.h>

extern "C" {
#include "stm32hal.h"
}

// ============================================================
// Core write method — framed binary record to Plume
// ============================================================

void SdLogger::writeRecord(SdLogRecordType type, const void* payload, uint16_t payload_len) {
    if (sd_ == nullptr) return;

    SdLogHeader hdr;
    hdr.magic        = SD_LOG_MAGIC;
    hdr.record_type  = static_cast<uint8_t>(type);
    hdr.length       = payload_len;
    hdr.timestamp_ms = HAL_GetTick();

    // Write header + payload as a single contiguous write.
    // Plume's ring buffer handles the byte-level copy.
    uint8_t buf[sizeof(SdLogHeader) + 1024];  // stack buffer for small records
    if (sizeof(SdLogHeader) + payload_len <= sizeof(buf)) {
        memcpy(buf, &hdr, sizeof(hdr));
        memcpy(buf + sizeof(hdr), payload, payload_len);
        sd_->write(buf, sizeof(hdr) + payload_len);
    } else {
        // Large record: write header then payload separately
        sd_->write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
        sd_->write(reinterpret_cast<const uint8_t*>(payload), payload_len);
    }
}

// ============================================================
// DataDump and FSM Logging
// ============================================================

void SdLogger::logDataDump(const void* data, uint16_t size) {
    writeRecord(SD_LOG_DATADUMP, data, size);
}

void SdLogger::logFsmTransition(flight_computer::State prev, flight_computer::State next) {
    SdLogFsmTransition evt;
    evt.prev_state = static_cast<uint8_t>(prev);
    evt.new_state  = static_cast<uint8_t>(next);
    writeRecord(SD_LOG_FSM_TRANSITION, &evt, sizeof(evt));
}

// ============================================================
// ESKF Logger Interface Implementation
// ============================================================

void SdLogger::logState(const eskf::StateSnapshot& snapshot) {
    writeRecord(SD_LOG_ESKF_STATE, &snapshot, sizeof(snapshot));
}

void SdLogger::logStateCritical(const eskf::StateSnapshot& snapshot) {
    writeRecord(SD_LOG_ESKF_STATE, &snapshot, sizeof(snapshot));
}

void SdLogger::logCovariance(const eskf::CovarianceSnapshot& snapshot) {
    writeRecord(SD_LOG_ESKF_COVARIANCE, &snapshot, sizeof(snapshot));
}

void SdLogger::logEvent(eskf::EskfEventType event, uint64_t timestamp_us, float value) {
    SdLogEskfEvent evt;
    evt.event_type   = static_cast<uint8_t>(event);
    evt.pad          = 0;
    evt.value        = value;
    evt.timestamp_us = timestamp_us;
    writeRecord(SD_LOG_ESKF_EVENT, &evt, sizeof(evt));
}

void SdLogger::logGpsRejection(eskf::EskfEventType event, uint64_t timestamp_us,
                               const eskf::GpsRejectionInfo& info) {
    // Pack event type + info together
    struct __attribute__((packed)) {
        uint8_t event_type;
        uint8_t pad[3];
        eskf::GpsRejectionInfo info;
        uint64_t timestamp_us;
    } record;
    record.event_type   = static_cast<uint8_t>(event);
    record.pad[0] = record.pad[1] = record.pad[2] = 0;
    record.info         = info;
    record.timestamp_us = timestamp_us;
    writeRecord(SD_LOG_GPS_REJECTION, &record, sizeof(record));
}

void SdLogger::logRewind(eskf::EskfEventType event, uint64_t timestamp_us,
                         const eskf::RewindInfo& info) {
    struct __attribute__((packed)) {
        uint8_t event_type;
        uint8_t pad[3];
        eskf::RewindInfo info;
        uint64_t timestamp_us;
    } record;
    record.event_type   = static_cast<uint8_t>(event);
    record.pad[0] = record.pad[1] = record.pad[2] = 0;
    record.info         = info;
    record.timestamp_us = timestamp_us;
    writeRecord(SD_LOG_REWIND, &record, sizeof(record));
}

void SdLogger::logCorrection(eskf::EskfEventType event, uint64_t timestamp_us,
                             float innovation, float nis) {
    SdLogCorrection record;
    record.event_type   = static_cast<uint8_t>(event);
    record.pad[0] = record.pad[1] = record.pad[2] = 0;
    record.innovation   = innovation;
    record.nis          = nis;
    record.timestamp_us = timestamp_us;
    writeRecord(SD_LOG_CORRECTION, &record, sizeof(record));
}

void SdLogger::logRailShadow(const eskf::RailShadowSnapshot& snapshot) {
    writeRecord(SD_LOG_RAIL_SHADOW, &snapshot, sizeof(snapshot));
}

void SdLogger::logFlightShadow(const eskf::FlightShadowSnapshot& snapshot) {
    writeRecord(SD_LOG_FLIGHT_SHADOW, &snapshot, sizeof(snapshot));
}

void SdLogger::logImuPipeline(const eskf::ImuPipelineSnapshot& snapshot) {
    writeRecord(SD_LOG_IMU_PIPELINE, &snapshot, sizeof(snapshot));
}

void SdLogger::logImuDynamics(const eskf::ImuDynamicsSnapshot& snapshot) {
    writeRecord(SD_LOG_IMU_DYNAMICS, &snapshot, sizeof(snapshot));
}

// ============================================================
// Full-Rate Raw Sensor Logging
// ============================================================

void SdLogger::logImuRawBatch(size_t sensor_index, const Drivers::InvIMU::IMUData* samples, size_t count) {
    if (sd_ == nullptr || count == 0) return;

    SdLogImuBatchHeader batch_hdr;
    batch_hdr.sensor_index = static_cast<uint8_t>(sensor_index);
    batch_hdr.sample_count = static_cast<uint8_t>(count);
    batch_hdr.reserved     = 0;

    const uint16_t payload_len = static_cast<uint16_t>(
        sizeof(batch_hdr) + count * sizeof(Drivers::InvIMU::IMUData));

    SdLogHeader hdr;
    hdr.magic        = SD_LOG_MAGIC;
    hdr.record_type  = static_cast<uint8_t>(SD_LOG_IMU_RAW);
    hdr.length       = payload_len;
    hdr.timestamp_ms = HAL_GetTick();

    // Write header, batch header, then samples (3 separate writes to avoid
    // large stack copy — Plume concatenates them into the arena).
    sd_->write(reinterpret_cast<const uint8_t*>(&hdr), sizeof(hdr));
    sd_->write(reinterpret_cast<const uint8_t*>(&batch_hdr), sizeof(batch_hdr));
    sd_->write(reinterpret_cast<const uint8_t*>(samples), count * sizeof(Drivers::InvIMU::IMUData));
}

void SdLogger::logBaroRaw(size_t sensor_index, const Drivers::BMP390::BaroData& sample) {
    if (sd_ == nullptr) return;

    SdLogBaroSample record;
    record.sensor_index = static_cast<uint8_t>(sensor_index);
    record.pad[0] = record.pad[1] = record.pad[2] = 0;
    record.pressure_pa   = sample.pressure_pa;
    record.temperature_c = sample.temperature_c;
    record.timestamp_us  = sample.timestamp_us;

    writeRecord(SD_LOG_BARO_RAW, &record, sizeof(record));
}
