
#include "types.hpp"

#include <bits/stdc++.h>
using namespace std;


class SdLogger {
public:
    void logDataDump(flight_computer::DataDump data);
    void logCameraDump (const CameraDump& dump);
    void logFsmTransition(flight_computer::State prev, flight_computer::State next);
    void logImuRawBatch(size_t sensor_index, const Drivers::InvIMU::IMUData* samples, size_t count);
    void logBaroRaw(SdLogBaroSample sample);
    void logBootMarker(SdLogBootMarker marker);
    void logSdHealth(SdLogSdHealth health);
    void logAppMetrics(const SdLogAppMetrics& metrics);
    void logUbxRaw(const uint8_t* ubx_packet, uint16_t length);

    void logState(const eskf::StateSnapshot& snapshot);
    void logStateCritical(const eskf::StateSnapshot& snapshot);
    void logCovariance(const eskf::CovarianceSnapshot& snapshot);
    void logEvent(eskf::EskfEventType event, uint64_t timestamp_us, float value);
    void logGpsRejection(eskf::EskfEventType event, uint64_t timestamp_us,
                         const eskf::GpsRejectionInfo& info);
    void logRewind(eskf::EskfEventType event, uint64_t timestamp_us,
                   const eskf::RewindInfo& info);
    void logCorrection(eskf::EskfEventType event, uint64_t timestamp_us,
                       float innovation, float nis);
    void logRailShadow(const eskf::RailShadowSnapshot& snapshot);
    void logFlightShadow(const eskf::FlightShadowSnapshot& snapshot);
    void logImuPipeline(const eskf::ImuPipelineSnapshot& snapshot);
    void logImuDynamics(const eskf::ImuDynamicsSnapshot& snapshot);

    SdLogger () : outfile("log.bin") {}

    uint32_t time = 0;
private:
    void writeRecord(SdLogRecordType type, const void* payload, uint16_t payload_len);

    std::ofstream outfile;
};

// ============================================================
// Core write method — framed binary record to Plume
// ============================================================

void SdLogger::writeRecord(SdLogRecordType type, const void* payload, uint16_t payload_len) {
    SdLogHeader hdr;
    hdr.magic        = SD_LOG_MAGIC;
    hdr.record_type  = static_cast<uint8_t>(type);
    hdr.length       = payload_len;
    hdr.timestamp_us = (uint32_t)time ++;

    const uint32_t t0 = hdr.timestamp_us;

    outfile.write((char*) &hdr, sizeof(hdr));
    outfile.write((char*) payload, payload_len);
}

// ============================================================
// DataDump and FSM Logging
// ============================================================

void SdLogger::logDataDump(flight_computer::DataDump data) {
    writeRecord(SD_LOG_DATADUMP, &data, sizeof(data));
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
    SdLogGpsRejection record;
    record.event_type   = static_cast<uint8_t>(event);
    record.pad[0] = record.pad[1] = record.pad[2] = 0;
    record.info         = info;
    record.timestamp_us = timestamp_us;
    writeRecord(SD_LOG_GPS_REJECTION, &record, sizeof(record));
}

void SdLogger::logRewind(eskf::EskfEventType event, uint64_t timestamp_us,
                         const eskf::RewindInfo& info) {
    SdLogRewind record;
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
    hdr.timestamp_us = time ++;

    // Write header, batch header, then samples (3 separate writes to avoid
    // large stack copy — Plume concatenates them into the arena).
    outfile.write((char*) &hdr, sizeof(hdr));
    outfile.write((char*) &batch_hdr, sizeof(batch_hdr));
    outfile.write((char*) samples, count * sizeof(Drivers::InvIMU::IMUData));
}

void SdLogger::logBaroRaw(SdLogBaroSample record) {
    writeRecord(SD_LOG_BARO_RAW, &record, sizeof(record));
}

// ============================================================
// Boot Marker, Health, Metrics, UBX Raw
// ============================================================

void SdLogger::logBootMarker(SdLogBootMarker marker) {
    writeRecord(SD_LOG_BOOT_MARKER, &marker, sizeof(marker));
}

void SdLogger::logSdHealth(SdLogSdHealth health) {
    writeRecord(SD_LOG_SD_HEALTH, &health, sizeof(health));
}

void SdLogger::logAppMetrics(const SdLogAppMetrics& metrics) {
    writeRecord(SD_LOG_APP_METRICS, &metrics, sizeof(metrics));
}

void SdLogger::logUbxRaw(const uint8_t* ubx_packet, uint16_t length) {
    if (ubx_packet == nullptr || length == 0) return;
    writeRecord(SD_LOG_UBX_RAW, ubx_packet, length);
}

void SdLogger::logCameraDump (const CameraDump& dump) {
    writeRecord(SD_LOG_CAMERA, &dump, sizeof(dump));
}

using namespace flight_computer;

int main (void) {
    SdLogger logger;

    logger.logDataDump(DataDump{ .av_state = State::ASCENT });
    logger.logAppMetrics(SdLogAppMetrics{1});
    logger.logBaroRaw(SdLogBaroSample{2});
    logger.logBootMarker(SdLogBootMarker{3});
    logger.logCameraDump(CameraDump{ .aero_bot = { .cameraState = camera::ABORT_ON_START } });
    logger.logCorrection(eskf::EskfEventType::AeroBlindExited, 1, 1., 2.);
    logger.logCovariance(eskf::CovarianceSnapshot{ 6 });
    logger.logEvent(eskf::EskfEventType::BaroInnovationClamped, 12, 1.3);
    logger.logFlightShadow(eskf::FlightShadowSnapshot{ 7 });
    logger.logFsmTransition(State::BURN, State::ASCENT);
    logger.logGpsRejection(eskf::EskfEventType::BaroQueueFlushed, 13, eskf::GpsRejectionInfo{42});
    logger.logImuDynamics(eskf::ImuDynamicsSnapshot{.accel_body = {0.1, 0.2, 0.3}});
    logger.logImuPipeline(eskf::ImuPipelineSnapshot{.accel_body = {0.4, 0.2, 0.1}});
    //logger.logImuRawBatch

    logger.logRailShadow(eskf::RailShadowSnapshot{ .ground_pressure_pa = 15.2 });
    logger.logRewind(eskf::EskfEventType::FilterDiverged, 8, eskf::RewindInfo{ .baro_replayed = 4200 });
    logger.logSdHealth(SdLogSdHealth{ .arena_total_bytes = 515151515 });
    logger.logState(eskf::StateSnapshot{ .b_acc = { 1.2, 1.3, 2.3 } });
    logger.logStateCritical(eskf::StateSnapshot{ .b_acc = { 1.4, 1.3, 0.3 } });

    Drivers::InvIMU::IMUData samples[100];
    for (size_t off = 0; off < 100; off ++) {
        samples[off] = Drivers::InvIMU::IMUData{
            .accel_x = (float) off,
            .accel_y = (float) 2 * off,
            .accel_z = - (float) off,
            .gyro_x = 0,
            .gyro_y = 0,
            .gyro_z = 0,
            .temperature = 0,
            .timestamp_us = 12000
        };
    }
    logger.logImuRawBatch(2, samples, sizeof(samples) / sizeof(Drivers::InvIMU::IMUData));
    logger.logUbxRaw((uint8_t*) "hello, world !", 15);
}
