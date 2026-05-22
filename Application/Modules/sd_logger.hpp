#pragma once
// SD Logger — Connects the ESKF logger interface + DataDump to Plume SD card.
// Binary framed records for offline parsing.

#include "Application/Kalman/kalman/eskf_logger.hpp"
#include "plume_driver.hpp"
#include "Application/Data/fsm.hpp"
#include "Drivers/InvIMU/InvIMU.h"
#include "Drivers/BMP390/BMP390.h"
#include <stdint.h>
#include "Application/Modules/sd_logger_types.hpp"

// ============================================================
// SdLogger: ESKF Logger backed by Plume SD Card
// ============================================================

class SdLogger : public eskf::IEskfLogger {
public:
    /// Initialize with a reference to the SDCardInterface.
    /// Call after Plume is initialized and file is opened.
    void init(SDCardInterface* sd) { sd_ = sd; }

    /// Write a framed DataDump record.
    void logDataDump(const void* data, uint16_t size);

    /// Write a framed FSM transition record.
    void logFsmTransition(flight_computer::State prev, flight_computer::State next);

    /// Write a batch of raw IMU samples (full-rate, called from ImuModule drain).
    void logImuRawBatch(size_t sensor_index, const Drivers::InvIMU::IMUData* samples, size_t count);

    /// Write a single raw baro sample (full-rate, called from BaroModule drain).
    void logBaroRaw(size_t sensor_index, const Drivers::BMP390::BaroData& sample);

    /// Log a boot marker to delimit sessions.
    void logBootMarker(uint8_t imu_count, uint8_t baro_count, bool gps_ok);

    /// Log SD card health metrics.
    void logSdHealth();

    /// Log application performance metrics.
    void logAppMetrics(const SdLogAppMetrics& metrics);

    /// Log a raw UBX GPS packet (before decoding).
    void logUbxRaw(const uint8_t* ubx_packet, uint16_t length);

    /// Notify that one SD tick (plume_tick) was executed.
    void notifyTick() { ++tick_count_; }

    // --- Health metric accessors (for debug prints) ---
    uint32_t bytesWritten()    const { return bytes_written_; }
    uint32_t writeCount()      const { return write_count_; }
    uint32_t writeFailCount()  const { return write_fail_count_; }
    uint32_t maxWriteTimeUs()  const { return max_write_time_us_; }
    uint32_t tickCount()       const { return tick_count_; }

    // --- IEskfLogger interface ---
    void logState(const eskf::StateSnapshot& snapshot) override;
    void logStateCritical(const eskf::StateSnapshot& snapshot) override;
    void logCovariance(const eskf::CovarianceSnapshot& snapshot) override;
    void logEvent(eskf::EskfEventType event, uint64_t timestamp_us, float value) override;
    void logGpsRejection(eskf::EskfEventType event, uint64_t timestamp_us,
                         const eskf::GpsRejectionInfo& info) override;
    void logRewind(eskf::EskfEventType event, uint64_t timestamp_us,
                   const eskf::RewindInfo& info) override;
    void logCorrection(eskf::EskfEventType event, uint64_t timestamp_us,
                       float innovation, float nis) override;
    void logRailShadow(const eskf::RailShadowSnapshot& snapshot) override;
    void logFlightShadow(const eskf::FlightShadowSnapshot& snapshot) override;
    void logImuPipeline(const eskf::ImuPipelineSnapshot& snapshot) override;
    void logImuDynamics(const eskf::ImuDynamicsSnapshot& snapshot) override;

private:
    SDCardInterface* sd_ = nullptr;

    // SD health tracking
    uint32_t bytes_written_ = 0;
    uint32_t write_count_ = 0;
    uint32_t write_fail_count_ = 0;
    uint32_t max_write_time_us_ = 0;
    uint32_t tick_count_ = 0;

    /// Write header + payload to Plume ring buffer.
    void writeRecord(SdLogRecordType type, const void* payload, uint16_t payload_len);
};
