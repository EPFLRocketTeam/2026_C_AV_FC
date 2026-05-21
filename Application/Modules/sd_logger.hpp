#pragma once
// SD Logger — Connects the ESKF logger interface + DataDump to Plume SD card.
// Binary framed records for offline parsing.

#include "Application/Kalman/kalman/eskf_logger.hpp"
#include "plume_driver.hpp"
#include "Application/Data/fsm.hpp"
#include "Drivers/InvIMU/InvIMU.h"
#include "Drivers/BMP390/BMP390.h"
#include <stdint.h>

// ============================================================
// Binary Log Record Types
// ============================================================

enum SdLogRecordType : uint8_t {
    SD_LOG_DATADUMP           = 0x01,
    SD_LOG_ESKF_STATE         = 0x02,
    SD_LOG_ESKF_COVARIANCE    = 0x03,
    SD_LOG_ESKF_EVENT         = 0x04,
    SD_LOG_GPS_REJECTION      = 0x05,
    SD_LOG_REWIND             = 0x06,
    SD_LOG_RAIL_SHADOW        = 0x07,
    SD_LOG_FLIGHT_SHADOW      = 0x08,
    SD_LOG_IMU_PIPELINE       = 0x09,
    SD_LOG_IMU_DYNAMICS       = 0x0A,
    SD_LOG_FSM_TRANSITION     = 0x10,
    SD_LOG_CORRECTION         = 0x11,
    SD_LOG_IMU_RAW            = 0x20,  // Full-rate raw IMU batch
    SD_LOG_BARO_RAW           = 0x21,  // Full-rate raw baro sample
    SD_LOG_BOOT_MARKER        = 0x30,  // Session start indicator
    SD_LOG_SD_HEALTH          = 0x31,  // SD card write health metrics
    SD_LOG_APP_METRICS        = 0x32,  // Application performance metrics
    SD_LOG_UBX_RAW            = 0x33,  // Raw UBX GPS packet
};

// ============================================================
// Binary Log Header (prepended to every record)
// ============================================================

#pragma pack(push, 1)
struct SdLogHeader {
    uint8_t  magic;         // 0xAE
    uint8_t  record_type;   // SdLogRecordType
    uint16_t length;        // payload length (bytes after this header)
    uint32_t timestamp_ms;  // HAL_GetTick() at log time
};

struct SdLogFsmTransition {
    uint8_t prev_state;
    uint8_t new_state;
};

struct SdLogImuBatchHeader {
    uint8_t  sensor_index;   // 0-3
    uint8_t  sample_count;   // number of IMUData samples following
    uint16_t reserved;       // alignment padding
};

struct SdLogBaroSample {
    uint8_t  sensor_index;
    uint8_t  pad[3];
    float    pressure_pa;
    float    temperature_c;
    uint64_t timestamp_us;
};

struct SdLogEskfEvent {
    uint8_t event_type;  // EskfEventType
    uint8_t pad;
    float   value;
    uint64_t timestamp_us;
};

struct SdLogCorrection {
    uint8_t event_type;
    uint8_t pad[3];
    float innovation;
    float nis;
    uint64_t timestamp_us;
};

// Boot marker: logged once at initialization to delimit runs.
struct SdLogBootMarker {
    uint32_t firmware_crc;       // Placeholder for FW identification
    uint8_t  imu_count;          // Number of healthy IMUs (0-4)
    uint8_t  baro_count;         // Number of healthy baros (0-4)
    uint8_t  gps_ok;             // 1 if GPS init succeeded
    uint8_t  sd_ok;              // 1 if SD init succeeded
    uint32_t boot_time_ms;       // HAL_GetTick() at marker write
    uint32_t reset_reason;       // RCC reset flags (RCC->RSR)
};

// SD health metrics: logged periodically.
struct SdLogSdHealth {
    uint32_t bytes_written;      // Total bytes written this session
    uint32_t write_count;        // Number of writeRecord calls
    uint32_t write_fail_count;   // Number of failed writes (Plume full)
    uint32_t arena_used_bytes;   // Current arena ring buffer usage
    uint32_t arena_total_bytes;  // Arena capacity
    uint32_t max_write_time_us;  // Worst-case single write duration
    uint32_t tick_count;         // Number of SD tick calls
    uint32_t disk_remaining_kb;  // Remaining disk space (KB)
};

// Application performance metrics: logged periodically.
struct SdLogAppMetrics {
    uint32_t publish_ms;         // Timestamp (ms since boot)
    uint32_t loop_avg_us;        // Average main loop duration
    uint32_t loop_min_us;        // Min loop duration this interval
    uint32_t loop_max_us;        // Max loop duration this interval
    uint32_t loop_count;         // Loop iterations this interval
    uint32_t imu_batches;        // IMU batches processed (all sources)
    uint32_t imu_drops;          // IMU ring overflows
    uint32_t kalman_time_us;     // Last Kalman tick wall time
    uint32_t kalman_avg_us;      // Average Kalman tick time
    uint32_t kalman_max_us;      // Max Kalman tick time
    uint32_t sd_tick_time_us;    // Last SD tick wall time
    uint32_t sd_tick_max_us;     // Max SD tick time
    uint32_t catchup_events;     // ESKF catchup events processed
    uint32_t stale_skip_count;   // ESKF stale-skips
    uint32_t group_fire_count;   // IMU group fires
    uint32_t solo_flush_count;   // IMU solo flushes
};
#pragma pack(pop)

static constexpr uint8_t SD_LOG_MAGIC = 0xAE;

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
