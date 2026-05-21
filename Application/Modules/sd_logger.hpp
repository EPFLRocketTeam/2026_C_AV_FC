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

    /// Write header + payload to Plume ring buffer.
    void writeRecord(SdLogRecordType type, const void* payload, uint16_t payload_len);
};
