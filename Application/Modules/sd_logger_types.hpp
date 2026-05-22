
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
