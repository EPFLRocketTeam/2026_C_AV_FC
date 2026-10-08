
#include <stdexcept>
#include <assert.h>
#include "./channel.hpp"
#include "./types.hpp"

using namespace flight_computer;

struct UbxRawChannel : public CsvChannel<uint8_t> {
    UbxRawChannel () = default;
    UbxRawChannel (
        std::function<std::ostream&(const std::string&)> get_st,
        std::string st_name
    ) : CsvChannel<uint8_t>(get_st, st_name) {}

    void custom_write_header () {
        init_stream();
    }
    void custom_aggregate (uint64_t timestamp_ms, SdLogHeader header, const void* payload) {
        init_stream();
        os->write((char*) &timestamp_ms, sizeof(uint64_t));
        os->write((char*) &header, sizeof(SdLogHeader));
        os->write((char*) payload, header.length);
    }
};
struct RawImuChannel : public CsvChannel<Drivers::InvIMU::IMUData> {
private:
    uint64_t batch_id = 0;
public:
    RawImuChannel () = default;
    RawImuChannel (
        std::function<std::ostream&(const std::string&)> get_st,
        std::string st_name
    ) : CsvChannel<Drivers::InvIMU::IMUData>(get_st, st_name) {}

    void custom_write_header () {
        init_stream();
        if (header_written) return ;
        *os << csv::header<uint64_t>{ .first = true, .field = "batch_id" }
            << csv::header<uint64_t>{ .first = false, .field = "ts_us" }
            << csv::header<SdLogImuBatchHeader>{ .first = false, .field = "header" }
            << csv::header<Drivers::InvIMU::IMUData>{ .first = false, .field = "" }
            << "\n";
    }

    void custom_aggregate (uint64_t timestamp_ms, SdLogHeader header, const void* payload) {
        init_stream();
        custom_write_header();

        SdLogImuBatchHeader batch_header = *((SdLogImuBatchHeader*) payload);

        uint64_t current_batch = batch_id ++;

        const Drivers::InvIMU::IMUData* arr = (Drivers::InvIMU::IMUData*)(payload + sizeof(batch_header));

        for (size_t offset = 0; offset < batch_header.sample_count; offset ++) {
            Drivers::InvIMU::IMUData data;
            std::memcpy(&data, &arr[offset], sizeof(Drivers::InvIMU::IMUData));

            *os << csv::value<uint64_t>{ .value = current_batch, .first = true }
                << csv::value<uint64_t>{ .value = timestamp_ms, .first = false }
                << csv::value<SdLogImuBatchHeader>{ .value = batch_header, .first = false }
                << csv::value<Drivers::InvIMU::IMUData>{ .value = data, .first = false }
                << "\n";
        }
    }
};

#define X_CHANNELS \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_DATADUMP,        DataDump,                   "fc/DataDump.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_FSM_TRANSITION,  SdLogFsmTransition,         "fc/FsmTransitions.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_ESKF_STATE,      eskf::StateSnapshot,        "fc/eskf/State.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_ESKF_COVARIANCE, eskf::CovarianceSnapshot,   "fc/eskf/Covariance.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_ESKF_EVENT,      SdLogEskfEvent,             "fc/eskf/Event.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_GPS_REJECTION,   SdLogGpsRejection,          "fc/eskf/GpsRejection.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_REWIND,          SdLogRewind,                "fc/eskf/Rewind.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_CORRECTION,      SdLogCorrection,            "fc/eskf/Correction.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_RAIL_SHADOW,     eskf::RailShadowSnapshot,   "fc/eskf/RailShadow.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_FLIGHT_SHADOW,   eskf::FlightShadowSnapshot, "fc/eskf/FlightShadow.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_IMU_PIPELINE,    eskf::ImuPipelineSnapshot,  "fc/eskf/ImuPipeline.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_IMU_DYNAMICS,    eskf::ImuDynamicsSnapshot,  "fc/eskf/ImuDynamics.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_BARO_RAW,        SdLogBaroSample,            "fc/sensors/BaroSample.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_BOOT_MARKER,     SdLogBootMarker,            "fc/BootMarker.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_SD_HEALTH,       SdLogSdHealth,              "fc/SdHealth.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_APP_METRICS,     SdLogAppMetrics,            "fc/AppMetrics.csv") \
    X_CHANNEL(SD_LOG_MAGIC, SD_LOG_CAMERA,          CameraDump,                 "fc/Cameras.csv") \
    X_CUSTOM_CHANNEL(SD_LOG_MAGIC, SD_LOG_IMU_RAW,  RawImuChannel,              "fc/sensors/IMUSample.csv") \
    X_CUSTOM_CHANNEL(SD_LOG_MAGIC, SD_LOG_UBX_RAW,  UbxRawChannel,              "fc/sensors/UBXRaw.csv")

#define CONCAT_IMPL(a, b) a##b
#define CONCAT(a, b) CONCAT_IMPL(a, b)

#define CHANNEL_VAR(RecordType) Channel_##RecordType

struct CsvChannelContainer {
private:
    #define X_CHANNEL(Magic, RecordType, Typename, FileName) \
        CsvChannel<Typename> CHANNEL_VAR(RecordType);
    #define X_CUSTOM_CHANNEL(Magic, RecordType, ClsChannel, FileName) \
        ClsChannel CHANNEL_VAR(RecordType);
    X_CHANNELS
    #undef X_CHANNEL
    #undef X_CUSTOM_CHANNEL

    bool has_init = false;
    uint8_t exp_magic = 0;
    bool init_with_magic (uint8_t magic) {
        if (has_init) return magic == exp_magic;
        exp_magic = magic;
        has_init = true;

        #define X_CHANNEL(Magic, RecordType, Typename, FileName) \
            if (Magic == magic) { CHANNEL_VAR(RecordType).write_header(); }
        #define X_CUSTOM_CHANNEL(Magic, RecordType, ClsChannel, FileName) \
            if (Magic == magic) { CHANNEL_VAR(RecordType).custom_write_header(); }
        X_CHANNELS
        #undef X_CHANNEL
        #undef X_CUSTOM_CHANNEL
        return true;
    }
public:
    CsvChannelContainer (
        std::function<std::ostream&(const std::string&)> get_channel_stream
    ) {
        #define X_CHANNEL(Magic, RecordType, Typename, FileName) \
            CHANNEL_VAR(RecordType) = CsvChannel<Typename>(get_channel_stream, FileName);
        #define X_CUSTOM_CHANNEL(Magic, RecordType, ClsChannel, FileName) \
            CHANNEL_VAR(RecordType) = ClsChannel(get_channel_stream, FileName);
        X_CHANNELS
        #undef X_CHANNEL
        #undef X_CUSTOM_CHANNEL
    }

    void ingest (SdLogHeader header, const void* payload) {
        if (!init_with_magic(header.magic)) {
            throw std::runtime_error("Invalid magic.");
        }

        bool found = false;
        
        #define X_CHANNEL(Magic, RecordType, Typename, FileName) \
            if (Magic == header.magic && RecordType == header.record_type) { \
                if (((int) sizeof(Typename)) != ((int) header.length)) { \
                    printf(#Typename " : local = %d, remote = %d\n", (int) sizeof(Typename), (int) header.length); \
                    assert(false); \
                } \
                CHANNEL_VAR(RecordType).aggregate(header.timestamp_us, *((const Typename*) payload)); \
                found = true; \
            }
        #define X_CUSTOM_CHANNEL(Magic, RecordType, ClsChannel, FileName) \
            if (Magic == header.magic && RecordType == header.record_type) { \
                CHANNEL_VAR(RecordType).custom_aggregate(header.timestamp_us, header, payload); \
                found = true; \
            }
        X_CHANNELS
        #undef X_CHANNEL
        #undef X_CUSTOM_CHANNEL

        if (!found) {
            throw std::runtime_error("Invalid record type.");
        }
    }

};

#undef CONCAT_IMPL
#undef CONCAT