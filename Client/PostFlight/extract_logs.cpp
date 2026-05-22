
#include "csv_rules.hpp"
#include <cstdint>
#include <filesystem>
#define uint32_t unsigned long

namespace fs = std::filesystem;

template<typename T>
struct CSVChannel {
    std::ofstream file;
    bool header_written = false;

    CSVChannel () = default;
    CSVChannel (const fs::path& path) : file(path) {
        if (!file) throw std::runtime_error("Cannot open: " + path.string());
    }

    void aggregate_ms (uint32_t timestamp_ms, const T& object) {
        if (!header_written) {
            std::stringstream stream;
            generate_csv<uint32_t>(stream, "ts_ms", timestamp_ms, true, true);
            generate_csv<T>(stream, "", object, true, false);
            file << stream.str() << "\n";
            header_written = true;
        }

        std::stringstream stream;
        generate_csv<uint32_t>(stream, "ts_ms", timestamp_ms, false, true);
        generate_csv<T>(stream, "", object, false, false);
        file << stream.str() << "\n";
    }
    void aggregate_us (uint64_t timestamp_us, const T& object) {
        if (!header_written) {
            std::stringstream stream;
            generate_csv<uint64_t>(stream, "ts_us", timestamp_us, true, true);
            generate_csv<T>(stream, "", object, true, false);
            file << stream.str() << "\n";
            header_written = true;
        }

        std::stringstream stream;
        generate_csv<uint64_t>(stream, "ts_us", timestamp_us, false, true);
        generate_csv<T>(stream, "", object, false, false);
        file << stream.str() << "\n";
    }
};

using namespace eskf;

#define X_CHANNELS \
    CHANNEL(SD_LOG_DATADUMP,       DataDump,           "DataDump.csv") \
    CHANNEL(SD_LOG_APP_METRICS,    SdLogAppMetrics,    "AppMetrics.csv") \
    CHANNEL(SD_LOG_SD_HEALTH,      SdLogSdHealth,      "SdHealth.csv") \
    CHANNEL(SD_LOG_BOOT_MARKER,    SdLogBootMarker,    "BootMarker.csv") \
    CHANNEL(SD_LOG_FSM_TRANSITION, SdLogFsmTransition, "FsmTransition.csv") \
    \
    CHANNEL(SD_LOG_ESKF_STATE,      StateSnapshot,        "eskf/State.csv") \
    CHANNEL(SD_LOG_ESKF_COVARIANCE, CovarianceSnapshot,   "eskf/Covariance.csv") \
    CHANNEL(SD_LOG_ESKF_EVENT,      SdLogEskfEvent,       "eskf/Event.csv") \
    CHANNEL(SD_LOG_GPS_REJECTION,   SdLogGpsRejection,    "eskf/GpsRejection.csv") \
    CHANNEL(SD_LOG_REWIND,          SdLogRewind,          "eskf/Rewind.csv") \
    CHANNEL(SD_LOG_RAIL_SHADOW,     RailShadowSnapshot,   "eskf/RailShadow.csv") \
    CHANNEL(SD_LOG_CORRECTION,      SdLogCorrection,      "eskf/Correction.csv") \
    CHANNEL(SD_LOG_FLIGHT_SHADOW,   FlightShadowSnapshot, "eskf/FlightShadow.csv") \
    CHANNEL(SD_LOG_IMU_PIPELINE,    ImuPipelineSnapshot,  "eskf/ImuPipeline.csv") \
    CHANNEL(SD_LOG_IMU_DYNAMICS,    ImuDynamicsSnapshot,  "eskf/ImuDynamics.csv") \
    \
    CHANNEL(SD_LOG_BARO_RAW, SdLogBaroSample, "sensors/BaroSample.csv")
#define IMU_RAW_DATA_FILE                     "sensors/IMUSample.csv"

int main (int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <logfile> [output_dir]\n";
        return 1;
    }

    fs::path out_dir = (argc >= 3) ? argv[2] : "results";
    fs::create_directories(out_dir);
    fs::create_directories(out_dir / "eskf");
    fs::create_directories(out_dir / "sensors");

    const std::string& input_path = argv[1];

    std::ifstream file(input_path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open: " + input_path);

    #define CHANNEL(_Enum, Type, File) CSVChannel<Type> channel_##Type (out_dir / File);
    X_CHANNELS
    #undef CHANNEL

    CSVChannel<IMUDataAndSensor> channel_IMUDataAndSensor (out_dir / IMU_RAW_DATA_FILE);
    
    SdLogHeader header;
    while (file.read(reinterpret_cast<char*>(&header), sizeof(header))) {
        std::vector<std::byte> payload(header.length);
        if (!file.read(reinterpret_cast<char*>(payload.data()), header.length)) {
            std::cerr << "Warning: truncated payload at ts=" << header.timestamp_ms << "\n";
            break;
        }

        if (header.magic != 0xAE) {
            std::cerr << "Warning: Invalid magic" << "\n";
            break ;
        }

        switch (header.record_type) {
            #define CHANNEL(Enum, Type, _File) \
                case Enum: \
                    channel_##Type.aggregate_ms (header.timestamp_ms, *reinterpret_cast<Type*>(payload.data())); \
                    break ;
            X_CHANNELS
            #undef CHANNEL
            
            case SD_LOG_IMU_RAW: {
                SdLogImuBatchHeader batchHeader = *reinterpret_cast<SdLogImuBatchHeader*>(payload.data());
                
                Drivers::InvIMU::IMUData *batch = reinterpret_cast<Drivers::InvIMU::IMUData*>(payload.data() + sizeof(SdLogImuBatchHeader));

                for (size_t offset = 0; offset < batchHeader.sample_count; offset ++) {
                    IMUDataAndSensor data;
                    data.sensor_index = batchHeader.sensor_index;
                    data.sensor_data  = batch[offset];

                    channel_IMUDataAndSensor.aggregate_us(batch[offset].timestamp_us, data);
                }
                
                break ;
            }
            case SD_LOG_UBX_RAW:
                break ;
            default:
                cerr << "Invalid uuid: " << header.record_type << "\n";
                return 0;
        }
    }
}
