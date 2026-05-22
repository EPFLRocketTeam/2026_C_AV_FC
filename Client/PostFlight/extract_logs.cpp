
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

    void aggregate (uint32_t timestamp_ms, const T& object) {
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
};

using namespace eskf;

#define X_CHANNELS \
    CHANNEL(SD_LOG_DATADUMP, DataDump, "DataDump.csv") \
    CHANNEL(SD_LOG_ESKF_STATE, StateSnapshot, "eskf_State.csv") \
    CHANNEL(SD_LOG_ESKF_COVARIANCE, CovarianceSnapshot, "eskf_Covariance.csv")

int main (int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <logfile> [output_dir]\n";
        return 1;
    }

    fs::path out_dir = (argc >= 3) ? argv[2] : "results";
    fs::create_directories(out_dir);

    const std::string& input_path = argv[1];

    std::ifstream file(input_path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open: " + input_path);

    #define CHANNEL(_Enum, Type, File) CSVChannel<Type> channel_##Type (out_dir / File);
    X_CHANNELS
    #undef CHANNEL
    
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
                    channel_##Type.aggregate (header.timestamp_ms, *reinterpret_cast<Type*>(payload.data()));
            X_CHANNELS
            #undef CHANNEL
            
            // TODO
            case SD_LOG_ESKF_EVENT:
            case SD_LOG_GPS_REJECTION:
            case SD_LOG_REWIND:
            case SD_LOG_RAIL_SHADOW:
            case SD_LOG_FLIGHT_SHADOW:
            case SD_LOG_IMU_PIPELINE:
            case SD_LOG_IMU_DYNAMICS:
            case SD_LOG_FSM_TRANSITION:
            case SD_LOG_CORRECTION:
            case SD_LOG_IMU_RAW:
            case SD_LOG_BARO_RAW:
            case SD_LOG_BOOT_MARKER:
            case SD_LOG_SD_HEALTH:
            case SD_LOG_APP_METRICS:
            case SD_LOG_UBX_RAW:
                break ;
            default:
                cerr << "Invalid uuid: " << header.record_type << "\n";
                return 0;
        }
    }
}
