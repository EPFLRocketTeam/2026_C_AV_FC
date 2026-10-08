
#include <meta>
#include <type_traits>

#include <iostream>
#include <map>
#include <filesystem>
#include <fstream>
#include "./entrypoint.hpp"

struct FileSystemStreams {
private:
    std::map<std::string, std::ofstream> streams;
    std::filesystem::path root;
public:
    FileSystemStreams () : root(".") {}
    FileSystemStreams (std::string folder) : root(folder) {}

    std::filesystem::path get_path (const std::string &buffer) {
        return root / buffer;
    }

    std::ofstream &get (const std::string &buffer) {
        auto it = streams.find(buffer);
        if (it == streams.end()) {
            auto path = get_path(buffer);

            std::filesystem::create_directories(path.parent_path());

            it = streams.try_emplace(buffer, path).first;
        }

        return it->second;
    }
};

int main (int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <logfile> [output_dir]\n";
        return 1;
    }

    std::string output_dir = argc == 2 ? std::string("./results") : std::string(argv[2]);

    FileSystemStreams streams(output_dir);
    CsvChannelContainer container([&](const std::string &buffer) -> std::ostream& {
        return streams.get(buffer);
    });

    std::ifstream input_stream(argv[1]);

    run_decode(container, input_stream);
}
