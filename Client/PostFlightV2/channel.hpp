
#include "./header.hpp"
#include "./value.hpp"
#include <filesystem>
#include <functional>
#include <ostream>
#include <string>

template<typename T>
struct CsvChannel {
public:
    std::function<std::ostream&(const std::string&)> get_stream;
    std::string stream_name;

    std::ostream* os = nullptr;
    bool header_written = false;
    bool stream_init = false;
    
    void init_stream () {
        if (stream_init) return ;
        stream_init = true;

        os = &get_stream(stream_name);
    }
public:
    CsvChannel () = default;
    CsvChannel (
        std::function<std::ostream&(const std::string&)> get_st,
        std::string st_name
    ) : stream_name(st_name), get_stream(get_st) {}
    
    void write_header () {
        if (header_written) return ;
        header_written = true;

        init_stream();
        
        *os << csv::header<uint64_t>{ .first = true, .field = "ts_us" }
            << csv::header<T>{ .first = false, .field = "" }
            << "\n";
    }

    void aggregate (uint64_t timestamp_ms, const T &object) {
        init_stream();
        write_header();

        *os << csv::value<uint64_t>{ .value = timestamp_ms, .first = true }
            << csv::value<T>{ .value = object, .first = false }
            << "\n";
    }
};
