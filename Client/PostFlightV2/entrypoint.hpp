
#include <iostream>
#include "./container.hpp"

const size_t MAX_SZE = 1 << 20;

uint64_t base_buffer[MAX_SZE / 8];

void run_decode (
    CsvChannelContainer &container,
    std::istream& stream
) {
    uint8_t* buffer = (uint8_t*) base_buffer;
    
    while (1) {
        SdLogHeader header;

        stream.read((char*) &header, sizeof(header));
        if (stream.fail()) {
            break ;
        }

        if (header.length > MAX_SZE) {
            throw std::runtime_error("Invalid header length (length = " + std::to_string(header.length) + " > 1024)");
        }

        stream.read(buffer, header.length);

        container.ingest(header, (const void*) buffer);
    }
}
