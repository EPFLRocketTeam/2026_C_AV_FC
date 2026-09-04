
#include "Application/Config/config.hpp"
#include <cstdio>

struct FCDispatch {
private:
    prc_intranet::payload::config_chunk chunk;

    FlightParamsCANChannel channel;
public:
    void send (const FlightParams &params) {
        chunk.offset = channel.put(params, chunk.buffer, sizeof(chunk.buffer));
        config::internal::Fc_Can_SendChunkLog(chunk);
    }

    void commit (uint8_t board_id) {
        BoardIds true_id = static_cast<BoardIds>(board_id);
    
        config::internal::Fc_Can_SendCommit(true_id);
    }

    void print(uint8_t board, uint32_t crc_buffer, uint32_t crc_commited) {
        BoardIds true_id = static_cast<BoardIds>(board);

        switch (true_id) {
            case BoardIds::FP_ENGINE:
                printf("Config[Engine] : %u => %u\n", crc_buffer, crc_commited);
                break ;
            case BoardIds::FP_PRC_ETH:
                printf("Config[Eth] : %u => %u\n", crc_buffer, crc_commited);
                break ;
            case BoardIds::FP_PRC_LOX:
                printf("Config[Lox] : %u => %u\n", crc_buffer, crc_commited);
                break ;
        }
    }
};

namespace {
    FlightParamsDispatcher<FCDispatch> manager;
};

const FlightParams& config::get () {
    return manager.get_config();
}

FlightParams& config::internal::write () {
    return manager.write_buffer();
}

void config::internal::commit () {
    manager.commit();
}
void config::internal::tick () {
    manager.tick();
}

void config::internal::On_Crc (BoardIds ids, uint32_t crc_buffer, uint32_t crc_commited) {
    manager.onBoardUpdate(
        static_cast<uint8_t>(ids),
        crc_buffer,
        crc_commited
    );
}

void config::internal::print_status () {
    printf("Config[FC] : %u => %u\n", manager.get_buffer_crc(), manager.get_commited_crc());
    manager.print_status();
}
