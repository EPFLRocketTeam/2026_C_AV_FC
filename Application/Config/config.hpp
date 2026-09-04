
#pragma once

#include "FlightParams.hpp"
#include "Drivers/PRC_CAN/2026_C_AV_FC_PRC_INTRANET/include/prc_intranet/payload.hpp"

#include <stdint.h>
#include <stddef.h>

namespace config {
    const FlightParams& get ();

    namespace internal {
        FlightParams& write();
    
        void commit ();
        void tick ();
    
        void Fc_Can_SendChunkLog (prc_intranet::payload::config_chunk chunk);
        void Fc_Can_SendCommit (BoardIds id);

        void On_Crc (BoardIds ids, uint32_t crc_buffer, uint32_t crc_commited);

        void print_status ();
    };
};
