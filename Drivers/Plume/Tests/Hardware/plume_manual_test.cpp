
extern "C" {
    #include "stm32hal.h"
    #include "plume_manual_test.h"
    #include "plume/status.h"
}

#include "plume_driver.hpp"

void plume_manual_test (SD_HandleTypeDef *hsd) {
    SDCardInterface interface;

    const size_t arena_length = 64 * 1024;
    uint8_t arena_buffer[arena_length];
    
    printf("SD Card & Plume -- Manual Test\n");
    printf("Initializing SD Card...\n");

    if (!interface.init_sd_card(hsd, arena_buffer, arena_length)) {
        printf("Failure of init.\n");
        return ;
    }

    printf("Opening file...\n");
    if (!interface.open_file()) {
        printf("Failure of open.\n");
        return ;
    }

    printf("Writing 'Hello, World !\\n'\n");
    if (interface.write((const uint8_t*) "Hello, World !\n", 16) != PLUME_OK) {
        printf("Failure of write.\n");
        return ;
    }

    printf("Starting write of 256 kB.\n");
    uint32_t start_tick = HAL_GetTick();
    for (uint32_t i = 0; i < 256 * 256; i ++) {
        uint32_t j = ((i ^ 0b1101100110111000) << 16) | i;

        uint8_t status = interface.write((const uint8_t*) (&j), sizeof(uint32_t));
        if (status != PLUME_OK) {
            printf("Failure of write for %u. Error code: %u\n", i, (uint32_t) status);
            return ;
        }

        status = interface.tick();
        if (status != PLUME_OK) {
            printf("Failure of tick at %u. Error code: %u\n", i, (uint32_t) status);
            return ;
        }
    }
    uint32_t end_tick = HAL_GetTick();
    printf("Done in %u ticks.", end_tick - start_tick);
}
