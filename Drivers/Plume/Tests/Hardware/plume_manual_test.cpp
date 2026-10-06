
extern "C" {
    #include "stm32hal.h"
    #include "plume_manual_test.h"
    #include "plume/status.h"
    #include <stdio.h>
}

#include "plume_driver.hpp"
#include "../../../../Application/app_printf.h"

const size_t plume_manual_test_arena_length = 64 * 1024;
uint8_t plume_manual_test_arena_buffer[plume_manual_test_arena_length] \
	__attribute__((aligned(32))) \
	__attribute__((section(".AXI_SRAM")));

void plume_manual_test (SD_HandleTypeDef *hsd) {
    SDCardInterface interface;
    
    app_printf("[PLUME] SD Card & Plume -- Manual Test\r\n");
    app_printf("[PLUME] Initializing SD Card...\r\n");

    if (!interface.init_sd_card(hsd, plume_manual_test_arena_buffer, plume_manual_test_arena_length)) {
        app_printf("[PLUME] Failure of init. Exiting...\r\n");
        return ;
    }

    app_printf("[PLUME] Remaining number of files: %u\r\n", interface.number_files_remaining());
    app_printf("[PLUME] Remaining disk size: %u\r\n", interface.disk_size_remaining());

    app_printf("[PLUME] Opening file...\r\n");
    if (!interface.open_file()) {
        app_printf("[PLUME] Failure of open. Exiting...\r\n");
        return ;
    }

    app_printf("[PLUME] Remaining number of files: %u\r\n", interface.number_files_remaining());
    app_printf("[PLUME] Remaining disk size: %u\r\n", interface.disk_size_remaining());

    app_printf("[PLUME] Writing 'Hello, World !\\n'\r\n");
    if (interface.write((const uint8_t*) "Hello, World !\n", 16) != PLUME_OK) {
        app_printf("[PLUME] Failure of write. Exiting\r\n");
        return ;
    }

    app_printf("[PLUME] Starting write of 256 kB.\r\n");
    uint32_t start_tick = HAL_GetTick();
    for (uint32_t i = 0; i < 256 * 256; i ++) {
        uint32_t j = ((i ^ 0b1101100110111000) << 16) | i;

        uint8_t status = interface.write((const uint8_t*) (&j), sizeof(uint32_t));
        if (status != PLUME_OK) {
            app_printf("[PLUME] Failure of write for %lu. Error code: %lu\r\n", i, (uint32_t) status);
            return ;
        }

        status = interface.tick();
        if (status != PLUME_OK) {
            app_printf("[PLUME] Failure of tick at %lu. Error code: %lu\r\n", i, (uint32_t) status);
            return ;
        }
    }

    uint32_t end_tick = HAL_GetTick();
    app_printf("[PLUME] Done in %lu ticks.\r\n", end_tick - start_tick);

    app_printf("[PLUME] Remaining number of files: %u\r\n", interface.number_files_remaining());
    app_printf("[PLUME] Remaining disk size: %u\r\n", interface.disk_size_remaining());
}
