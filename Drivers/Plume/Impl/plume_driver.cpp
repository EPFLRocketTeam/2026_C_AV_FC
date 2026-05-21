
#include "plume_driver.hpp"
#include <stdio.h>
#include <string.h>

extern "C" {
    #include "plume/writer.h"
    #include "plume/status.h"
    #include "plume/const.h"
};

/* ── DMA completion flags (set from IRQ context) ─────────────────────────── */
volatile uint8_t g_sd_dma_complete = 1;   /* 1 = idle/done */
volatile uint8_t g_sd_dma_error    = 0;

extern "C" void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    (void)hsd;
    g_sd_dma_complete = 1;
}

extern "C" void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
    (void)hsd;
    g_sd_dma_error    = 1;
    g_sd_dma_complete = 1;       /* unblock the ready check */
}

#define DBG(...) printf(" - " #__VA_ARGS__ ": %u \r\n", __VA_ARGS__);
uint8_t plume_stm32_disk_information (SD_HandleTypeDef* hsd, struct plume_disk* disk_info) {
    if (hsd->State != HAL_SD_STATE_READY) {
        return -50;
    }

    disk_info->number_blocks = hsd->SdCard.LogBlockNbr;
    disk_info->block_size    = hsd->SdCard.LogBlockSize;
    printf("Information on disk: \r\n");
    printf(" - number blocks : %u\r\n", (uint32_t) disk_info->number_blocks);
    printf(" - block size    : %u\r\n", (uint32_t) disk_info->block_size);
    DBG(hsd->SdCard.BlockNbr);
    DBG(hsd->SdCard.BlockSize);
    DBG(hsd->SdCard.CardSpeed);
    DBG(hsd->SdCard.CardType);
    DBG(hsd->SdCard.CardVersion);
    DBG(hsd->SdCard.Class);
    DBG(hsd->SdCard.LogBlockNbr);
    DBG(hsd->SdCard.LogBlockSize);
    DBG(hsd->SdCard.RelCardAdd);

    return PLUME_OK;
}
uint8_t plume_stm32_read_block (SD_HandleTypeDef* hsd, struct plume_context* context, uint8_t* buffer, uint64_t block_id) {
    HAL_StatusTypeDef status = HAL_SD_ReadBlocks(hsd, buffer, (uint32_t) block_id, 1, HAL_MAX_DELAY);
    if (status == HAL_OK) {
        /* Invalidate D-cache so CPU sees data written by IDMA. */
        SCB_InvalidateDCache_by_Addr((uint32_t*)buffer, 512);
        return PLUME_OK;
    }

    return -45;
}
uint8_t plume_stm32_write_block (SD_HandleTypeDef* hsd, struct plume_context* context, const uint8_t* buffer, uint64_t block_id) {
    /* Wait for card to reach TRANSFER state (previous write programming done). */
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t0 > 500) {
            HAL_SD_Abort(hsd);
            return PLUME_OK_RETRY;
        }
    }

    /* Flush D-cache so IDMA reads committed data from AXI SRAM. */
    SCB_CleanDCache_by_Addr((uint32_t*)buffer, 512);

    g_sd_dma_complete = 0;
    g_sd_dma_error    = 0;

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(hsd, (uint8_t*)buffer, (uint32_t)block_id, 1);
    if (status != HAL_OK) {
        g_sd_dma_complete = 1;
        return PLUME_OK_RETRY;
    }
    return PLUME_OK_SENT_DMA;
}

uint8_t plume_stm32_write_blocks (SD_HandleTypeDef* hsd, struct plume_context* context, const uint8_t* buffer, uint64_t block_id, uint32_t num_blocks) {
    /* Wait for card to reach TRANSFER state. */
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
        if (HAL_GetTick() - t0 > 500) {
            HAL_SD_Abort(hsd);
            return PLUME_OK_RETRY;
        }
    }

    /* Flush D-cache for the entire batch so IDMA sees committed data. */
    SCB_CleanDCache_by_Addr((uint32_t*)buffer, num_blocks * 512);

    g_sd_dma_complete = 0;
    g_sd_dma_error    = 0;

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(hsd, (uint8_t*)buffer, (uint32_t)block_id, num_blocks);
    if (status != HAL_OK) {
        g_sd_dma_complete = 1;
        return PLUME_OK_RETRY;
    }
    return PLUME_OK_SENT_DMA;
}

uint8_t plume_stm32_write_block_ready (SD_HandleTypeDef* hsd, struct plume_context* context) {
    if (!g_sd_dma_complete) {
        return 0;          /* DMA transfer still in progress */
    }
    /* DMA finished — also wait for the card to finish programming. */
    if (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
        return 0;
    }
    return 1;
}




bool SDCardInterface::init_sd_card (
    SD_HandleTypeDef* hsd,
    uint8_t* arena_buffer,
    size_t   arena_length
) {
    if (hsd->State != HAL_SD_STATE_READY) {
        return PLUME_EBAD_DISK;
    }

    driver.driver_ptr = hsd;

    driver.disk_information = 
        PLUME_DISK_INFORMATION_FN_TYPE
        plume_stm32_disk_information;
    driver.read_block = 
        PLUME_READ_BLOCK_FN_TYPE
        plume_stm32_read_block;
    driver.write_block = 
        PLUME_WRITE_BLOCK_FN_TYPE
        plume_stm32_write_block;
    driver.write_block_ready = 
        PLUME_WRITE_BLOCK_READY_FN_TYPE
        plume_stm32_write_block_ready;
    driver.write_blocks =
        PLUME_WRITE_BLOCKS_FN_TYPE
        plume_stm32_write_blocks;

    context.arena_buffer = arena_buffer;
    context.arena_length = arena_length;

    uint8_t err_code = plume_init(&context, &driver);
    if (err_code == PLUME_EBAD_DISK) {
        /* PLUME_EBAD_DISK means block 0 doesn't have the Plume settings marker.
         * Only auto-format if block 0 looks genuinely blank (all 0x00 or 0xFF).
         * If block 0 has other data (corrupted Plume card or foreign FS), refuse
         * to format so we never accidentally overwrite recoverable flight data. */
        bool block0_blank = true;
        for (size_t i = 0; i < 512 && i < arena_length; ++i) {
            if (arena_buffer[i] != 0x00 && arena_buffer[i] != 0xFF) {
                block0_blank = false;
                break;
            }
        }
        if (!block0_blank) {
            printf("[SD] Block 0 has non-blank data (not 0x00/0xFF) — refusing auto-format.\r\n");
            printf("[SD] If this card needs reformatting, clear it manually first.\r\n");
            return false;
        }

        printf("[SD] Card not formatted (block 0 blank), performing quick format...\r\n");
        /* Quick format: write settings page (block 0) + clear FAT region.
         * This avoids the multi-hour full plume_clear_disk() on a 16GB card. */
        constexpr uint64_t fat_size = 64;

        /* Write block 0: settings page */
        for (size_t i = 0; i < arena_length && i < 512; ++i)
            arena_buffer[i] = 0x00;
        arena_buffer[0] = PLUME_PAGE_SETTINGS;
        memcpy(arena_buffer + 1, &fat_size, sizeof(uint64_t));
        uint8_t wr_err = plume_write_block_blocking(&context, arena_buffer, 0);
        if (!plume_is_ok(wr_err)) {
            printf("[SD] Quick format: failed to write settings block (%u)\r\n", wr_err);
            return false;
        }

        /* Clear FAT blocks (1..fat_size-1) so binary search finds them empty */
        for (size_t i = 0; i < 512; ++i)
            arena_buffer[i] = 0x00;
        for (uint64_t blk = 1; blk < fat_size; ++blk) {
            wr_err = plume_write_block_blocking(&context, arena_buffer, blk);
            if (!plume_is_ok(wr_err)) {
                printf("[SD] Quick format: failed at FAT block %u (%u)\r\n",
                       (unsigned)blk, wr_err);
                return false;
            }
        }
        printf("[SD] Quick format done (%u FAT blocks written)\r\n", (unsigned)fat_size);

        /* Wait for last DMA write to finish before re-init */
        {
            uint32_t t0 = HAL_GetTick();
            while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
                if (HAL_GetTick() - t0 > 1000) break;
            }
        }

        /* Retry init */
        err_code = plume_init(&context, &driver);
    }

    if (err_code != PLUME_OK) {
    	printf("Failure of init: %u\r\n", err_code);
    }

    return err_code == PLUME_OK;
}
bool SDCardInterface::open_file () {
    return plume_open_write(&context) == PLUME_OK;
}

size_t SDCardInterface::number_files_remaining () {
    return context.fat_size - context.next_file_block;
}
size_t SDCardInterface::disk_size_remaining () {
    return (context.disk_info.number_blocks - context.next_valid_block) * context.disk_info.block_size;
}

uint8_t SDCardInterface::write (const uint8_t* buffer, int length) {
    return plume_write(&context, buffer, length);
}
uint8_t SDCardInterface::tick () {
    return plume_tick(&context);
}
