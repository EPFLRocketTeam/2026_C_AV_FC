
#include "plume_driver.hpp"
#include <stdio.h>

extern "C" {
    #include "plume/writer.h"
    #include "plume/status.h"
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
