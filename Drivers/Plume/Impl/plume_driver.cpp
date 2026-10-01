
#include "plume_driver.hpp"
#include <stdio.h>
#include <string.h>
#include "app_timebase.h"
#include "app_printf.h"
#define __STDC_FORMAT_MACROS
#include <cinttypes>
#include <cstdint>
#include <cstdio>

extern "C" {
    #include "plume/writer.h"
    #include "plume/status.h"
    #include "plume/const.h"
};

/* ── DMA bounce buffer ──────────────────────────────────────────────────── */
/* SDMMC1 IDMA can only access AXI SRAM (RAM_D1, 0x24000000).
 * The Plume arena lives in RAM_D2 (0x30000000) which IDMA cannot reach.
 * We memcpy into this bounce buffer before every DMA write.
 * Size = PLUME_MAX_BATCH_SIZE blocks × 512 B = 32 KB. */
__attribute__((section(".axi_sram.s_dma_bounce"), aligned(32)))
static uint8_t s_dma_bounce[PLUME_MAX_BATCH_SIZE * 512];

/* ── DMA completion flags (set from IRQ context) ─────────────────────────── */
volatile uint8_t g_sd_dma_complete = 1;   /* 1 = idle/done */
volatile uint8_t g_sd_dma_error    = 0;

/* ── SD timing instrumentation ──────────────────────────────────────────── */
static SdTimingStats s_sd_timing = {};

/* Public accessor for printing from main.cpp */
SdTimingStats sd_timing_snapshot() {
    SdTimingStats snap = s_sd_timing;
    /* Reset for next window */
    s_sd_timing.dma_count = 0;
    s_sd_timing.dma_error_count = 0;
    s_sd_timing.max_xfer_us = 0;
    s_sd_timing.max_prog_us = 0;
    s_sd_timing.max_cycle_us = 0;
    s_sd_timing.sum_cycle_us = 0;
    s_sd_timing.total_blocks = 0;
    s_sd_timing.min_batch = 0xFFFFFFFF;
    s_sd_timing.max_batch = 0;
    s_sd_timing.last_error_code = 0;
    return snap;
}

extern "C" void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    (void)hsd;
    s_sd_timing.dma_cb_us = app_timebase_now_us();
    g_sd_dma_complete = 1;
}

extern "C" void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
    s_sd_timing.dma_cb_us = app_timebase_now_us();
    s_sd_timing.dma_error_count++;
    s_sd_timing.last_error_code = hsd->ErrorCode;
    g_sd_dma_error    = 1;
    g_sd_dma_complete = 1;       /* unblock the ready check */
}

#define DBG(...) app_printf(" - " #__VA_ARGS__ ": %lu \r\n", (uint32_t) __VA_ARGS__);
uint8_t plume_stm32_disk_information (SD_HandleTypeDef* hsd, struct plume_disk* disk_info) {
    if (hsd->State != HAL_SD_STATE_READY) {
        return -50;
    }

    disk_info->number_blocks = hsd->SdCard.LogBlockNbr;
    disk_info->block_size    = hsd->SdCard.LogBlockSize;
    app_printf("Information on disk: \r\n");
    app_printf(" - number blocks : %lu\r\n", (uint32_t) disk_info->number_blocks);
    app_printf(" - block size    : %lu\r\n", (uint32_t) disk_info->block_size);
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
// uint8_t plume_stm32_read_block (SD_HandleTypeDef* hsd, struct plume_context* context, uint8_t* buffer, uint64_t block_id) {
//     /* HAL_SD_ReadBlocks() on STM32H7 reads SDMMC FIFO via CPU → data goes
//      * into D-cache naturally. No cache maintenance needed for the read path.
//      * (IDMA is only used by HAL_SD_ReadBlocks_DMA.) */
//     HAL_StatusTypeDef status = HAL_SD_ReadBlocks(hsd, buffer, (uint32_t) block_id, 1, HAL_MAX_DELAY);
//     if (status == HAL_OK) {
//         return PLUME_OK;
//     }
// 
//     return -45;
// }

uint8_t plume_stm32_read_block(SD_HandleTypeDef* hsd, struct plume_context* context, uint8_t* buffer, uint64_t block_id) {
    (void)context;
    uint32_t start_tick = HAL_GetTick();

    app_printf("[PLUME SD] [INFO] === Starting Blocking DMA Read Operation ===\r\n");
    app_printf("[PLUME SD] [DEBUG] Target Block ID : %llu (0x%llX)\r\n", block_id, block_id);
    app_printf("[PLUME SD] [DEBUG] Destination Buf : 0x%08lX | Bounce Buf: 0x%08lX\r\n", 
               (uint32_t)buffer, (uint32_t)s_dma_bounce);

    if (buffer == NULL) {
        app_printf("[PLUME SD] [ERR] Destination buffer pointer is NULL!\r\n");
        return -45;
    }

    /* 1. Ensure card is in TRANSFER state before issuing read */
    uint32_t t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - t0) > 500) {
            app_printf("[PLUME SD] [ERR] Pre-read card state check timed out!\r\n");
            HAL_SD_Abort(hsd);
            return -45;
        }
    }

    /* 2. Reset DMA flags & pre-clear static status register flags */
    g_sd_dma_complete = 0;
    g_sd_dma_error    = 0;
    __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);

    /* 3. Launch non-blocking IDMA read into AXI SRAM bounce buffer */
    HAL_StatusTypeDef status = HAL_SD_ReadBlocks_DMA(hsd, s_dma_bounce, (uint32_t)block_id, 1);
    if (status != HAL_OK) {
        app_printf("[PLUME SD] [ERR] HAL_SD_ReadBlocks_DMA launch failed! Status: %d\r\n", status);
        g_sd_dma_complete = 1;
        return -45;
    }

    /* 4. BLOCKING WAIT PHASE 1: Wait for HAL Driver & ISR to signal completion */
    uint32_t dma_wait_t0 = HAL_GetTick();
    while (hsd->State != HAL_SD_STATE_READY && g_sd_dma_complete == 0) {
        /* Timeout check (1000 ms) */
        if ((HAL_GetTick() - dma_wait_t0) > 1000) {
            
            /* SAFETY RECOVERY: Manually trigger IRQ handler if 25.6 kHz IMU EXTI preempted SDMMC1_IRQn */
            if (__HAL_SD_GET_FLAG(hsd, SDMMC_FLAG_DATAEND | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_RXOVERR)) {
                HAL_SD_IRQHandler(hsd);
                if (hsd->State == HAL_SD_STATE_READY) {
                    app_printf("[PLUME SD] [WARN] SDMMC1 ISR was preempted! Manually processed in polling loop.\r\n");
                    break;
                }
            }

            app_printf("[PLUME SD] [ERR] DMA Transfer Timeout! HAL State: %lu | SDMMC_STA: 0x%08lX\r\n", 
                       (uint32_t)hsd->State, hsd->Instance->STA);
            HAL_SD_Abort(hsd);
            __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);
            return -45;
        }
    }

    /* 5. Check if an interrupt error flag fired */
    if (g_sd_dma_error != 0 || hsd->ErrorCode != HAL_SD_ERROR_NONE) {
        app_printf("[PLUME SD] [ERR] DMA read failed during ISR! ErrorCode: 0x%08lX\r\n", hsd->ErrorCode);
        __HAL_SD_CLEAR_FLAG(hsd, SDMMC_STATIC_FLAGS);
        return -45;
    }

    /* 6. BLOCKING WAIT PHASE 2: Ensure physical SD card finishes bus transfer */
    uint32_t card_wait_t0 = HAL_GetTick();
    while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
        if ((HAL_GetTick() - card_wait_t0) > 500) {
            app_printf("[PLUME SD] [ERR] Card failed to return to TRANSFER state after DMA!\r\n");
            return -45;
        }
    }

    /* 7. Invalidate D-Cache for bounce buffer ONLY IF D-Cache is enabled in SCB->CCR */
    if (SCB->CCR & SCB_CCR_DC_Msk) {
        SCB_InvalidateDCache_by_Addr((uint32_t*)s_dma_bounce, 512);
    }

    /* 8. Copy validated data from AXI SRAM bounce buffer to caller destination buffer */
    memcpy(buffer, s_dma_bounce, 512);

    uint32_t total_elapsed = HAL_GetTick() - start_tick;
    app_printf("[PLUME SD] [INFO] Block %llu successfully read in %lu ms!\r\n", block_id, total_elapsed);

    /* 9. Data Header Preview */
    app_printf("[PLUME SD] [DEBUG] Data Header Preview (First 16 bytes):\r\n  └─ HEX: ");
    for (int i = 0; i < 16; i++) {
        app_printf("%02X ", buffer[i]);
    }
    app_printf("\r\n");

    return PLUME_OK;
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

    /* Copy arena data (RAM_D2) into AXI SRAM bounce buffer for IDMA. */
    memcpy(s_dma_bounce, buffer, 512);

    /* Flush D-cache so IDMA reads committed data from AXI SRAM. */
    // SCB_CleanDCache_by_Addr((uint32_t*)s_dma_bounce, 512);

    s_sd_timing.last_batch_size = 1;
    s_sd_timing.total_blocks += 1;
    if (1 < s_sd_timing.min_batch) s_sd_timing.min_batch = 1;
    if (1 > s_sd_timing.max_batch) s_sd_timing.max_batch = 1;

    g_sd_dma_complete = 0;
    g_sd_dma_error    = 0;
    s_sd_timing.dma_start_us = app_timebase_now_us();

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(hsd, s_dma_bounce, (uint32_t)block_id, 1);
    if (status != HAL_OK) {
        g_sd_dma_complete = 1;
        s_sd_timing.dma_start_us = 0;
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

    /* NOTE: CMD23 (SET_BLOCK_COUNT) removed intentionally.
     * The HAL uses open-ended CMD25 + CMD12 (STOP_TRANSMISSION) to end
     * multi-block writes.  Sending CMD23 before HAL_SD_WriteBlocks_DMA()
     * causes the card to auto-stop, so the HAL's subsequent CMD12 gets
     * CCRCFAIL/CTIMEOUT → false ErrorCallback on every single write. */

    /* Clamp to bounce buffer capacity. */
    if (num_blocks > PLUME_MAX_BATCH_SIZE) {
        num_blocks = PLUME_MAX_BATCH_SIZE;
    }

    /* Copy arena data (RAM_D2) into AXI SRAM bounce buffer for IDMA. */
    memcpy(s_dma_bounce, buffer, num_blocks * 512);

    /* Flush D-cache for the entire batch so IDMA sees committed data. */
    // SCB_CleanDCache_by_Addr((uint32_t*)s_dma_bounce, num_blocks * 512);

    /* ── Record batch size and DMA start timestamp ── */
    s_sd_timing.last_batch_size = num_blocks;
    s_sd_timing.total_blocks += num_blocks;
    if (num_blocks < s_sd_timing.min_batch) s_sd_timing.min_batch = num_blocks;
    if (num_blocks > s_sd_timing.max_batch) s_sd_timing.max_batch = num_blocks;

    g_sd_dma_complete = 0;
    g_sd_dma_error    = 0;
    s_sd_timing.dma_start_us = app_timebase_now_us();

    HAL_StatusTypeDef status = HAL_SD_WriteBlocks_DMA(hsd, s_dma_bounce, (uint32_t)block_id, num_blocks);
    if (status != HAL_OK) {
        g_sd_dma_complete = 1;
        s_sd_timing.dma_start_us = 0;   /* don't record broken DMA */
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

    /* ── Record timing stats ── */
    uint64_t now = app_timebase_now_us();
    if (s_sd_timing.dma_start_us > 0) {
        uint32_t xfer = (uint32_t)(s_sd_timing.dma_cb_us - s_sd_timing.dma_start_us);
        uint32_t prog = (uint32_t)(now - s_sd_timing.dma_cb_us);
        uint32_t cycle = (uint32_t)(now - s_sd_timing.dma_start_us);
        if (xfer > s_sd_timing.max_xfer_us) s_sd_timing.max_xfer_us = xfer;
        if (prog > s_sd_timing.max_prog_us) s_sd_timing.max_prog_us = prog;
        if (cycle > s_sd_timing.max_cycle_us) s_sd_timing.max_cycle_us = cycle;
        s_sd_timing.sum_cycle_us += cycle;
        s_sd_timing.dma_count++;
    }

    if (g_sd_dma_error) {
        g_sd_dma_error = 0;   /* consume the error — caller will see data not written */
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
            app_printf("[SD] Block 0 has non-blank data (not 0x00/0xFF) — refusing auto-format.\r\n");
            app_printf("[SD] If this card needs reformatting, clear it manually first.\r\n");
            return false;
        }

        app_printf("[SD] Card not formatted (block 0 blank), performing quick format...\r\n");
        /* Quick format: write settings page (block 0) + clear FAT region.
         * Use blocking (polling) HAL writes — no DMA complexity for one-time init. */
        constexpr uint64_t fat_size = 64;

        /* Write block 0: settings page */
        for (size_t i = 0; i < arena_length && i < 512; ++i)
            arena_buffer[i] = 0x00;
        arena_buffer[0] = PLUME_PAGE_SETTINGS;
        memcpy(arena_buffer + 1, &fat_size, sizeof(uint64_t));

        SCB_CleanDCache_by_Addr((uint32_t*)arena_buffer, 512);
        HAL_StatusTypeDef hal_rc = HAL_SD_WriteBlocks(hsd, arena_buffer, 0, 1, 1000);
        if (hal_rc != HAL_OK) {
            app_printf("[SD] Quick format: failed to write settings block (HAL=%d)\r\n", (int)hal_rc);
            return false;
        }
        /* Wait for card programming */
        while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {}

        /* Clear FAT blocks (1..fat_size-1) so binary search finds them empty */
        for (size_t i = 0; i < 512; ++i)
            arena_buffer[i] = 0x00;
        SCB_CleanDCache_by_Addr((uint32_t*)arena_buffer, 512);
        for (uint64_t blk = 1; blk < fat_size; ++blk) {
            hal_rc = HAL_SD_WriteBlocks(hsd, arena_buffer, (uint32_t)blk, 1, 1000);
            if (hal_rc != HAL_OK) {
                app_printf("[SD] Quick format: failed at FAT block %u (HAL=%d)\r\n",
                       (unsigned)blk, (int)hal_rc);
                return false;
            }
            while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {}
        }
        app_printf("[SD] Quick format done (%u FAT blocks written)\r\n", (unsigned)fat_size);

        /* Retry init */
        err_code = plume_init(&context, &driver);
    }

    if (err_code != PLUME_OK) {
    	app_printf("Failure of init: %u\r\n", err_code);
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

void SDCardInterface::beginTransaction () {
    if (inTransaction) {
        return ;
    }

    inTransaction = true;
    transactionFailed = false;

    snapshot = plume_save(&context);
}
void SDCardInterface::endTransaction () {
    if (!inTransaction) {
        return ;
    }

    inTransaction = false;
    lastTxFailed_ = transactionFailed;

    if (transactionFailed) {
        plume_rollback(&context, &snapshot);
    }
}

uint8_t SDCardInterface::write (const uint8_t* buffer, int length) {
    uint8_t worked = plume_write(&context, buffer, length);
    
    if (inTransaction && worked != PLUME_OK) {
        transactionFailed = true;
    }

    return worked;
}
uint8_t SDCardInterface::tick () {
    return plume_tick(&context);
}
