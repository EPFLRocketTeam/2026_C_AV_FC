extern "C" {
#include "Drivers/STM32HAL/stm32hal.h"
#include "Application/Tests/sd_benchmark.h"
#include <stdio.h>
#include <string.h>
}

// ---------------------------------------------------------------------------
// SD Card Benchmark — measures raw HAL_SD_WriteBlocks latency
// Reports throughput, p50, p95, p99, p99.9, max for single-block
// and multi-block (8-block) sequential writes.
// ---------------------------------------------------------------------------

static constexpr uint32_t kSingleBlockCount = 4096;   // 2 MB
static constexpr uint32_t kMultiBlockCount  = 512;     // 512 × 8 = 2 MB
static constexpr uint32_t kMultiBlockSize   = 8;       // blocks per write
static constexpr uint32_t kStartBlock       = 100000;  // safe offset past Plume metadata

// Latency storage (µs per write). Reused between phases.
static uint32_t g_latencies[kSingleBlockCount];

// Write buffers — aligned for SDMMC hardware
static uint8_t g_buf_512[512]    __attribute__((aligned(32)));
static uint8_t g_buf_4k[4096]    __attribute__((aligned(32)));
static uint8_t g_buf_8k[8192]    __attribute__((aligned(32)));
static uint8_t g_buf_16k[16384]  __attribute__((aligned(32)));

// DMA completion flag
static volatile bool g_dma_tx_complete = false;
static volatile bool g_dma_tx_error = false;

// ---------------------------------------------------------------------------
// Simple shell sort for uint32_t array (fast enough for 4096 elements)
// ---------------------------------------------------------------------------
static void sort_u32(uint32_t* arr, uint32_t n) {
    for (uint32_t gap = n / 2; gap > 0; gap /= 2) {
        for (uint32_t i = gap; i < n; i++) {
            uint32_t tmp = arr[i];
            uint32_t j = i;
            while (j >= gap && arr[j - gap] > tmp) {
                arr[j] = arr[j - gap];
                j -= gap;
            }
            arr[j] = tmp;
        }
    }
}

// ---------------------------------------------------------------------------
// DWT cycle counter helpers
// ---------------------------------------------------------------------------
static inline void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t dwt_cycles(void) {
    return DWT->CYCCNT;
}

static uint32_t cycles_to_us(uint32_t cycles) {
    // SystemCoreClock = 480 MHz → 480 cycles/µs
    return cycles / (SystemCoreClock / 1000000u);
}

// ---------------------------------------------------------------------------
// Report percentiles from a sorted latency array
// ---------------------------------------------------------------------------
static void report_percentiles(const char* label, const uint32_t* sorted, uint32_t n) {
    uint64_t sum = 0;
    for (uint32_t i = 0; i < n; i++) sum += sorted[i];

    uint32_t p50  = sorted[n * 50 / 100];
    uint32_t p95  = sorted[n * 95 / 100];
    uint32_t p99  = sorted[n * 99 / 100];
    uint32_t p999 = sorted[n * 999 / 1000];
    uint32_t pmin = sorted[0];
    uint32_t pmax = sorted[n - 1];
    uint32_t avg  = (uint32_t)(sum / n);

    printf("[SD-BENCH] %s (n=%lu):\r\n", label, (unsigned long)n);
    printf("  min=%lu  avg=%lu  p50=%lu  p95=%lu  p99=%lu  p99.9=%lu  max=%lu  (us)\r\n",
           (unsigned long)pmin, (unsigned long)avg,
           (unsigned long)p50, (unsigned long)p95,
           (unsigned long)p99, (unsigned long)p999,
           (unsigned long)pmax);

    // Data volume and throughput
    uint32_t total_ms = (uint32_t)(sum / 1000u);
    if (total_ms == 0) total_ms = 1;
    // For multi-block: each write is kMultiBlockSize * 512 bytes
    // Caller prints throughput separately since they know the block size
}

// ---------------------------------------------------------------------------
// Wait for card to reach transfer state (with timeout)
// ---------------------------------------------------------------------------
static bool wait_card_ready(SD_HandleTypeDef* hsd, uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        HAL_SD_CardStateTypeDef st = HAL_SD_GetCardState(hsd);
        if (st == HAL_SD_CARD_TRANSFER) return true;
        // Small busy-wait
    }
    return false;
}

// ---------------------------------------------------------------------------
// Fill buffer with deterministic pattern
// ---------------------------------------------------------------------------
static void fill_pattern(uint8_t* buf, uint32_t len, uint32_t seed) {
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t v = seed ^ (i * 0x9E3779B9u);
        uint32_t rem = len - i;
        uint32_t to_copy = rem < 4 ? rem : 4;
        memcpy(buf + i, &v, to_copy);
    }
}

// ---------------------------------------------------------------------------
// Phase 1: Single-block (512B) sequential writes
// ---------------------------------------------------------------------------
static void bench_single_block(SD_HandleTypeDef* hsd) {
    printf("[SD-BENCH] Phase 1: %lu single-block (512B) sequential writes...\r\n",
           (unsigned long)kSingleBlockCount);

    fill_pattern(g_buf_512, 512, 0xDEADBEEF);

    uint32_t total_start = dwt_cycles();
    uint32_t errors = 0;

    for (uint32_t i = 0; i < kSingleBlockCount; i++) {
        // Wait for card to be in transfer state
        if (!wait_card_ready(hsd, 1000)) {
            // Try abort and re-wait
            HAL_SD_Abort(hsd);
            if (!wait_card_ready(hsd, 1000)) {
                errors++;
                g_latencies[i] = 0;
                continue;
            }
        }
        uint32_t t0 = dwt_cycles();
        HAL_StatusTypeDef st = HAL_SD_WriteBlocks(hsd, g_buf_512, kStartBlock + i, 1, HAL_MAX_DELAY);
        uint32_t t1 = dwt_cycles();

        if (st != HAL_OK) {
            errors++;
            g_latencies[i] = 0;
            if (errors <= 5) {
                printf("[SD-BENCH]   Write[%lu] err: HAL=%d state=%u err=0x%lX\r\n",
                       (unsigned long)i, (int)st,
                       (unsigned)hsd->State, (unsigned long)hsd->ErrorCode);
            }
        } else {
            g_latencies[i] = cycles_to_us(t1 - t0);
        }
    }

    uint32_t total_end = dwt_cycles();
    uint32_t total_us = cycles_to_us(total_end - total_start);

    if (errors > 0) {
        printf("[SD-BENCH]   ERRORS: %lu / %lu writes failed!\r\n",
               (unsigned long)errors, (unsigned long)kSingleBlockCount);
    }

    // Throughput: 2MB in total_us
    uint32_t total_bytes = kSingleBlockCount * 512u;
    uint32_t throughput_kBs = 0;
    if (total_us > 0) {
        throughput_kBs = (uint32_t)((uint64_t)total_bytes * 1000000ull / total_us / 1024ull);
    }
    printf("[SD-BENCH]   Total: %lu us for %lu KB => %lu KB/s\r\n",
           (unsigned long)total_us,
           (unsigned long)(total_bytes / 1024),
           (unsigned long)throughput_kBs);

    sort_u32(g_latencies, kSingleBlockCount);
    report_percentiles("Single-block 512B", g_latencies, kSingleBlockCount);
}

// ---------------------------------------------------------------------------
// Generic multi-block sequential writes
// ---------------------------------------------------------------------------
static void bench_multi_block_generic(SD_HandleTypeDef* hsd, uint8_t* buf,
                                       uint32_t blocks_per_write, uint32_t num_writes,
                                       uint32_t base_block, const char* label) {
    uint32_t buf_size = blocks_per_write * 512u;
    printf("[SD-BENCH] %s: %lu × %lu-block (%lu KB) sequential writes...\r\n",
           label, (unsigned long)num_writes,
           (unsigned long)blocks_per_write,
           (unsigned long)(buf_size / 1024));

    fill_pattern(buf, buf_size, 0xCAFEBABE ^ blocks_per_write);

    uint32_t total_start = dwt_cycles();
    uint32_t errors = 0;

    for (uint32_t i = 0; i < num_writes; i++) {
        if (!wait_card_ready(hsd, 1000)) {
            HAL_SD_Abort(hsd);
            if (!wait_card_ready(hsd, 1000)) {
                errors++;
                g_latencies[i] = 0;
                continue;
            }
        }
        uint32_t t0 = dwt_cycles();
        HAL_StatusTypeDef st = HAL_SD_WriteBlocks(hsd, buf,
                                                   base_block + i * blocks_per_write,
                                                   blocks_per_write, HAL_MAX_DELAY);
        uint32_t t1 = dwt_cycles();

        if (st != HAL_OK) {
            errors++;
            g_latencies[i] = 0;
            HAL_SD_Abort(hsd);
        } else {
            g_latencies[i] = cycles_to_us(t1 - t0);
        }
    }

    uint32_t total_end = dwt_cycles();
    uint32_t total_us = cycles_to_us(total_end - total_start);

    if (errors > 0) {
        printf("[SD-BENCH]   ERRORS: %lu / %lu writes failed!\r\n",
               (unsigned long)errors, (unsigned long)num_writes);
    }

    uint32_t total_bytes = num_writes * blocks_per_write * 512u;
    uint32_t throughput_kBs = 0;
    if (total_us > 0) {
        throughput_kBs = (uint32_t)((uint64_t)total_bytes * 1000000ull / total_us / 1024ull);
    }
    printf("[SD-BENCH]   Total: %lu us for %lu KB => %lu KB/s\r\n",
           (unsigned long)total_us,
           (unsigned long)(total_bytes / 1024),
           (unsigned long)throughput_kBs);

    // Build label for percentiles
    char plabel[64];
    snprintf(plabel, sizeof(plabel), "%s (%luKB)", label, (unsigned long)(buf_size / 1024));
    sort_u32(g_latencies, num_writes);
    report_percentiles(plabel, g_latencies, num_writes);
}

// ---------------------------------------------------------------------------
// Phase 3: Sustained write test (10s of continuous single-block writes)
// Measures behavior over time to catch GC stalls
// ---------------------------------------------------------------------------
static void bench_sustained(SD_HandleTypeDef* hsd) {
    printf("[SD-BENCH] Phase 3: Sustained single-block writes for ~10 seconds...\r\n");

    fill_pattern(g_buf_512, 512, 0xB00B1E55);

    // Use blocks far from previous phases
    uint32_t base_block = kStartBlock + 200000;
    uint32_t count = 0;
    const uint32_t max_count = kSingleBlockCount; // reuse latency array size

    uint32_t total_start = dwt_cycles();
    uint32_t errors = 0;
    uint32_t deadline_ms = HAL_GetTick() + 10000;  // 10 seconds

    while (HAL_GetTick() < deadline_ms && count < max_count) {
        if (!wait_card_ready(hsd, 500)) {
            HAL_SD_Abort(hsd);
            if (!wait_card_ready(hsd, 500)) {
                errors++;
                g_latencies[count] = 0;
                count++;
                continue;
            }
        }
        uint32_t t0 = dwt_cycles();
        HAL_StatusTypeDef st = HAL_SD_WriteBlocks(hsd, g_buf_512,
                                                   base_block + count, 1, HAL_MAX_DELAY);
        uint32_t t1 = dwt_cycles();

        if (st != HAL_OK) {
            errors++;
            g_latencies[count] = 0;
        } else {
            g_latencies[count] = cycles_to_us(t1 - t0);
        }
        count++;
    }

    uint32_t total_end = dwt_cycles();
    uint32_t total_us = cycles_to_us(total_end - total_start);

    printf("[SD-BENCH]   Wrote %lu blocks in %lu us\r\n",
           (unsigned long)count, (unsigned long)total_us);

    if (errors > 0) {
        printf("[SD-BENCH]   ERRORS: %lu / %lu writes failed!\r\n",
               (unsigned long)errors, (unsigned long)count);
    }

    uint32_t total_bytes = count * 512u;
    uint32_t throughput_kBs = 0;
    if (total_us > 0) {
        throughput_kBs = (uint32_t)((uint64_t)total_bytes * 1000000ull / total_us / 1024ull);
    }
    printf("[SD-BENCH]   Throughput: %lu KB/s (%lu KB total)\r\n",
           (unsigned long)throughput_kBs,
           (unsigned long)(total_bytes / 1024));

    if (count > 0) {
        sort_u32(g_latencies, count);
        report_percentiles("Sustained 512B", g_latencies, count);

        // Also print worst-10 latencies
        printf("[SD-BENCH]   Worst 10 latencies (us):");
        uint32_t start = count > 10 ? count - 10 : 0;
        for (uint32_t i = start; i < count; i++) {
            printf(" %lu", (unsigned long)g_latencies[i]);
        }
        printf("\r\n");
    }
}

// ---------------------------------------------------------------------------
// HAL DMA callbacks
// ---------------------------------------------------------------------------
extern "C" void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    (void)hsd;
    g_dma_tx_complete = true;
}

extern "C" void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
    (void)hsd;
    g_dma_tx_error = true;
}

// ---------------------------------------------------------------------------
// Phase 4: DMA multi-block writes — measures wall-clock + CPU-free time
// ---------------------------------------------------------------------------
static void bench_dma_multi_block(SD_HandleTypeDef* hsd, uint8_t* buf,
                                   uint32_t blocks_per_write, uint32_t num_writes,
                                   uint32_t base_block, const char* label) {
    uint32_t buf_size = blocks_per_write * 512u;
    printf("[SD-BENCH] %s (DMA): %lu × %lu-block (%lu KB) writes...\r\n",
           label, (unsigned long)num_writes,
           (unsigned long)blocks_per_write,
           (unsigned long)(buf_size / 1024));

    fill_pattern(buf, buf_size, 0xDEADC0DE ^ blocks_per_write);

    uint32_t total_start = dwt_cycles();
    uint32_t errors = 0;
    uint32_t total_cpu_free_cycles = 0;

    for (uint32_t i = 0; i < num_writes; i++) {
        if (!wait_card_ready(hsd, 1000)) {
            HAL_SD_Abort(hsd);
            if (!wait_card_ready(hsd, 1000)) {
                errors++;
                g_latencies[i] = 0;
                continue;
            }
        }

        g_dma_tx_complete = false;
        g_dma_tx_error = false;

        uint32_t t0 = dwt_cycles();
        HAL_StatusTypeDef st = HAL_SD_WriteBlocks_DMA(hsd, buf,
                                                       base_block + i * blocks_per_write,
                                                       blocks_per_write);
        uint32_t t_submit = dwt_cycles();

        if (st != HAL_OK) {
            errors++;
            g_latencies[i] = 0;
            HAL_SD_Abort(hsd);
            continue;
        }

        // CPU is free here — count cycles until DMA completes
        uint32_t cpu_free_start = dwt_cycles();
        uint32_t timeout_tick = HAL_GetTick() + 5000; // 5s timeout
        while (!g_dma_tx_complete && !g_dma_tx_error) {
            if (HAL_GetTick() > timeout_tick) {
                g_dma_tx_error = true;
                printf("[SD-BENCH]   DMA timeout! state=%u err=0x%lX\r\n",
                       (unsigned)hsd->State, (unsigned long)hsd->ErrorCode);
                break;
            }
        }
        uint32_t t1 = dwt_cycles();
        total_cpu_free_cycles += (t1 - cpu_free_start);

        if (g_dma_tx_error) {
            errors++;
            g_latencies[i] = 0;
            HAL_SD_Abort(hsd);
        } else {
            g_latencies[i] = cycles_to_us(t1 - t0);
        }
    }

    uint32_t total_end = dwt_cycles();
    uint32_t total_us = cycles_to_us(total_end - total_start);
    uint32_t cpu_free_us = cycles_to_us(total_cpu_free_cycles);

    if (errors > 0) {
        printf("[SD-BENCH]   ERRORS: %lu / %lu writes failed!\r\n",
               (unsigned long)errors, (unsigned long)num_writes);
    }

    uint32_t total_bytes = num_writes * blocks_per_write * 512u;
    uint32_t throughput_kBs = 0;
    if (total_us > 0) {
        throughput_kBs = (uint32_t)((uint64_t)total_bytes * 1000000ull / total_us / 1024ull);
    }
    printf("[SD-BENCH]   Total: %lu us for %lu KB => %lu KB/s\r\n",
           (unsigned long)total_us,
           (unsigned long)(total_bytes / 1024),
           (unsigned long)throughput_kBs);
    printf("[SD-BENCH]   CPU free: %lu us / %lu us total (%lu%%)\r\n",
           (unsigned long)cpu_free_us, (unsigned long)total_us,
           (unsigned long)(total_us > 0 ? cpu_free_us * 100 / total_us : 0));

    char plabel[64];
    snprintf(plabel, sizeof(plabel), "%s DMA (%luKB)", label, (unsigned long)(buf_size / 1024));
    sort_u32(g_latencies, num_writes);
    report_percentiles(plabel, g_latencies, num_writes);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
extern "C" void sd_benchmark_run(SD_HandleTypeDef* hsd) {
    printf("\r\n");
    printf("========================================\r\n");
    printf("  SD Card Write Benchmark\r\n");
    printf("========================================\r\n");

    if (hsd == NULL || hsd->Instance == NULL) {
        printf("[SD-BENCH] ERROR: NULL SD handle!\r\n");
        return;
    }

    if (hsd->State != HAL_SD_STATE_READY) {
        printf("[SD-BENCH] ERROR: SD not ready (state=%u)\r\n", (unsigned)hsd->State);
        return;
    }

    // Print card info
    printf("[SD-BENCH] Card info:\r\n");
    printf("  BlockNbr    = %lu\r\n", (unsigned long)hsd->SdCard.BlockNbr);
    printf("  BlockSize   = %lu\r\n", (unsigned long)hsd->SdCard.BlockSize);
    printf("  LogBlockNbr = %lu\r\n", (unsigned long)hsd->SdCard.LogBlockNbr);
    printf("  LogBlockSz  = %lu\r\n", (unsigned long)hsd->SdCard.LogBlockSize);
    printf("  CardType    = %lu\r\n", (unsigned long)hsd->SdCard.CardType);
    printf("  CardSpeed   = %lu\r\n", (unsigned long)hsd->SdCard.CardSpeed);
    printf("  Class       = %lu\r\n", (unsigned long)hsd->SdCard.Class);
    uint32_t cap_mb = (uint32_t)((uint64_t)hsd->SdCard.LogBlockNbr *
                                  hsd->SdCard.LogBlockSize / (1024 * 1024));
    printf("  Capacity    = %lu MB\r\n", (unsigned long)cap_mb);
    printf("  ClockDiv    = %lu\r\n", (unsigned long)hsd->Init.ClockDiv);
    printf("  BusWide     = %s\r\n",
           hsd->Init.BusWide == SDMMC_BUS_WIDE_4B ? "4-bit" :
           hsd->Init.BusWide == SDMMC_BUS_WIDE_1B ? "1-bit" : "8-bit");
    printf("  SDMMC clock ~ %lu MHz\r\n",
           (unsigned long)(200 / (hsd->Init.ClockDiv * 2)));
    printf("  SD State    = %u\r\n", (unsigned)hsd->State);

    // Check clock configuration
    {
        uint32_t rcc_cr = RCC->CR;
        uint32_t pll2on = (rcc_cr >> 26) & 1;
        uint32_t pll2rdy = (rcc_cr >> 27) & 1;
        uint32_t d1ccipr = RCC->D1CCIPR;
        uint32_t sdmmcsel = (d1ccipr >> 16) & 1;
        printf("  PLL2 ON=%lu RDY=%lu  SDMMCSEL=%lu (0=PLL1q, 1=PLL2r)\r\n",
               (unsigned long)pll2on, (unsigned long)pll2rdy, (unsigned long)sdmmcsel);

        // Check SDMMC1 peripheral clock
        uint32_t ahb3enr = RCC->AHB3ENR;
        uint32_t sdmmc1en = (ahb3enr >> 16) & 1;
        printf("  SDMMC1 CLK EN=%lu\r\n", (unsigned long)sdmmc1en);
    }
    printf("\r\n");

    // Init DWT
    dwt_init();

    // Pre-flight: single test write to verify SD card is writable
    {
        fill_pattern(g_buf_512, 512, 0x12345678);
        printf("[SD-BENCH] Pre-flight: writing 1 block to block %lu...\r\n",
               (unsigned long)kStartBlock);
        printf("[SD-BENCH]   buf addr=0x%08lX (should be 0x24xxxxxx for AXI SRAM)\r\n",
               (unsigned long)(uintptr_t)g_buf_512);

        // Wait for card to be ready
        HAL_SD_CardStateTypeDef cardState = HAL_SD_GetCardState(hsd);
        printf("[SD-BENCH]   Card state before write: %lu\r\n", (unsigned long)cardState);

        HAL_StatusTypeDef st = HAL_SD_WriteBlocks(hsd, g_buf_512, kStartBlock, 1, 5000);
        printf("[SD-BENCH]   Result: HAL=%d state=%u err=0x%lX\r\n",
               (int)st, (unsigned)hsd->State, (unsigned long)hsd->ErrorCode);
        printf("[SD-BENCH]   Result: HAL=%d state=%u err=0x%lX\r\n",
               (int)st, (unsigned)hsd->State, (unsigned long)hsd->ErrorCode);
        if (st != HAL_OK) {
            printf("[SD-BENCH]   Write failed! Aborting benchmark.\r\n");
            // Try to read-back the state
            HAL_SD_CardStatusTypeDef cardStatus;
            HAL_StatusTypeDef st2 = HAL_SD_GetCardStatus(hsd, &cardStatus);
            if (st2 == HAL_OK) {
                printf("[SD-BENCH]   CardStatus: DataBusWidth=%u SpeedClass=%u\r\n",
                       cardStatus.DataBusWidth, cardStatus.SpeedClass);
            }
            return;
        }
        printf("[SD-BENCH]   Pre-flight write OK!\r\n");

        // Verify: read it back
        uint8_t readback[512] __attribute__((aligned(32)));
        st = HAL_SD_ReadBlocks(hsd, readback, kStartBlock, 1, 5000);
        printf("[SD-BENCH]   Read-back: HAL=%d first4=[%02X %02X %02X %02X]\r\n",
               (int)st, readback[0], readback[1], readback[2], readback[3]);
    }
    printf("\r\n");

    // Run phases
    bench_single_block(hsd);
    printf("\r\n");

    // Phase 2: 8-block (4KB) writes
    uint32_t mb_base = kStartBlock + kSingleBlockCount + 100;
    bench_multi_block_generic(hsd, g_buf_4k, 8, 512, mb_base, "Phase 2");
    printf("\r\n");

    // Phase 2b: 16-block (8KB) writes
    mb_base += 512 * 8 + 100;
    bench_multi_block_generic(hsd, g_buf_8k, 16, 256, mb_base, "Phase 2b");
    printf("\r\n");

    // Phase 2c: 32-block (16KB) writes
    mb_base += 256 * 16 + 100;
    bench_multi_block_generic(hsd, g_buf_16k, 32, 128, mb_base, "Phase 2c");
    printf("\r\n");

    bench_sustained(hsd);
    printf("\r\n");

    // Phase 4: DMA multi-block writes
    printf("[SD-BENCH] Starting DMA phases...\r\n");
    uint32_t dma_base = mb_base + 256 * 32 + 100;
    bench_dma_multi_block(hsd, g_buf_4k, 8, 512, dma_base, "Phase 4a");
    printf("\r\n");

    dma_base += 512 * 8 + 100;
    bench_dma_multi_block(hsd, g_buf_8k, 16, 256, dma_base, "Phase 4b");
    printf("\r\n");

    dma_base += 256 * 16 + 100;
    bench_dma_multi_block(hsd, g_buf_16k, 32, 128, dma_base, "Phase 4c");

    printf("\r\n========================================\r\n");
    printf("  Benchmark complete\r\n");
    printf("========================================\r\n");
}
