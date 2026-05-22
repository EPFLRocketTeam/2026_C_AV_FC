#pragma once

extern "C" {
    #include "stm32hal.h"
    #include "plume/context.h"
    #include "plume/driver.h"
    #include "plume/atomic.h"
};

/* SD card DMA timing statistics (populated by plume_driver, read from main) */
struct SdTimingStats {
    volatile uint64_t dma_start_us;
    volatile uint64_t dma_cb_us;
    uint32_t last_batch_size;
    uint32_t dma_count;
    uint32_t dma_error_count;
    uint32_t max_xfer_us;      /* max DMA transfer time (start→callback) */
    uint32_t max_prog_us;      /* max card programming (callback→ready)  */
    uint32_t max_cycle_us;     /* max full cycle (start→ready)           */
    uint64_t sum_cycle_us;
    uint32_t total_blocks;
    uint32_t min_batch;
    uint32_t max_batch;
    uint32_t last_error_code;  /* hsd->ErrorCode from last error callback */
};

/* Returns snapshot of current timing stats and resets for next window */
SdTimingStats sd_timing_snapshot();

class SDCardInterface {
private:
    SD_HandleTypeDef* hsd;
    
    struct plume_context context;
    struct plume_driver  driver;

    struct plume_snapshot snapshot;
    bool inTransaction = false;
    bool transactionFailed = false;
    bool lastTxFailed_ = false;
public:
    bool init_sd_card (
        SD_HandleTypeDef* hsd,
        uint8_t* arena_buffer,
        size_t   arena_length
    );
    bool open_file ();

    size_t number_files_remaining ();
    size_t disk_size_remaining ();

    void beginTransaction ();
    void endTransaction ();

    uint8_t write (const uint8_t* buffer, int length);
    uint8_t tick ();

    /// Whether the last completed transaction was rolled back.
    bool lastTransactionFailed() const { return lastTxFailed_; }

    /// Ring buffer bytes currently occupied
    size_t arena_used_bytes() const { return (size_t)context.rb_number_bytes_used; }
    /// Total ring buffer capacity
    size_t arena_total_bytes() const { return (size_t)context.rb_number_bytes_total; }
};
