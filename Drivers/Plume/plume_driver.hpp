#pragma once

extern "C" {
    #include "stm32hal.h"
    #include "plume/context.h"
    #include "plume/driver.h"
    #include "plume/atomic.h"
};

class SDCardInterface {
private:
    SD_HandleTypeDef* hsd;
    
    struct plume_context context;
    struct plume_driver  driver;

    struct plume_snapshot snapshot;
    bool inTransaction = false;
    bool transactionFailed = false;
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

    /// Ring buffer bytes currently occupied
    size_t arena_used_bytes() const { return (size_t)context.rb_number_bytes_used; }
    /// Total ring buffer capacity
    size_t arena_total_bytes() const { return (size_t)context.rb_number_bytes_total; }
};
