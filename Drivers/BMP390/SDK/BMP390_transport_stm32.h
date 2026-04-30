#pragma once

#include "stm32h7xx_hal.h"
#include "../bmp3.h"
#include <cstdint>

// ── SPI context passed as intf_ptr to the BMP3 SDK ────────────────────────────

struct Bmp3SpiCtx {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef*      cs_port;
    uint16_t           cs_pin;
};

// ── Vendor SDK callbacks ───────────────────────────────────────────────────────
// Register these directly on bmp3_dev: dev.read / dev.write / dev.delay_us

BMP3_INTF_RET_TYPE bmp3_spi_read (uint8_t reg, uint8_t* dst, uint32_t len, void* ctx);
BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg, const uint8_t* src, uint32_t len, void* ctx);
void               bmp3_delay_us_hal(uint32_t us, void* ctx);

// ── DWT microsecond timer ──────────────────────────────────────────────────────

void     bmp3_enable_dwt();
uint64_t bmp3_now_us();
