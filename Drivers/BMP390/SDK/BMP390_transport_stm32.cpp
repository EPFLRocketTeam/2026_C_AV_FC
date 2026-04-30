#include "BMP390_transport_stm32.h"
#include <cstring>

// ── SPI transport ─────────────────────────────────────────────────────────────
// BMP390 SPI read protocol: [addr|0x80] [dummy rx] [data × len]
// The SDK already ORs 0x80 onto reg before calling read, so we pass it through.
// Transaction size = 1 (addr) + 1 (dummy) + len.

static constexpr size_t kBufMax = 32; // calibration (21 B) + 2 overhead + margin

BMP3_INTF_RET_TYPE bmp3_spi_read(uint8_t reg, uint8_t* dst, uint32_t len, void* ctx) {
    if (!ctx || !dst || len == 0 || (len + 2) > kBufMax) return -1;
    auto* c = static_cast<Bmp3SpiCtx*>(ctx);

    uint8_t tx[kBufMax] = {};
    uint8_t rx[kBufMax] = {};
    tx[0] = reg; // bit 7 already set by the SDK for reads

    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(c->hspi, tx, rx,
                                                    static_cast<uint16_t>(len + 1), 10);
    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);

    if (st != HAL_OK) return -1;
    memcpy(dst, rx + 1, len); // skip addr-echo + dummy byte
    return BMP3_INTF_RET_SUCCESS;
}

BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg, const uint8_t* src, uint32_t len, void* ctx) {
    if (!ctx || !src || len == 0 || (len + 1) > kBufMax) return -1;
    auto* c = static_cast<Bmp3SpiCtx*>(ctx);

    uint8_t tx[kBufMax];
    tx[0] = reg & 0x7Fu; // clear bit 7 for writes
    memcpy(tx + 1, src, len);

    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(c->hspi, tx,
                                             static_cast<uint16_t>(len + 1), 10);
    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);

    return (st == HAL_OK) ? BMP3_INTF_RET_SUCCESS : -1;
}

void bmp3_delay_us_hal(uint32_t us, void* /*ctx*/) {
    // HAL_Delay is ms-resolution; for short delays spin on DWT if available.
    if (us >= 1000) {
        HAL_Delay((us + 999) / 1000);
    } else {
        uint32_t cycles = (us * (SystemCoreClock / 1000000UL));
        uint32_t start  = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles) {}
    }
}

// ── DWT microsecond timer (same approach as the raw driver) ───────────────────

void bmp3_enable_dwt() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint64_t bmp3_now_us() {
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
        return HAL_GetTick() * 1000ULL;
    }
    static uint32_t prev_cyc = 0;
    static uint64_t accum_us = 0;
    uint32_t cyc  = DWT->CYCCNT;
    accum_us += static_cast<uint64_t>(cyc - prev_cyc) * 1000000ULL / SystemCoreClock;
    prev_cyc  = cyc;
    return accum_us;
}
