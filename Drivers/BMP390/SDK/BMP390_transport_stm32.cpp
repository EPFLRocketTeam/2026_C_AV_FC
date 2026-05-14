#include "BMP390_transport_stm32.h"
#include <cstring>
#include <cstdio>

// ── SPI transport ─────────────────────────────────────────────────────────────
// BMP390 SPI read protocol: [addr|0x80] [dummy rx] [data...].
// Bosch's SDK already ORs 0x80 into reg and includes the dummy byte in len
// for SPI reads, so the transport sends one address byte plus len clock bytes.

static constexpr size_t kBufMax = 32; // calibration (21 B) + 2 overhead + margin

// Debug counter: prints SPI details for the first N calls per sensor init
static volatile uint32_t g_bmp3_spi_debug_calls = 0;
static constexpr uint32_t kBmp3SpiDebugMaxCalls = 20; // print first 20 SPI transactions

void bmp3_spi_debug_reset() {
    g_bmp3_spi_debug_calls = 0;
}

BMP3_INTF_RET_TYPE bmp3_spi_read(uint8_t reg, uint8_t* dst, uint32_t len, void* ctx) {
    if (!ctx || !dst || len == 0 || (len + 1) > kBufMax) {
        printf("[BMP3-SPI] READ guard fail: ctx=%p dst=%p len=%lu\r\n",
               ctx, dst, (unsigned long)len);
        return -1;
    }
    auto* c = static_cast<Bmp3SpiCtx*>(ctx);

    uint8_t tx[kBufMax] = {};
    uint8_t rx[kBufMax] = {};
    tx[0] = reg; // bit 7 already set by the SDK for reads

    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(c->hspi, tx, rx,
                                                    static_cast<uint16_t>(len + 1), 10);
    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);

    if (g_bmp3_spi_debug_calls < kBmp3SpiDebugMaxCalls) {
        g_bmp3_spi_debug_calls++;
        printf("[BMP3-SPI] RD reg=0x%02X len=%lu HAL=%d SPI_State=%u SPI_Err=0x%lX",
               reg, (unsigned long)len, (int)st,
               (unsigned)c->hspi->State,
               (unsigned long)c->hspi->ErrorCode);
        if (st == HAL_OK && len <= 4) {
            printf(" rx[1..%lu]=", (unsigned long)len);
            for (uint32_t i = 0; i < len; i++) printf("%02X ", rx[i+1]);
        }
        printf("\r\n");
    }

    if (st != HAL_OK) return -1;
    memcpy(dst, rx + 1, len); // skip addr-echo; SDK strips dummy from dst
    return BMP3_INTF_RET_SUCCESS;
}

BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg, const uint8_t* src, uint32_t len, void* ctx) {
    if (!ctx || !src || len == 0 || (len + 1) > kBufMax) {
        printf("[BMP3-SPI] WRITE guard fail: ctx=%p src=%p len=%lu\r\n",
               ctx, src, (unsigned long)len);
        return -1;
    }
    auto* c = static_cast<Bmp3SpiCtx*>(ctx);

    uint8_t tx[kBufMax];
    tx[0] = reg & 0x7Fu; // clear bit 7 for writes
    memcpy(tx + 1, src, len);

    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(c->hspi, tx,
                                             static_cast<uint16_t>(len + 1), 10);
    HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET);

    if (g_bmp3_spi_debug_calls < kBmp3SpiDebugMaxCalls) {
        g_bmp3_spi_debug_calls++;
        printf("[BMP3-SPI] WR reg=0x%02X len=%lu HAL=%d SPI_State=%u\r\n",
               reg, (unsigned long)len, (int)st, (unsigned)c->hspi->State);
    }

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
