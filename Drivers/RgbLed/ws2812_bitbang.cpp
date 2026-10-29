#include "Drivers/RgbLed/ws2812_bitbang.hpp"

#if !defined(UNIT_TEST_ENV)

namespace Drivers {
namespace RgbLed {

namespace {
constexpr uint32_t nsToCycles(uint32_t hz, uint32_t ns) {
    return static_cast<uint32_t>((static_cast<uint64_t>(hz) * ns) / 1000000000ull);
}
} // namespace

void Ws2812BitBang::init() {
    // Reuse the DWT counter the app timebase runs; start it only if stopped.
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }

    const uint32_t hz = SystemCoreClock;
    period_cycles_    = nsToCycles(hz, Timing::kBitPeriodNs);
    zero_high_cycles_ = nsToCycles(hz, Timing::kZeroHighNs);
    one_high_cycles_  = nsToCycles(hz, Timing::kOneHighNs);
    reset_cycles_     = nsToCycles(hz, Timing::kResetLowUs * 1000u);

    port_->BSRR = static_cast<uint32_t>(pin_) << 16;  // low before it drives
    GPIO_InitTypeDef gpio{};
    gpio.Pin   = pin_;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port_, &gpio);
    port_->BSRR = static_cast<uint32_t>(pin_) << 16;

    ready_ = period_cycles_ > 0u;
}

void Ws2812BitBang::waitCycles(uint32_t start, uint32_t cycles) const {
    while ((DWT->CYCCNT - start) < cycles) {
    }
}

void Ws2812BitBang::show(const Rgb* colours, size_t n) {
    if (!ready_ || colours == nullptr) {
        return;
    }
    const uint32_t set   = pin_;
    const uint32_t reset = static_cast<uint32_t>(pin_) << 16;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t bit_start = DWT->CYCCNT;
    for (size_t i = 0; i < n; ++i) {
        const uint8_t wire[3] = {colours[i].g, colours[i].r, colours[i].b};
        for (uint8_t byte : wire) {
            for (uint8_t mask = 0x80u; mask != 0u; mask >>= 1) {
                port_->BSRR = set;
                waitCycles(bit_start, (byte & mask) ? one_high_cycles_ : zero_high_cycles_);
                port_->BSRR = reset;
                waitCycles(bit_start, period_cycles_);
                bit_start += period_cycles_;
            }
        }
    }
    if ((primask & 0x1u) == 0u) {
        __enable_irq();
    }

    // Reset gap: the line stays low; interrupts may run.
    waitCycles(bit_start, reset_cycles_);
}

} // namespace RgbLed
} // namespace Drivers

#endif // !UNIT_TEST_ENV
