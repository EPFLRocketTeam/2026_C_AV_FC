#include "Drivers/RgbLed/ws2812_bitbang.hpp"

#if !defined(UNIT_TEST_ENV)

namespace Drivers {
namespace RgbLed {

namespace {
constexpr uint32_t kInitProbeSpins = 256u;  // a few hundred cycles at least
constexpr uint32_t kShowProbeSpins = 8u;

constexpr uint32_t nsToCycles(uint32_t hz, uint32_t ns) {
    return static_cast<uint32_t>((static_cast<uint64_t>(hz) * ns) / 1000000000ull);
}
} // namespace

void Ws2812BitBang::init() {
    bool counter_ok = false;
#if defined(DWT) && defined(CoreDebug)
    // Reuse the DWT counter the app timebase runs. Trace must be on for the
    // DWT to respond; unlock it (a no op where the lock is not implemented),
    // then start the counter only if stopped. CYCCNT is never reset.
    if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0u) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->LAR = 0xC5ACCE55u;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    counter_ok = counterMoves(kInitProbeSpins);
#endif

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

    ready_ = counter_ok && period_cycles_ > 0u;
}

bool Ws2812BitBang::counterMoves(uint32_t spins) {
#if defined(DWT)
    const uint32_t a = DWT->CYCCNT;
    for (uint32_t i = 0; i < spins; ++i) {
        __NOP();
    }
    return DWT->CYCCNT != a;
#else
    (void)spins;
    return false;
#endif
}

// Waits until CYCCNT is cycles past start. Gives up (returns false) after
// 2 * cycles polls: each poll takes at least one cycle, so only a counter that
// stopped can get there.
bool Ws2812BitBang::waitCycles(uint32_t start, uint32_t cycles) const {
    const uint32_t limit = 2u * cycles;
    for (uint32_t polls = 0; (DWT->CYCCNT - start) < cycles; ++polls) {
        if (polls > limit) {
            return false;
        }
    }
    return true;
}

void Ws2812BitBang::abortFrame(uint32_t primask) {
    port_->BSRR = static_cast<uint32_t>(pin_) << 16;
    if ((primask & 0x1u) == 0u) {
        __enable_irq();
    }
    faulted_ = true;
}

void Ws2812BitBang::show(const Rgb* colours, size_t n) {
    if (!ready_ || faulted_ || colours == nullptr) {
        return;
    }
    // Cheap recheck before interrupts go off.
    if (!counterMoves(kShowProbeSpins)) {
        faulted_ = true;
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
                if (!waitCycles(bit_start, (byte & mask) ? one_high_cycles_ : zero_high_cycles_)) {
                    abortFrame(primask);
                    return;
                }
                port_->BSRR = reset;
                if (!waitCycles(bit_start, period_cycles_)) {
                    abortFrame(primask);
                    return;
                }
                bit_start += period_cycles_;
            }
        }
    }
    if ((primask & 0x1u) == 0u) {
        __enable_irq();
    }

    // Reset gap: the line stays low; interrupts may run.
    if (!waitCycles(bit_start, reset_cycles_)) {
        faulted_ = true;
    }
}

} // namespace RgbLed
} // namespace Drivers

#endif // !UNIT_TEST_ENV
