#ifndef DRIVERS_RGBLED_WS2812_BITBANG_HPP
#define DRIVERS_RGBLED_WS2812_BITBANG_HPP

#include <cstddef>
#include <cstdint>

namespace Drivers {
namespace RgbLed {

// One LED colour, 0..255 per channel. The driver sends it in the LED's wire
// order (GRB for WS2812B), so callers always think in RGB.
struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool operator==(const Rgb& o) const { return r == o.r && g == o.g && b == o.b; }
    bool operator!=(const Rgb& o) const { return !(*this == o); }
};

// ── WS2812 class timing ─────────────────────────────────────────────────────
// Every timing constant lives here. The part number is unconfirmed: the FC
// schematic names no LED part (U4, U5, U6, symbol ert:NEW_RGB_LED, BOM
// manufacturer field "JULIAN"), so these are WS2812B class values; retune them
// here for the real part. Colour order is GRB, most significant bit first.
namespace Timing {
constexpr uint32_t kBitPeriodNs = 1250u;  // one bit, 800 kbit/s
constexpr uint32_t kZeroHighNs  = 350u;   // high time of a 0 bit; inside both the
                                          // classic WS2812B (250..550) and the
                                          // WS2812B V5 (220..380) windows
constexpr uint32_t kOneHighNs   = 800u;   // high time of a 1 bit
constexpr uint32_t kResetLowUs  = 300u;   // low gap that latches the frame
} // namespace Timing

} // namespace RgbLed
} // namespace Drivers

#if !defined(UNIT_TEST_ENV)
#include "Drivers/STM32HAL/stm32hal.h"

namespace Drivers {
namespace RgbLed {

// Bit banged driver for N chained WS2812 class LEDs on one GPIO.
//
// On the FC the chain is U4 -> U5 -> U6, fed from PG12 (net LEDCTRL, 33 ohm
// series, 4.7k pull down). No timer reaches PG12, hence the bit bang.
//
// show() masks interrupts for the bit stream of one frame only: 24 bits per
// LED at 1.25 us, so three LEDs are 72 bits = 90 us with interrupts off.
// The 300 us reset gap that follows runs with interrupts enabled, so one
// show() of three LEDs costs about 390 us of loop time. Call it only when a
// colour changed.
//
// Bit timing uses the DWT cycle counter. app_timebase_init() already enables
// it; init() unlocks the DWT, enables it only if it is not running, never
// resets CYCCNT (so the app timebase is left untouched), and marks the driver
// ready only once it has seen the counter move. Cycle counts come from
// SystemCoreClock at init().
//
// No wait can hang the board: each one gives up after twice its target in
// loop iterations (one iteration is at least one cycle, so a counting CYCCNT
// always finishes first). A wait that gives up, or a counter found stopped by
// the recheck at the top of show(), aborts the frame with the line low and
// sets a sticky fault; from then on show() does nothing and faulted() is true.
//
// The GPIO port clock must already be on (MX_GPIO_Init enables GPIOG).
class Ws2812BitBang {
public:
    Ws2812BitBang(GPIO_TypeDef* port, uint16_t pin) : port_(port), pin_(pin) {}

    // Configures the pin push pull, very high speed, driven low; computes the
    // cycle counts. Call once SystemCoreClock is final.
    void init();

    // Sends n colours down the chain, then holds the line low for the reset
    // gap. Does nothing before a successful init() or after a fault.
    void show(const Rgb* colours, size_t n);

    // True once a wait gave up or the counter was found stopped. Sticky.
    bool faulted() const { return faulted_; }

private:
    static bool counterMoves(uint32_t spins);
    bool waitCycles(uint32_t start, uint32_t cycles) const;
    void abortFrame(uint32_t primask);

    GPIO_TypeDef* port_;
    uint16_t pin_;
    bool ready_ = false;
    bool faulted_ = false;
    uint32_t period_cycles_ = 0;
    uint32_t zero_high_cycles_ = 0;
    uint32_t one_high_cycles_ = 0;
    uint32_t reset_cycles_ = 0;
};

} // namespace RgbLed
} // namespace Drivers
#endif // !UNIT_TEST_ENV

#endif // DRIVERS_RGBLED_WS2812_BITBANG_HPP
