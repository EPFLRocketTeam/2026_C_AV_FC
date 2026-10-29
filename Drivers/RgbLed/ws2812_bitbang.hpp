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
// Low times are the period minus the high time: 0 bit 350 high, 900 low;
// 1 bit 650 high, 600 low. Both halves fit the classic WS2812B and the
// WS2812B V5 windows (V5 wants T0H about 220..380 and lows of 580 or more).
constexpr uint32_t kZeroHighNs  = 350u;   // high time of a 0 bit
constexpr uint32_t kOneHighNs   = 650u;   // high time of a 1 bit
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
// LED at 1.25 us, so three LEDs are 72 bits = 90 us with interrupts off, and
// that is all a show() costs. It does not wait out the 300 us reset gap after
// the frame: the caller's 20 ms tick already guarantees it. Instead show()
// skips the frame when less than the reset gap has passed since the previous
// frame ended (cycle counter, wrap safe subtraction; a HAL tick check rules
// out a false skip once the 8.9 s CYCCNT wrap aliases). Call it only when a
// colour changed, and no faster than the reset gap.
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

    // Sends n colours down the chain and leaves the line low. Does nothing
    // before a successful init(), after a fault, or within the reset gap of
    // the previous frame.
    void show(const Rgb* colours, size_t n);

    // True once init() saw the cycle counter run; show() is a no op otherwise.
    bool ready() const { return ready_; }

    // True once a wait gave up or the counter was found stopped. Sticky.
    // The driver never prints; the caller reports it.
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
    bool has_shown_ = false;
    uint32_t last_end_cycle_ = 0;  // CYCCNT when the previous frame ended
    uint32_t last_end_ms_ = 0;     // HAL_GetTick() at the same moment
};

} // namespace RgbLed
} // namespace Drivers
#endif // !UNIT_TEST_ENV

#endif // DRIVERS_RGBLED_WS2812_BITBANG_HPP
