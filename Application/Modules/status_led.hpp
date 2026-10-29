#ifndef APPLICATION_MODULES_STATUS_LED_HPP
#define APPLICATION_MODULES_STATUS_LED_HPP

// Pattern engine for the three FC status LEDs. Pure: time comes in as now_ms,
// colours go out; no HAL. The super loop feeds it and pushes the colours to
// the strip only when update() reports a change.
//
//   LED 1  flight state colour; three blinks (100 ms on, 100 ms off) on every
//          state entry; red blink at 2 Hz in both abort states.
//   LED 2  sensors: green when ok, red blink at 2 Hz otherwise.
//   LED 3  logging: green when the SD log is active, off otherwise.
//   error  all three blink red at 4 Hz, overriding everything.

#include <cstddef>
#include <cstdint>

#include "Application/Data/fsm.hpp"
#include "Drivers/RgbLed/ws2812_bitbang.hpp"

namespace flight_computer {

class StatusLed {
public:
    using Rgb = Drivers::RgbLed::Rgb;

    struct Input {
        State av_state;
        bool sensors_ok;
        bool sd_ok;
        bool error;
        uint32_t now_ms;
    };

    static constexpr size_t kLedCount = 3;

    // Brightness cap: 25 percent, readable without blinding. Palette entries
    // are full scale and get scaled down to this.
    static constexpr uint8_t kMaxChannel = 64;

    static constexpr uint32_t kMinUpdateMs   = 20;   // poll period: the caller ticks update() this often
    static constexpr uint32_t kEntryHalfMs   = 100;  // entry blink: 100 on, 100 off
    static constexpr uint32_t kEntryBlinks   = 3;
    static constexpr uint32_t kAbortHalfMs   = 250;  // 2 Hz
    static constexpr uint32_t kSensorHalfMs  = 250;  // 2 Hz
    static constexpr uint32_t kErrorHalfMs   = 125;  // 4 Hz

    // Full scale colour per State value, indexed by the enum.
    static constexpr Rgb kStatePalette[] = {
        {255, 255, 255},  // INIT             white
        {0, 255, 255},    // CALIBRATION      cyan
        {0, 0, 255},      // FILLING          blue
        {255, 255, 0},    // ARMED            yellow
        {255, 96, 0},     // PRESSURIZATION   orange
        {255, 0, 255},    // TRY_IGNITION     magenta
        {255, 0, 0},      // IGNITION         red
        {255, 0, 0},      // BURN             red
        {0, 255, 0},      // ASCENT           green
        {128, 0, 255},    // DESCENT          purple
        {0, 64, 0},       // LANDED           dim green
        {255, 0, 0},      // ABORT_ON_GROUND  red
        {255, 0, 0},      // ABORT_IN_FLIGHT  red
    };
    static constexpr size_t kStateCount = sizeof(kStatePalette) / sizeof(kStatePalette[0]);
    static_assert(kStateCount == static_cast<size_t>(ABORT_IN_FLIGHT) + 1,
                  "one palette entry per State value");

    static constexpr Rgb kOff{0, 0, 0};
    static constexpr Rgb kRed{255, 0, 0};
    static constexpr Rgb kGreen{0, 255, 0};

    static constexpr Rgb scale(Rgb c) {
        return Rgb{static_cast<uint8_t>(c.r * kMaxChannel / 255),
                   static_cast<uint8_t>(c.g * kMaxChannel / 255),
                   static_cast<uint8_t>(c.b * kMaxChannel / 255)};
    }

    // Scaled steady colour of a state. Unknown values show red, as the FSM
    // sends an unknown state to ABORT_ON_GROUND.
    static constexpr Rgb stateColour(State s) {
        return scale(static_cast<size_t>(s) < kStateCount ? kStatePalette[s] : kRed);
    }

    // Computes the three colours for this instant. Returns true only when
    // they differ from the previous call (always true on the first call).
    bool update(const Input& in, Rgb out[kLedCount]) {
        if (!has_state_ || in.av_state != state_) {
            state_ = in.av_state;
            entry_ms_ = in.now_ms;
            has_state_ = true;
        }

        Rgb next[kLedCount];
        if (in.error) {
            const Rgb c = blinkOn(in.now_ms, kErrorHalfMs) ? scale(kRed) : kOff;
            next[0] = next[1] = next[2] = c;
        } else {
            next[0] = stateLed(in.now_ms);
            next[1] = in.sensors_ok ? scale(kGreen)
                                    : (blinkOn(in.now_ms, kSensorHalfMs) ? scale(kRed) : kOff);
            next[2] = in.sd_ok ? scale(kGreen) : kOff;
        }

        bool changed = !has_out_;
        for (size_t i = 0; i < kLedCount; ++i) {
            changed = changed || next[i] != last_[i];
            last_[i] = next[i];
            out[i] = next[i];
        }
        has_out_ = true;
        return changed;
    }

private:
    static constexpr bool isAbort(State s) {
        return s == ABORT_ON_GROUND || s == ABORT_IN_FLIGHT;
    }
    static constexpr uint32_t entryBlinkMs() { return 2 * kEntryHalfMs * kEntryBlinks; }

    // On during the first half of each period, counted from origin.
    static constexpr bool blinkOn(uint32_t t, uint32_t half_ms, uint32_t origin = 0) {
        return ((t - origin) / half_ms) % 2 == 0;
    }

    Rgb stateLed(uint32_t now) const {
        if (isAbort(state_)) {
            return blinkOn(now, kAbortHalfMs, entry_ms_) ? scale(kRed) : kOff;
        }
        if (now - entry_ms_ < entryBlinkMs()) {
            return blinkOn(now, kEntryHalfMs, entry_ms_) ? stateColour(state_) : kOff;
        }
        return stateColour(state_);
    }

    bool has_state_ = false;
    bool has_out_ = false;
    State state_ = INIT;
    uint32_t entry_ms_ = 0;
    Rgb last_[kLedCount] = {};
};

} // namespace flight_computer

#endif // APPLICATION_MODULES_STATUS_LED_HPP
