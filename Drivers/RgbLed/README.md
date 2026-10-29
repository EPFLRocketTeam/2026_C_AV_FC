# Status LEDs

The FC carries three chained addressable RGB LEDs, U4, U5 and U6, fed from PG12
(net LEDCTRL). `ws2812_bitbang` drives them by bit banging the GPIO; the pattern
engine is `Application/Modules/status_led.hpp`, ticked every 20 ms from the super
loop. Brightness is capped at 25 percent (`StatusLed::kMaxChannel`).

| LED | Shows |
|-----|-------|
| 1 (U4) | Flight state colour |
| 2 (U5) | Sensors: green when at least one IMU and one baro are healthy, red blink at 2 Hz otherwise |
| 3 (U6) | Logging: green when SD logging is active, off otherwise |

## State colours (LED 1)

| State | Colour |
|-------|--------|
| INIT | white |
| CALIBRATION | cyan |
| FILLING | blue |
| ARMED | yellow |
| PRESSURIZATION | orange |
| TRY_IGNITION | magenta |
| IGNITION, BURN | red |
| ASCENT | green |
| DESCENT | purple |
| LANDED | dim green |
| ABORT_ON_GROUND, ABORT_IN_FLIGHT | red, blinking at 2 Hz |

## Blink codes

| Pattern | Meaning |
|---------|---------|
| LED 1 blinks three times (100 ms on, 100 ms off), then holds | Entered a new state |
| LED 1 blinks red at 2 Hz | Abort state |
| LED 2 blinks red at 2 Hz | No healthy IMU or no healthy baro |
| All three blink red at 4 Hz | Error; overrides everything. Today: IMU init failed at boot |

Every blink counts from the last state entry, so it starts in the on half. In an
abort with failed sensors, LED 1 and LED 2 therefore flash red together. A state
change while the error override is on still starts its entry blink, hidden under
the override: if the error clears within 600 ms the rest of the blink shows, and
it is never replayed afterwards.

## Timing

The LED part is not named in the BOM, so the driver assumes WS2812B: 1.25 us
bit period, a 0 bit of 0.35 us high and 0.90 us low, a 1 bit of 0.65 us high
and 0.60 us low (inside both the classic WS2812B and the V5 windows), 300 us reset, GRB order, most
significant bit first. All of these are in `Timing` at the top of
`ws2812_bitbang.hpp`; retune them there for the real part. A frame of three LEDs
is 72 bits, 90 us with interrupts masked, and that is the whole cost of a
refresh: the 300 us reset gap is not waited out, because the 20 ms tick already
spaces frames far apart; a frame asked for within the gap is skipped. The strip
is only rewritten when a colour changes.
