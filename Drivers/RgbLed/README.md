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

## Timing

The LED part is not named in the BOM, so the driver assumes WS2812B: 1.25 us
bit period, 0.40 us and 0.80 us high times, 300 us reset, GRB order, most
significant bit first. All of these are in `Timing` at the top of
`ws2812_bitbang.hpp`; retune them there for the real part. A frame of three LEDs
is 72 bits, 90 us with interrupts masked, followed by the reset gap with
interrupts enabled. The strip is only rewritten when a colour changes.
