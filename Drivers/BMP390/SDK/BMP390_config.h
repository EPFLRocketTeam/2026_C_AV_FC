#pragma once

// ── BMP390 SDK-based driver — build-time defaults ─────────────────────────────
// All values are BMP3_SensorAPI constants (bmp3_defs.h).
// Override any of these before including BMP390_SDK.hpp.

// Oversampling (BMP3_OVERSAMPLING_1X .. BMP3_OVERSAMPLING_32X)
#ifndef BMP390_DEFAULT_OSR_P
#define BMP390_DEFAULT_OSR_P BMP3_OVERSAMPLING_4X   // ~80 Hz capable
#endif

#ifndef BMP390_DEFAULT_OSR_T
#define BMP390_DEFAULT_OSR_T BMP3_OVERSAMPLING_1X
#endif

// IIR filter coefficient (BMP3_IIR_FILTER_DISABLE .. BMP3_IIR_FILTER_COEFF_127)
#ifndef BMP390_DEFAULT_IIR
#define BMP390_DEFAULT_IIR BMP3_IIR_FILTER_DISABLE  // no phase lag
#endif

// Command-mode rate limit (Hz). Prevents triggering faster than sensor can convert.
#ifndef BMP390_DEFAULT_COMMAND_RATE_HZ
#define BMP390_DEFAULT_COMMAND_RATE_HZ 50
#endif
