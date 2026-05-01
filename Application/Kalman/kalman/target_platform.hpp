#pragma once

// Platform target fallbacks for builds that do not provide APP_TARGET_* macros.
// ERT defaults to STM32 unless APP_TARGET_NATIVE is explicitly enabled.
#ifndef APP_TARGET_NATIVE
#define APP_TARGET_NATIVE 0
#endif

#ifndef APP_TARGET_STM32
#define APP_TARGET_STM32 1
#endif

#if APP_TARGET_NATIVE
#undef APP_TARGET_STM32
#define APP_TARGET_STM32 0
#endif
