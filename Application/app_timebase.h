#ifndef APPLICATION_APP_TIMEBASE_H
#define APPLICATION_APP_TIMEBASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_timebase_init(void);
uint64_t app_timebase_now_us(void);
uint32_t app_timebase_now_ms(void);
void app_timebase_print_init_diag(void);

#define RUN_EVERY(interval_ms) \
    if (({ \
        static uint32_t _last_run = 0; \
        uint32_t _now = app_timebase_now_ms(); \
        bool _expired = (_now - _last_run >= (uint32_t)(interval_ms)); \
        if (_expired) { _last_run = _now; } \
        _expired; \
    }))

#ifdef __cplusplus
}
#endif

#endif
