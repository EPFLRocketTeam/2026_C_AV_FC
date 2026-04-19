#ifndef APPLICATION_APP_TIMEBASE_H
#define APPLICATION_APP_TIMEBASE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_timebase_init(void);
uint64_t app_timebase_now_us(void);
uint32_t app_timebase_now_ms(void);

#ifdef __cplusplus
}
#endif

#endif
