#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Per-section timing of the super loop, for bench runs.
 *
 *   uint64_t t0 = app_perf_begin();
 *   ...section...
 *   app_perf_end(APP_PERF_IMU, t0);
 *
 * app_perf_loop_mark() at the top of each iteration measures the whole
 * iteration, including what Core/Src/main.c runs around
 * app_super_loop_iterate() (radio tick, CAN drain). app_perf_print() prints
 * the maxima since the previous print and resets them.
 *
 * Loop stalls are counted against the IMU FIFO depth: one ICM-45686 at
 * 6.4 kHz fills 2 KB in ~16 ms and 8 KB in ~64 ms; a longer stall loses
 * frames.
 */
#ifndef APP_PERF_TRACE
#define APP_PERF_TRACE 1
#endif

#ifndef APP_PERF_REPORT_MS
#define APP_PERF_REPORT_MS 1000u
#endif

typedef enum {
	APP_PERF_SHELL,     /* FC_Shell_Tick */
	APP_PERF_PERIPH,    /* config, cameras, temperature, buzzer, batteries */
	APP_PERF_SD_LOG,    /* SD logging block and its status prints */
	APP_PERF_SD_TICK,   /* g_sd_interface.tick() (both drain points) */
	APP_PERF_IMU,       /* imuModule.update and the IMU status prints */
	APP_PERF_BARO_GPS,  /* baroModule.update, gpsModule.update */
	APP_PERF_KALMAN,    /* kalman_loop */
	APP_PERF_FSM,       /* fsm_tick */
	APP_PERF_RADIO,     /* simple_radio_tick */
	APP_PERF_CAN,       /* CAN RX drain in Core/Src/main.c */
	APP_PERF_REPORT,    /* the perf/IMU/radio report itself */
	APP_PERF_SECTION_COUNT
} app_perf_section_t;

#if APP_PERF_TRACE

uint64_t app_perf_begin(void);
void app_perf_end(app_perf_section_t section, uint64_t t0);
void app_perf_loop_mark(void);
void app_perf_print(void);

#else

static inline uint64_t app_perf_begin(void) { return 0u; }
static inline void app_perf_end(app_perf_section_t section, uint64_t t0) {
	(void) section; (void) t0;
}
static inline void app_perf_loop_mark(void) {}
static inline void app_perf_print(void) {}

#endif

#ifdef __cplusplus
}
#endif
