#ifndef APPLICATION_KALMAN_KALMAN_PROCESS_H
#define APPLICATION_KALMAN_KALMAN_PROCESS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int kalman_loop(void);
void kalman_note_main_loop_iteration_us(uint32_t iteration_us);
void kalman_note_baro_trigger(uint64_t trigger_us);

/// Retrieve ESKF IMU grouping statistics for metrics logging.
void kalman_get_group_stats(uint32_t* fire_count, uint32_t* solo_flush,
                            uint32_t* stale_flush);

#ifdef __cplusplus
}
#endif

#endif
