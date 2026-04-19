#ifndef APPLICATION_KALMAN_KALMAN_LIFECYCLE_H
#define APPLICATION_KALMAN_KALMAN_LIFECYCLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void kalman_on_liftoff(uint32_t liftoff_ms);
void kalman_on_state_change(uint32_t state);
uint32_t kalman_current_state(void);
uint8_t kalman_take_pending_liftoff(uint32_t *liftoff_ms_out);
void kalman_reset_lifecycle(void);

void kalman_note_wake_imu(void);
void kalman_note_wake_lifecycle(void);
void kalman_note_wake_timer(void);
void kalman_note_wake_backlog(void);

uint32_t kalman_wake_count_imu(void);
uint32_t kalman_wake_count_lifecycle(void);
uint32_t kalman_wake_count_timer(void);
uint32_t kalman_wake_count_backlog(void);

#ifdef __cplusplus
}
#endif

#endif
