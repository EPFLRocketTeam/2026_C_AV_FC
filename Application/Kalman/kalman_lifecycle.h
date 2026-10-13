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

/// Clear the liftoff latch and the Kalman-owned EventStore flags
/// (apogee_detected, imu_liftoff_detected, vertical_acc_hold,
/// touchdown_detected, catastrophic_failure) without changing the published FSM state.
void kalman_lifecycle_rearm(void);

#ifdef __cplusplus
}
#endif

#endif
