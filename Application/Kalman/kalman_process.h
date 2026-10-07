#ifndef APPLICATION_KALMAN_KALMAN_PROCESS_H
#define APPLICATION_KALMAN_KALMAN_PROCESS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int kalman_loop(void);

/// Request a full reset of the Kalman runtime, e.g. after an IMU anomaly
/// during FILLING, without rebooting the board or going through an abort.
///
/// Clears the estimator (preflight tares, rail/flight shadows, ground
/// references, turn-on bias, GNSS anchor), apogee hub, both liftoff
/// detectors, health counters, the liftoff latch and the Kalman-owned
/// EventStore flags (apogee_detected, imu_liftoff_detected,
/// vertical_acc_hold, catastrophic_failure). FSM state and timers are NOT
/// touched. The estimator then needs stationary time to reconverge, like
/// after boot (tens of seconds to ~2 minutes).
///
/// Safe from any context (only sets a flag); the reset is performed at the
/// start of the next kalman_loop(). Honoured only in INIT, CALIBRATION,
/// FILLING, ARMED, ABORT_ON_GROUND and LANDED, checked both here and when
/// the reset is executed.
///
/// @return 1 if the request was accepted, 0 if refused (not a ground state).
uint8_t kalman_request_reset(void);
void kalman_note_main_loop_iteration_us(uint32_t iteration_us);
void kalman_note_baro_trigger(uint64_t trigger_us);

/// Retrieve ESKF IMU grouping statistics for metrics logging.
void kalman_get_group_stats(uint32_t* fire_count, uint32_t* solo_flush,
                            uint32_t* stale_flush);

#ifdef __cplusplus
}
#endif

#endif
