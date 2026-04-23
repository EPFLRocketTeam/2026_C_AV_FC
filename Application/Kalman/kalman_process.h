#ifndef APPLICATION_KALMAN_KALMAN_PROCESS_H
#define APPLICATION_KALMAN_KALMAN_PROCESS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int kalman_loop(void);
void kalman_note_main_loop_iteration_us(uint32_t iteration_us);

#ifdef __cplusplus
}
#endif

#endif
