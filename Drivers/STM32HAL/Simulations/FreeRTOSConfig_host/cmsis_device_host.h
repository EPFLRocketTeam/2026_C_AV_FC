/**
 * Minimal CMSIS device header stub for host/POSIX builds.
 *
 * freertos_os2.h includes CMSIS_device_header for IRQ priority intrinsics
 * (__get_IPSR, __get_BASEPRI, etc.) that are ARM-specific. On a POSIX host
 * those paths are never taken, so we only need the bare minimum to compile.
 */
#ifndef CMSIS_DEVICE_HOST_H
#define CMSIS_DEVICE_HOST_H

#include <stdint.h>

/* ARM NVIC priority intrinsics — no-ops on POSIX */
static inline uint32_t __get_IPSR(void)     { return 0; }
static inline uint32_t __get_BASEPRI(void)  { return 0; }
static inline void     __set_BASEPRI(uint32_t v) { (void)v; }

/* __disable_irq / __enable_irq — no-ops on POSIX */
static inline void __disable_irq(void) {}
static inline void __enable_irq(void)  {}

#endif /* CMSIS_DEVICE_HOST_H */
