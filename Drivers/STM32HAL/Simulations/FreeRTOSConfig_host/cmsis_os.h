/**
 * Host/POSIX implementation of the CMSIS-RTOS2 API subset used by this project.
 *
 * Maps the CMSIS-RTOS2 calls to native FreeRTOS API so we get real FreeRTOS
 * behaviour on Linux without the STM32-specific cmsis_os2.c wrapper (which
 * references SysTick, NVIC, IRQn_Type and makes 32-bit pointer assumptions).
 *
 * Only the functions actually used by application code are implemented.
 */
#ifndef CMSIS_OS_HOST_H
#define CMSIS_OS_HOST_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Types ---- */
typedef TaskHandle_t      osThreadId_t;
typedef SemaphoreHandle_t osMutexId_t;

/* ---- Constants ---- */
/* osWaitForever must stay uint32_t (CMSIS-RTOS2 API uses uint32_t for timeout).
 * We convert to portMAX_DELAY internally when passed to FreeRTOS. */
#define osWaitForever  0xFFFFFFFFU

/* Flags options (only used in osThreadFlagsWait — unused on host tests) */
#define osFlagsWaitAny  0x00000001U
#define osFlagsWaitAll  0x00000002U
#define osFlagsNoClear  0x00000004U

/* ---- Thread flags ---- */

/**
 * Set thread notification flags (maps to xTaskNotify with eSetBits).
 * Returns the flags that were set, or 0 on error.
 */
static inline uint32_t osThreadFlagsSet(osThreadId_t thread_id, uint32_t flags)
{
    if (thread_id == NULL) return 0;
    BaseType_t higher_woken = pdFALSE;
    xTaskNotifyFromISR(thread_id, flags, eSetBits, &higher_woken);
    portYIELD_FROM_ISR(higher_woken);
    return flags;
}

/**
 * Wait for thread notification flags (maps to xTaskNotifyWait).
 * Returns the flags that were received, or 0xFFFFFFFF on error.
 */
static inline uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    (void)options;
    TickType_t ticks = (timeout == osWaitForever) ? portMAX_DELAY : (TickType_t)timeout;
    uint32_t received = 0;
    if (xTaskNotifyWait(0, flags, &received, ticks) == pdTRUE)
        return received & flags;
    return 0xFFFFFFFFU; /* timeout / error */
}

/* ---- Mutex ---- */

/** Create a mutex. Returns NULL on failure. */
static inline osMutexId_t osMutexNew(const void *attr)
{
    (void)attr;
    return xSemaphoreCreateMutex();
}

/**
 * Acquire (lock) a mutex.
 * Returns 0 on success, non-zero on timeout/error.
 */
static inline uint32_t osMutexAcquire(osMutexId_t mutex_id, uint32_t timeout)
{
    if (mutex_id == NULL) return 1;
    TickType_t ticks = (timeout == osWaitForever) ? portMAX_DELAY : (TickType_t)timeout;
    return (xSemaphoreTake(mutex_id, ticks) == pdTRUE) ? 0 : 1;
}

/**
 * Release (unlock) a mutex.
 * Returns 0 on success, non-zero on error.
 */
static inline uint32_t osMutexRelease(osMutexId_t mutex_id)
{
    if (mutex_id == NULL) return 1;
    return (xSemaphoreGive(mutex_id) == pdTRUE) ? 0 : 1;
}

#ifdef __cplusplus
}
#endif

#endif /* CMSIS_OS_HOST_H */
