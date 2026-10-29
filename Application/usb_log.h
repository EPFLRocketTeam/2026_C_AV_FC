/*    FILE: usb_log.h
 * PURPOSE: The HAL free rules behind _write's USB output (Core/Src/main.c):
 *          send only to a configured host, wait at most 5 ms on a busy CDC
 *          endpoint, then mark the port stalled and drop later writes until
 *          the host shows it is back, and report how many writes were
 *          dropped on the next one that goes out. The transmit call and the
 *          millisecond clock are passed in, so the host tests run it as is.
 */

#ifndef APP_USB_LOG_H
#define APP_USB_LOG_H

#include <stdint.h>
#include <stdio.h>

#define USB_LOG_BUSY_BUDGET_MS 5u

typedef struct {
  volatile uint8_t stalled; /* set by a write that ran out of budget */
  uint32_t dropped;         /* writes dropped since the last one sent */
  char notice[40];          /* stays valid while the notice is in flight */
} UsbLog;

/* Queues one buffer: returns 0 when queued, nonzero while the endpoint is
 * busy. The buffer must stay valid until the transfer completes. */
typedef int (*UsbLogTransmit)(const uint8_t *data, uint16_t length);
typedef uint32_t (*UsbLogTick)(void);

/* From the USB interrupt: a completed IN transfer, received bytes or a new
 * configuration show the host is back. */
static inline void usb_log_clear_stall(UsbLog *log) {
  log->stalled = 0;
}

/* Returns 1 when queued, 0 when the budget ran out (the log is then stalled). */
static inline int usb_log_send(UsbLog *log, const char *data, int length,
                               UsbLogTransmit transmit, UsbLogTick tick) {
  const uint32_t start = tick();
  while (transmit((const uint8_t *)data, (uint16_t)length) != 0) {
    if (tick() - start >= USB_LOG_BUSY_BUDGET_MS) {
      log->stalled = 1;
      return 0;
    }
  }
  return 1;
}

/* One _write. Output to an unconfigured port is discarded without counting:
 * there is no host to tell. */
static inline void usb_log_write(UsbLog *log, const char *data, int length, int configured,
                                 UsbLogTransmit transmit, UsbLogTick tick) {
  if (!configured) return;
  if (log->stalled) {
    log->dropped++;
    return;
  }
  if (log->dropped != 0) {
    const int n = snprintf(log->notice, sizeof log->notice, "[USB] dropped %lu lines\r\n",
                           (unsigned long)log->dropped);
    if (!usb_log_send(log, log->notice, n, transmit, tick)) {
      log->dropped++;
      return;
    }
    log->dropped = 0;
  }
  if (!usb_log_send(log, data, length, transmit, tick)) log->dropped++;
}

#endif /* APP_USB_LOG_H */
