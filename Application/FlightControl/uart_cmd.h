#ifndef UART_CMD_H
#define UART_CMD_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Set to true by the USB CDC receive handler when a "LIFTOFF" command
/// is received.  Consumed (cleared) by the FSM on the next tick.
extern volatile bool g_uart_force_liftoff;

/// Called from CDC_Receive_HS when USB data arrives.
/// Parses for known commands and sets the appropriate flags.
void uart_cmd_process(const uint8_t* buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* UART_CMD_H */
