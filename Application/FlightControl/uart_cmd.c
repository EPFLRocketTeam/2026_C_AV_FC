#include "uart_cmd.h"
#include <string.h>
#include <stdio.h>

volatile bool g_uart_force_liftoff = false;

// Simple command parser for USB CDC input.
// Recognises:
//   "LIFTOFF\n" or "LIFTOFF\r\n"  — force immediate liftoff transition
//
// Called from interrupt context (CDC_Receive_HS), so keep it short.
void uart_cmd_process(const uint8_t* buf, uint32_t len) {
    if (len == 0 || buf == NULL) return;

    // Strip trailing CR/LF
    while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n')) {
        --len;
    }

    if (len == 7 && memcmp(buf, "LIFTOFF", 7) == 0) {
        g_uart_force_liftoff = true;
    }
}
