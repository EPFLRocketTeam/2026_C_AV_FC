//    FILE: fc_shell.hpp
// PURPOSE: A tiny command shell over FC's USB VCP: buffers incoming bytes
//          into a fixed-size line buffer, and once a complete line
//          (terminated by \n) has arrived, replays it character-by-
//          character through the generated command trie (see the CLI
//          submodule's fc_commands_generated.hpp/push_char). Called from
//          CDC_Receive_HS, same as uart_cmd_process (see uart_cmd.c) --
//          keep individual command handlers short, this runs in
//          interrupt context.

#ifndef APP_FC_SHELL_H
#define APP_FC_SHELL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Feeds raw bytes from a USB CDC receive event. Call once per
// CDC_Receive_HS call, passing exactly what it received -- a single call
// may contain a partial line, a full line, or multiple lines; the shell
// buffers/reassembles across calls as needed.
void Fc_Shell_ProcessRxBytes(const uint8_t *data, uint32_t length);
void FC_Shell_Tick();

#ifdef __cplusplus
}
#endif

#endif // APP_FC_SHELL_H
