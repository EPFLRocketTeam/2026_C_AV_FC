//    FILE: fc_shell_replay.hpp
// PURPOSE: Hands one assembled shell line to the generated command trie
//          (fc_commands_generated.hpp): every character, then the '\n' that
//          fires the leaf. HAL free, shared by FC_Shell_Tick and the host
//          tests.

#ifndef APP_FC_SHELL_REPLAY_H
#define APP_FC_SHELL_REPLAY_H

#include <cstdint>

#include "fc_commands_generated.hpp"

namespace fc_shell {

inline void ReplayLine(context* ctx, const driver* drv, const char* line, uint32_t length) {
  for (uint32_t i = 0; i < length; i++) {
    push_char(ctx, drv, line[i]);
  }
  push_char(ctx, drv, '\n');
}

}  // namespace fc_shell

#endif  // APP_FC_SHELL_REPLAY_H
