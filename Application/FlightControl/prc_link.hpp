//    FILE: prc_link.hpp
// PURPOSE: FC side of the CAN connectivity matrix. The tracking itself
//          (alive state, rows, lost beats, reboots) is
//          prc_intranet::link::LinkMatrix, in the intranet's link_matrix.hpp,
//          shared by the 4 boards. This file only holds what is specific to
//          the FC: the 3-bit prc_alive field of the radio downlink, and the
//          console view of the matrix. Pure and HAL free, so it is host
//          tested (Tests/test_prc_link.cpp, Tests/test_downlink_prc_alive.cpp).
//
// Matrix layout: row i at bits 4i..4i+3, bit j of row i = "board i hears
// board j", boards 0 FC, 1 Engine, 2 Eth, 3 LOX (constants::LINK_BIT_*). A
// silent board's row is 0.

#ifndef APP_FC_PRC_LINK_HPP
#define APP_FC_PRC_LINK_HPP

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "prc_intranet/const.hpp"
#include "prc_intranet/link_matrix.hpp"
#include "Drivers/ERT_RF_Protocol_Interface/PacketDefinition_Firehorn2.h"

namespace fc_link {

namespace pc = prc_intranet::constants;

constexpr uint8_t BOARD_COUNT = prc_intranet::link::BOARD_COUNT;

// Console names, indexed like the matrix.
inline const char* board_name(uint8_t board) {
  static const char* const kNames[BOARD_COUNT] = {"FC", "ENGINE", "ETH", "LOX"};
  return board < BOARD_COUNT ? kNames[board] : "?";
}

// Row `row` of the matrix.
inline uint8_t matrix_row(uint16_t matrix, uint8_t row) {
  return static_cast<uint8_t>((matrix >> (4u * row)) & 0x0Fu);
}

// The downlink prc_alive field, with the same meaning as before the matrix:
// AV_PRC_ALIVE_ENGINE / _ETH / _LOX set when the FC hears that board (a
// heartbeat less than 500 ms ago). Taken from the FC's own row; the FC's own
// bit and the rows of the PRC boards are not sent.
inline uint8_t prc_alive_from_matrix(uint16_t matrix) {
  const uint8_t fc_row = matrix_row(matrix, pc::LINK_BIT_FC);
  uint8_t alive = 0;
  if (fc_row & (1u << pc::LINK_BIT_ENGINE)) alive |= AV_PRC_ALIVE_ENGINE;
  if (fc_row & (1u << pc::LINK_BIT_ETH))    alive |= AV_PRC_ALIVE_ETH;
  if (fc_row & (1u << pc::LINK_BIT_LOX))    alive |= AV_PRC_ALIVE_LOX;
  return alive;
}

// Number of lines of the console view: a header, then one line per board.
constexpr uint8_t MATRIX_LINES = 1 + BOARD_COUNT;

// Writes line `line` (0 .. MATRIX_LINES - 1) of the console view into
// `out`, NUL terminated. Rows are the board that hears, columns the board
// heard. "X" = hears, "." = does not hear, "-" = the row's board is silent
// (row 0, nothing received from it). Returns false for a line out of range.
//
//               FC ENGINE    ETH    LOX
//   FC           X      X      X      .
//   ENGINE       X      X      X      .
//   ETH          X      X      X      .
//   LOX          -      -      -      -   silent
inline bool format_matrix_line(uint16_t matrix, uint8_t line, char* out, size_t size) {
  if (line >= MATRIX_LINES || out == nullptr || size == 0) return false;

  if (line == 0) {
    std::snprintf(out, size, "%-8s%7s%7s%7s%7s", "",
                  board_name(0), board_name(1), board_name(2), board_name(3));
    return true;
  }

  const uint8_t board = static_cast<uint8_t>(line - 1);
  const uint8_t row   = matrix_row(matrix, board);
  int n = std::snprintf(out, size, "%-8s", board_name(board));
  for (uint8_t col = 0; col < BOARD_COUNT && n >= 0 && static_cast<size_t>(n) < size; col++) {
    const char* cell = (row == 0) ? "-" : ((row & (1u << col)) ? "X" : ".");
    n += std::snprintf(out + n, size - static_cast<size_t>(n), "%7s", cell);
  }
  if (row == 0 && n >= 0 && static_cast<size_t>(n) < size) {
    std::snprintf(out + n, size - static_cast<size_t>(n), "   silent");
  }
  return true;
}

} // namespace fc_link

#endif // APP_FC_PRC_LINK_HPP
