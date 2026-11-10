//    FILE: prc_link.hpp
// PURPOSE: Tracks the heartbeat of each PRC board (engine, ethanol, LOX) as
//          received over CAN. Pure and HAL free: the caller passes the
//          receive time, so it is host tested (Tests/test_prc_link.cpp).

#ifndef APP_FC_PRC_LINK_HPP
#define APP_FC_PRC_LINK_HPP

#include <cstdint>

class PrcLinkMonitor {
public:
  enum class Board : uint8_t { Engine = 0, Eth = 1, Lox = 2 };
  static constexpr uint8_t BOARD_COUNT = 3;
  static constexpr uint32_t TIMEOUT_MS = 500;
  static constexpr uint32_t BOOT_GRACE_MS = 2000;

  struct Link {
    uint32_t last_counter = 0;
    uint32_t last_rx_ms   = 0;
    uint32_t received     = 0;
    uint32_t missed       = 0; // beats skipped, from counter jumps
    bool     seen         = false;
  };

  // A forward jump of n counts n - 1 missed beats; the unsigned difference
  // keeps the 2^32 wrap a step of 1. A backward step (difference of 2^31 or
  // more) is a reboot, counter reset to 0, and counts nothing.
  void onHeartbeat(Board board, uint32_t counter, uint32_t now_ms) {
    Link& l = links_[index(board)];
    if (l.seen) {
      const uint32_t step = counter - l.last_counter;
      if (step > 1 && step < 0x80000000u) l.missed += step - 1;
    }
    l.last_counter = counter;
    l.last_rx_ms   = now_ms;
    l.received++;
    l.seen = true;
  }

  // True when a beat arrived less than TIMEOUT_MS before now_ms.
  bool alive(Board board, uint32_t now_ms) const {
    const Link& l = links_[index(board)];
    return l.seen && (now_ms - l.last_rx_ms) < TIMEOUT_MS;
  }

  // Bit 0 engine, bit 1 ethanol, bit 2 LOX.
  uint8_t aliveMask(uint32_t now_ms) const {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < BOARD_COUNT; i++) {
      if (alive(static_cast<Board>(i), now_ms)) mask |= static_cast<uint8_t>(1u << i);
    }
    return mask;
  }

  const Link& link(Board board) const { return links_[index(board)]; }

  // Bits whose state should be printed now (same order as aliveMask): every
  // alive or dead edge, plus one "dead" for each board never heard from
  // BOOT_GRACE_MS after the first call. The grace counts from that first
  // call, not from reset, so a long setup with the boards already sending
  // (frames not yet drained) is not reported dead.
  uint8_t takeReport(uint32_t now_ms) {
    if (!started_) {
      started_  = true;
      first_ms_ = now_ms;
    }
    const uint8_t mask = aliveMask(now_ms);
    uint8_t report = mask ^ reported_mask_;
    reported_mask_ = mask;
    if (!grace_done_ && (now_ms - first_ms_) >= BOOT_GRACE_MS) {
      grace_done_ = true;
      for (uint8_t i = 0; i < BOARD_COUNT; i++) {
        if (!links_[i].seen) report |= static_cast<uint8_t>(1u << i);
      }
    }
    return report;
  }

private:
  static uint8_t index(Board board) { return static_cast<uint8_t>(board); }

  Link    links_[BOARD_COUNT];
  uint8_t reported_mask_ = 0;
  bool    grace_done_    = false;
  bool    started_       = false;
  uint32_t first_ms_     = 0;
};

#endif // APP_FC_PRC_LINK_HPP
