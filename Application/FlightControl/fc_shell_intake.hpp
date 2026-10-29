//    FILE: fc_shell_intake.hpp
// PURPOSE: The HAL free byte intake behind FC's USB shell (fc_shell.cpp):
//          a single producer single consumer byte ring filled by the USB
//          CDC receive interrupt and drained by FC_Shell_Tick in the main
//          loop, and a line assembler that rebuilds lines across USB
//          packets. Header only so the host tests can use it as is.

#ifndef APP_FC_SHELL_INTAKE_H
#define APP_FC_SHELL_INTAKE_H

#include <atomic>
#include <cstdint>

namespace fc_shell {

// One producer (the USB interrupt) calls Push, one consumer (the main loop)
// calls Pop. Head and tail are free running counters; each side writes only
// its own, so no length is shared and no critical section is needed on a
// single core. The signal fences keep the compiler from moving buffer
// accesses across the index update. Bytes that do not fit are dropped.
template <uint32_t N>
class SpscRing {
  static_assert(N != 0 && (N & (N - 1)) == 0, "N must be a power of two");

 public:
  // Producer side. Returns how many bytes were stored.
  uint32_t Push(const uint8_t* data, uint32_t length) {
    const uint32_t head = head_;
    const uint32_t free_space = N - (head - tail_);
    const uint32_t count = length < free_space ? length : free_space;
    for (uint32_t i = 0; i < count; ++i) {
      buffer_[(head + i) & (N - 1)] = data[i];
    }
    std::atomic_signal_fence(std::memory_order_release);
    head_ = head + count;
    return count;
  }

  // Consumer side. Returns false when the ring is empty.
  bool Pop(uint8_t& byte) {
    const uint32_t tail = tail_;
    if (tail == head_) return false;
    std::atomic_signal_fence(std::memory_order_acquire);
    byte = buffer_[tail & (N - 1)];
    std::atomic_signal_fence(std::memory_order_release);
    tail_ = tail + 1;
    return true;
  }

 private:
  uint8_t buffer_[N] = {};
  volatile uint32_t head_ = 0;
  volatile uint32_t tail_ = 0;
};

// Rebuilds lines from a byte stream, whatever the packet boundaries. CR is
// skipped, LF ends a line, trailing spaces are trimmed (so "main lox open "
// still matches the %bool leaf's exact "open"). A line longer than N bytes
// is discarded whole at its LF rather than run truncated, since a cut
// command could fire with the wrong argument.
template <uint32_t N>
class LineAssembler {
 public:
  // Feeds one byte. Calls on_line(const char* line, uint32_t length) when a
  // complete line has arrived. Returns true in that case.
  template <typename OnLine>
  bool Feed(uint8_t byte, OnLine&& on_line) {
    if (byte == '\r') return false;
    if (byte != '\n') {
      if (length_ < N) {
        line_[length_++] = static_cast<char>(byte);
      } else {
        overflow_ = true;
      }
      return false;
    }
    const bool overflow = overflow_;
    uint32_t length = length_;
    length_ = 0;
    overflow_ = false;
    if (overflow) return false;
    while (length > 0 && line_[length - 1] == ' ') {
      --length;
    }
    on_line(static_cast<const char*>(line_), length);
    return true;
  }

 private:
  char line_[N] = {};
  uint32_t length_ = 0;
  bool overflow_ = false;
};

}  // namespace fc_shell

#endif  // APP_FC_SHELL_INTAKE_H
