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
// its own variables, so no length is shared and no critical section is
// needed on a single core. The signal fences keep the compiler from moving
// buffer accesses across the index updates.
//
// A full ring never splices two lines together. When Push has to drop
// bytes it records a gap at the position of the next stored byte and, unless
// the last dropped byte was a newline, also drops every later byte up to and
// including the next newline, so the bytes stored after the gap start a
// fresh line. Pop reports the gap so the consumer can throw away the partial
// line stored before it. If several gaps happen before the consumer reaches
// the first, everything from the first gap to the last one is skipped.
template <uint32_t N>
class SpscRing {
  static_assert(N != 0 && (N & (N - 1)) == 0, "N must be a power of two");

 public:
  enum class PopResult : uint8_t {
    kEmpty,     // nothing to read
    kByte,      // byte is valid, feed it
    kAfterGap,  // byte is valid and starts a line: drop the partial line first
    kSkipped,   // byte lies inside a run of gaps: drop it and the partial line
  };

  // Producer side. Returns how many bytes of data were stored.
  uint32_t Push(const uint8_t* data, uint32_t length) {
    uint32_t i = 0;
    while (discarding_ && i < length) {
      if (data[i++] == '\n') discarding_ = false;
    }
    const uint32_t head = head_;
    const uint32_t free_space = N - (head - tail_);
    const uint32_t wanted = length - i;
    const uint32_t count = wanted < free_space ? wanted : free_space;
    for (uint32_t k = 0; k < count; ++k) {
      buffer_[(head + k) & (N - 1)] = data[i + k];
    }
    if (count < wanted) {
      discarding_ = data[length - 1] != '\n';
      const uint32_t seq = gap_seq_;
      if (seq == gap_ack_) gap_first_ = head + count;
      gap_last_ = head + count;
      std::atomic_signal_fence(std::memory_order_release);
      gap_seq_ = seq + 1;
    }
    std::atomic_signal_fence(std::memory_order_release);
    head_ = head + count;
    return count;
  }

  // Consumer side.
  PopResult Pop(uint8_t& byte) {
    const uint32_t tail = tail_;
    if (tail == head_) return PopResult::kEmpty;
    std::atomic_signal_fence(std::memory_order_acquire);
    PopResult result = PopResult::kByte;
    const uint32_t seq = gap_seq_;
    if (seq != gap_ack_) {
      std::atomic_signal_fence(std::memory_order_acquire);
      const uint32_t first = gap_first_;
      const uint32_t last = gap_last_;
      if (Before(tail, last) && !Before(tail, first)) {
        result = PopResult::kSkipped;
      } else if (tail == last) {
        result = PopResult::kAfterGap;
        gap_ack_ = seq;
      }
    }
    byte = buffer_[tail & (N - 1)];
    std::atomic_signal_fence(std::memory_order_release);
    tail_ = tail + 1;
    return result;
  }

 private:
  // a comes before b on the free running counter.
  static bool Before(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b) < 0;
  }

  uint8_t buffer_[N] = {};
  volatile uint32_t head_ = 0;       // producer
  volatile uint32_t tail_ = 0;       // consumer
  volatile uint32_t gap_first_ = 0;  // producer
  volatile uint32_t gap_last_ = 0;   // producer
  volatile uint32_t gap_seq_ = 0;    // producer
  volatile uint32_t gap_ack_ = 0;    // consumer
  bool discarding_ = false;          // producer
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
    Discard();
    if (overflow) return false;
    while (length > 0 && line_[length - 1] == ' ') {
      --length;
    }
    on_line(static_cast<const char*>(line_), length);
    return true;
  }

  // Forgets the partial line received so far.
  void Discard() {
    length_ = 0;
    overflow_ = false;
  }

 private:
  char line_[N] = {};
  uint32_t length_ = 0;
  bool overflow_ = false;
};

// The ring and the assembler as FC_Shell_Tick uses them: Receive from the
// USB interrupt, Drain from the main loop. Drain returns how many bytes it
// read, skipped ones included.
template <uint32_t RingSize, uint32_t LineSize>
class Intake {
 public:
  uint32_t Receive(const uint8_t* data, uint32_t length) {
    return ring_.Push(data, length);
  }

  template <typename OnLine>
  uint32_t Drain(OnLine&& on_line) {
    uint32_t read = 0;
    uint8_t byte;
    for (;;) {
      const auto result = ring_.Pop(byte);
      if (result == SpscRing<RingSize>::PopResult::kEmpty) return read;
      ++read;
      if (result != SpscRing<RingSize>::PopResult::kByte) line_.Discard();
      if (result == SpscRing<RingSize>::PopResult::kSkipped) continue;
      line_.Feed(byte, on_line);
    }
  }

 private:
  SpscRing<RingSize> ring_;
  LineAssembler<LineSize> line_;
};

}  // namespace fc_shell

#endif  // APP_FC_SHELL_INTAKE_H
