#pragma once
#include <cstddef>
#include <cstdint>

namespace app {
// Re-evaluate after every discard: an internal acquisition gap may jump a
// front PAST the previous alignment target. One alignment per loop (or one
// sweep per group) can silently feed arbitrarily mismatched timestamps.
// Work is bounded by the number of samples present; every unsuccessful
// sweep either discards a sample or finds an empty healthy source.
template <size_t N, typename Ring>
bool alignImuFronts(Ring *(&rings)[N], const bool (&healthy)[N],
                    uint32_t &discarded, uint64_t tolerance_us = 200) {
  for (;;) {
    uint64_t newest = 0;
    bool any = false;
    for (size_t i = 0; i < N; ++i) {
      if (!healthy[i]) continue;
      any = true;
      const auto *front = rings[i]->get(0);
      if (!front) return false;
      if (front->timestamp_us > newest) newest = front->timestamp_us;
    }
    if (!any) return false;
    bool changed = false;
    const uint64_t floor = newest > tolerance_us ? newest - tolerance_us : 0;
    for (size_t i = 0; i < N; ++i) {
      if (!healthy[i]) continue;
      while (const auto *front = rings[i]->get(0)) {
        if (front->timestamp_us >= floor) break;
        auto sample = *front;
        rings[i]->pop(sample);
        ++discarded;
        changed = true;
      }
    }
    if (!changed) return true;
  }
}
}  // namespace app
