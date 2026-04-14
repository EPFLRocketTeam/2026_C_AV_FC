#include "Application/Kalman/kalman_lifecycle.h"

#include <atomic>

namespace {
std::atomic<uint32_t> g_kalman_state{0};
std::atomic<uint32_t> g_liftoff_ms{0};
std::atomic<bool> g_liftoff_pending{false};
std::atomic<bool> g_liftoff_latched{false};
}

extern "C" void kalman_on_liftoff(uint32_t liftoff_ms) {
    bool expected = false;
    if (!g_liftoff_latched.compare_exchange_strong(expected, true,
                                                   std::memory_order_acq_rel,
                                                   std::memory_order_relaxed)) {
        return;
    }

    g_liftoff_ms.store(liftoff_ms, std::memory_order_relaxed);
    g_liftoff_pending.store(true, std::memory_order_release);
}

extern "C" void kalman_on_state_change(uint32_t state) {
    g_kalman_state.store(state, std::memory_order_relaxed);

    // INIT re-arms lifecycle state for a fresh flight sequence.
    if (state == 0U) {
        g_liftoff_pending.store(false, std::memory_order_relaxed);
        g_liftoff_latched.store(false, std::memory_order_relaxed);
        g_liftoff_ms.store(0, std::memory_order_relaxed);
    }
}

extern "C" uint32_t kalman_current_state(void) {
    return g_kalman_state.load(std::memory_order_relaxed);
}

extern "C" uint8_t kalman_take_pending_liftoff(uint32_t *liftoff_ms_out) {
    bool expected = true;
    if (!g_liftoff_pending.compare_exchange_strong(expected, false,
                                                   std::memory_order_acq_rel,
                                                   std::memory_order_relaxed)) {
        return 0;
    }

    if (liftoff_ms_out != nullptr) {
        *liftoff_ms_out = g_liftoff_ms.load(std::memory_order_acquire);
    }
    return 1;
}

extern "C" void kalman_reset_lifecycle(void) {
    g_kalman_state.store(0, std::memory_order_relaxed);
    g_liftoff_ms.store(0, std::memory_order_relaxed);
    g_liftoff_pending.store(false, std::memory_order_relaxed);
    g_liftoff_latched.store(false, std::memory_order_relaxed);
}
