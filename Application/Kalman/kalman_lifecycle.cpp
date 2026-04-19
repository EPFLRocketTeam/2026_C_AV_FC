#include "Application/Kalman/kalman_lifecycle.h"
#include "Application/Data/data.hpp"

#include "cmsis_os.h"

#include <atomic>

extern osMutexId_t eventStoreMutexHandle;

namespace {
std::atomic<uint32_t> g_kalman_state{0};
std::atomic<uint32_t> g_liftoff_ms{0};
std::atomic<bool> g_liftoff_pending{false};
std::atomic<bool> g_liftoff_latched{false};
std::atomic<uint32_t> g_wake_imu{0};
std::atomic<uint32_t> g_wake_lifecycle{0};
std::atomic<uint32_t> g_wake_timer{0};
std::atomic<uint32_t> g_wake_backlog{0};

constexpr uint32_t kKalmanThreadWakeFlag = 0x0001U;

#if !defined(UNIT_TEST_ENV)
extern osThreadId_t kalmanTaskHandle;
#endif

void wakeKalmanTask() {
#if !defined(UNIT_TEST_ENV)
    if (kalmanTaskHandle != nullptr) {
        osThreadFlagsSet(kalmanTaskHandle, kKalmanThreadWakeFlag);
    }
#endif
}

void resetWakeCounters() {
    g_wake_imu.store(0, std::memory_order_relaxed);
    g_wake_lifecycle.store(0, std::memory_order_relaxed);
    g_wake_timer.store(0, std::memory_order_relaxed);
    g_wake_backlog.store(0, std::memory_order_relaxed);
}
}

extern "C" void kalman_note_wake_imu(void) {
    g_wake_imu.fetch_add(1u, std::memory_order_relaxed);
}

extern "C" void kalman_note_wake_lifecycle(void) {
    g_wake_lifecycle.fetch_add(1u, std::memory_order_relaxed);
}

extern "C" void kalman_note_wake_timer(void) {
    g_wake_timer.fetch_add(1u, std::memory_order_relaxed);
}

extern "C" void kalman_note_wake_backlog(void) {
    g_wake_backlog.fetch_add(1u, std::memory_order_relaxed);
}

extern "C" uint32_t kalman_wake_count_imu(void) {
    return g_wake_imu.load(std::memory_order_relaxed);
}

extern "C" uint32_t kalman_wake_count_lifecycle(void) {
    return g_wake_lifecycle.load(std::memory_order_relaxed);
}

extern "C" uint32_t kalman_wake_count_timer(void) {
    return g_wake_timer.load(std::memory_order_relaxed);
}

extern "C" uint32_t kalman_wake_count_backlog(void) {
    return g_wake_backlog.load(std::memory_order_relaxed);
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
    kalman_note_wake_lifecycle();
    wakeKalmanTask();
}

extern "C" void kalman_on_state_change(uint32_t state) {
    const uint32_t previous_state =
        g_kalman_state.exchange(state, std::memory_order_relaxed);

    // INIT re-arms lifecycle state for a fresh flight sequence.
    if (state == 0U) {
        g_liftoff_pending.store(false, std::memory_order_relaxed);
        g_liftoff_latched.store(false, std::memory_order_relaxed);
        g_liftoff_ms.store(0, std::memory_order_relaxed);
        resetWakeCounters();

        auto &goat = flight_computer::GOATStore::get_instance();
        if (eventStoreMutexHandle != nullptr) {
            osMutexAcquire(eventStoreMutexHandle, osWaitForever);
        }

        auto event = goat.eventStore.get();
        event.catastrophic_failure = false;
        event.apogee_detected = false;
        goat.eventStore.set(event);

        if (eventStoreMutexHandle != nullptr) {
            osMutexRelease(eventStoreMutexHandle);
        }
    }

    if (previous_state == state) {
        return;
    }

    kalman_note_wake_lifecycle();
    wakeKalmanTask();
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
    resetWakeCounters();
}
