#pragma once
// IMU-based liftoff detector using dual rectangular moving windows.
//
// Two independent sliding windows run over the IMU acceleration magnitude:
//   - FAST window (5-10 ms):  catches the initial high-g transient.
//   - SLOW window (20-40 ms): confirms sustained thrust.
//
// Each window counts the fraction of samples whose *excess* acceleration
// (|a| - g) exceeds its threshold.  Liftoff is declared when BOTH windows
// simultaneously exceed the required fraction (default 70%).
//
// The detector is gated by a "pad stable" pre-condition derived from the
// Rail Shadow filter's Mahony gate.  Once the gate has been continuously
// open for a configurable period the detector becomes ARMED and stays
// armed—even after the gate closes during liftoff.
//
// Designed for ~6400 Hz IMU rate (ICM-45686 FIFO).

#include <cstdint>
#include <cmath>

class LiftoffDetector {
public:
    // ── Tuning constants (compile-time) ─────────────────────────────
    // At 6400 Hz, 10 ms → 64 samples, 40 ms → 256 samples.
    static constexpr uint16_t kFastWindowSamples  = 64;   // ~10 ms @ 6400 Hz
    static constexpr uint16_t kSlowWindowSamples  = 192;  // ~30 ms @ 6400 Hz
    static constexpr float    kFastThresholdMps2  = 2.0f * 9.81f;  // 2 g excess
    static constexpr float    kSlowThresholdMps2  = 1.0f * 9.81f;  // 1 g excess
    static constexpr float    kFractionRequired   = 0.70f;         // 70 %

    // Local gravity constant (m/s²).  Subtracted from |a| to get excess.
    static constexpr float    kGravity            = 9.81f;

    // Number of consecutive "gate-open" samples required to ARM.
    // At 6400 Hz, 2 seconds = 12800 samples.
    static constexpr uint32_t kArmGateOpenSamples = 12800;

    // ── API ─────────────────────────────────────────────────────────

    void reset() {
        armed_         = false;
        detected_      = false;
        gate_streak_   = 0;
        fast_head_     = 0;
        fast_count_    = 0;
        fast_above_    = 0;
        slow_head_     = 0;
        slow_count_    = 0;
        slow_above_    = 0;
    }

    /// Feed one IMU sample.
    /// @param accel_x  Body-X acceleration (m/s²)
    /// @param accel_y  Body-Y acceleration (m/s²)
    /// @param accel_z  Body-Z acceleration (m/s²)
    /// @param gate_open  True when the Rail Shadow Mahony gate is open (pad stable)
    void update(float accel_x, float accel_y, float accel_z, bool gate_open) {
        if (detected_) return;  // latched

        // ── Arm logic ───────────────────────────────────────────────
        if (!armed_) {
            if (gate_open) {
                if (++gate_streak_ >= kArmGateOpenSamples) {
                    armed_ = true;
                }
            } else {
                gate_streak_ = 0;
            }
            return;  // don't evaluate windows until armed
        }

        // ── Compute excess acceleration ─────────────────────────────
        const float amag = std::sqrt(accel_x * accel_x +
                                     accel_y * accel_y +
                                     accel_z * accel_z);
        const float excess = amag - kGravity;

        const bool fast_above = (excess > kFastThresholdMps2);
        const bool slow_above = (excess > kSlowThresholdMps2);

        // ── Push into fast window ───────────────────────────────────
        pushWindow(fast_buf_, kFastWindowSamples,
                   fast_head_, fast_count_, fast_above_, fast_above);

        // ── Push into slow window ───────────────────────────────────
        pushWindow(slow_buf_, kSlowWindowSamples,
                   slow_head_, slow_count_, slow_above_, slow_above);

        // ── Check trigger condition ─────────────────────────────────
        // Both windows must be fully populated AND exceed the fraction.
        if (fast_count_ >= kFastWindowSamples &&
            slow_count_ >= kSlowWindowSamples) {
            const float fast_frac = static_cast<float>(fast_above_) /
                                    static_cast<float>(fast_count_);
            const float slow_frac = static_cast<float>(slow_above_) /
                                    static_cast<float>(slow_count_);
            if (fast_frac >= kFractionRequired &&
                slow_frac >= kFractionRequired) {
                detected_ = true;
            }
        }
    }

    bool isArmed()    const { return armed_; }
    bool isDetected() const { return detected_; }

private:
    // ── Circular bit-buffer helpers ─────────────────────────────────
    // Store "above threshold?" as a packed bit array to save RAM.
    // 256 samples → 32 bytes per window.  Total = 64 bytes.
    static constexpr uint16_t kFastBufWords = (kFastWindowSamples + 31) / 32;
    static constexpr uint16_t kSlowBufWords = (kSlowWindowSamples + 31) / 32;

    static bool getBit(const uint32_t* buf, uint16_t idx) {
        return (buf[idx >> 5] >> (idx & 31)) & 1u;
    }

    static void setBit(uint32_t* buf, uint16_t idx, bool val) {
        const uint32_t mask = 1u << (idx & 31);
        if (val) buf[idx >> 5] |=  mask;
        else     buf[idx >> 5] &= ~mask;
    }

    static void pushWindow(uint32_t* buf, uint16_t capacity,
                            uint16_t& head, uint16_t& count,
                            uint16_t& above_count, bool above) {
        if (count == capacity) {
            // Evict oldest
            if (getBit(buf, head)) --above_count;
            head = (head + 1 < capacity) ? head + 1 : 0;
        } else {
            ++count;
        }
        // Write new sample at (head + count - 1) % capacity
        uint16_t write_idx = head + count - 1;
        if (write_idx >= capacity) write_idx -= capacity;
        setBit(buf, write_idx, above);
        if (above) ++above_count;
    }

    // ── State ───────────────────────────────────────────────────────
    bool     armed_       = false;
    bool     detected_    = false;
    uint32_t gate_streak_ = 0;

    // Fast window (bit-packed circular buffer)
    uint32_t fast_buf_[kFastBufWords] = {};
    uint16_t fast_head_  = 0;
    uint16_t fast_count_ = 0;
    uint16_t fast_above_ = 0;

    // Slow window (bit-packed circular buffer)
    uint32_t slow_buf_[kSlowBufWords] = {};
    uint16_t slow_head_  = 0;
    uint16_t slow_count_ = 0;
    uint16_t slow_above_ = 0;
};
