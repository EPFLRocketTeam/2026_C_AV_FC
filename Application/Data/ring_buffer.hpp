#pragma once

#include <cstddef>

/**
 * @brief Generic compile-time-sized ring buffer.
 *
 * Holds at most N elements of type T. When the buffer is full, appending a
 * new element silently overwrites the oldest one.
 *
 * @tparam T  Element type.
 * @tparam N  Capacity (must be > 0).
 */
template <typename T, size_t N>
class RingBuffer {
    static_assert(N > 0, "RingBuffer size must be greater than 0");

  public:
    // -------------------------------------------------------------------------
    // Single-element interface
    // -------------------------------------------------------------------------

    /**
     * @brief Append one value.
     *
     * If the buffer is full the oldest element is overwritten.
     */
    void append(const T &value) {
        const size_t write_pos = (head_ + size_) % N;
        buffer_[write_pos]     = value;
        if (size_ == N) {
            head_ = (head_ + 1) % N;  // discard oldest
        } else {
            ++size_;
        }
    }

    /**
     * @brief Return pointer to element at logical index (0 = oldest, size()-1 = newest).
     *
     * Returns nullptr if index is out of bounds.
     */
    const T *get(size_t index) const {
        if (index >= size_) {
            return nullptr;
        }
        return &buffer_[(head_ + index) % N];
    }

    // -------------------------------------------------------------------------
    // Bulk interface
    // -------------------------------------------------------------------------

    /**
     * @brief Append n elements from data[0..n-1], oldest-first.
     *
     * Each element follows the same overwrite-on-full rule. If n > N only the
     * last N elements end up in the buffer.
     */
    void append(const T *data, size_t n) {
        for (size_t i = 0; i < n; ++i) {
            append(data[i]);
        }
    }

    /**
     * @brief Copy the n oldest elements into out[0..n-1].
     *
     * n is clamped to size() when larger.
     *
     * @return Number of elements actually written to out.
     */
    size_t get(T *out, size_t n) const {
        const size_t count = (n < size_) ? n : size_;
        for (size_t i = 0; i < count; ++i) {
            out[i] = buffer_[(head_ + i) % N];
        }
        return count;
    }

    // -------------------------------------------------------------------------
    // Capacity helpers
    // -------------------------------------------------------------------------

    /** @brief Number of valid elements currently held. */
    size_t size() const { return size_; }

    /** @brief True when no elements are stored. */
    bool empty() const { return size_ == 0; }

    /** @brief True when the buffer has reached its capacity. */
    bool full() const { return size_ == N; }

  private:
    T      buffer_[N]{};
    size_t head_ = 0;  ///< Physical index of the oldest element.
    size_t size_ = 0;  ///< Number of valid elements.
};
