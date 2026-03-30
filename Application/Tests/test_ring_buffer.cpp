#include "Application/Data/ring_buffer.hpp"

#include <gtest/gtest.h>

// =============================================================================
// Helpers
// =============================================================================

struct Point {
    int x{0};
    int y{0};
    bool operator==(const Point &o) const { return x == o.x && y == o.y; }
};

// =============================================================================
// Single-element interface
// =============================================================================

TEST(RingBufferSingle, DefaultStateIsEmpty) {
    RingBuffer<int, 4> buf;
    EXPECT_TRUE(buf.empty());
    EXPECT_FALSE(buf.full());
    EXPECT_EQ(buf.size(), 0u);
}

TEST(RingBufferSingle, AppendAndGetPartialFill) {
    RingBuffer<int, 4> buf;
    buf.append(10);
    buf.append(20);
    buf.append(30);

    EXPECT_EQ(buf.size(), 3u);
    EXPECT_FALSE(buf.empty());
    EXPECT_FALSE(buf.full());

    EXPECT_EQ(*buf.get(0), 10);  // oldest
    EXPECT_EQ(*buf.get(1), 20);
    EXPECT_EQ(*buf.get(2), 30);  // newest
}

TEST(RingBufferSingle, FullTriggersAtCapacity) {
    RingBuffer<int, 3> buf;
    buf.append(1);
    buf.append(2);
    buf.append(3);

    EXPECT_TRUE(buf.full());
    EXPECT_EQ(buf.size(), 3u);
}

TEST(RingBufferSingle, OverwriteDropsOldestKeepsNewest) {
    RingBuffer<int, 3> buf;
    buf.append(1);
    buf.append(2);
    buf.append(3);
    buf.append(4);  // overwrites 1

    EXPECT_EQ(buf.size(), 3u);
    EXPECT_EQ(*buf.get(0), 2);  // oldest is now 2
    EXPECT_EQ(*buf.get(1), 3);
    EXPECT_EQ(*buf.get(2), 4);  // newest
}

TEST(RingBufferSingle, GetZeroAlwaysReturnsOldestAfterWrapAround) {
    RingBuffer<int, 3> buf;
    for (int i = 1; i <= 7; ++i) {
        buf.append(i);
    }
    // After 7 appends into size-3 buffer: holds 5, 6, 7
    EXPECT_EQ(*buf.get(0), 5);
    EXPECT_EQ(*buf.get(1), 6);
    EXPECT_EQ(*buf.get(2), 7);
}

TEST(RingBufferSingle, WorksWithFloats) {
    RingBuffer<float, 4> buf;
    buf.append(1.5f);
    buf.append(2.5f);

    EXPECT_FLOAT_EQ(*buf.get(0), 1.5f);
    EXPECT_FLOAT_EQ(*buf.get(1), 2.5f);
}

TEST(RingBufferSingle, WorksWithNonTrivialStruct) {
    RingBuffer<Point, 4> buf;
    buf.append({1, 2});
    buf.append({3, 4});

    EXPECT_EQ(*buf.get(0), (Point{1, 2}));
    EXPECT_EQ(*buf.get(1), (Point{3, 4}));
}

TEST(RingBufferSingle, SizeOneEdgeCase) {
    RingBuffer<int, 1> buf;

    EXPECT_TRUE(buf.empty());

    buf.append(42);
    EXPECT_TRUE(buf.full());
    EXPECT_EQ(*buf.get(0), 42);

    buf.append(99);  // overwrites 42
    EXPECT_EQ(buf.size(), 1u);
    EXPECT_EQ(*buf.get(0), 99);
}

TEST(RingBufferSingle, OutOfBoundsReturnsNullptr) {
    RingBuffer<int, 4> buf;
    EXPECT_EQ(buf.get(0), nullptr);  // empty buffer

    buf.append(1);
    EXPECT_EQ(buf.get(1), nullptr);  // only index 0 is valid
    EXPECT_NE(buf.get(0), nullptr);
}

// =============================================================================
// Bulk append interface
// =============================================================================

TEST(RingBufferBulkAppend, FillsEmptyBufferCorrectly) {
    RingBuffer<int, 4> buf;
    const int data[] = {10, 20, 30};
    buf.append(data, 3);

    EXPECT_EQ(buf.size(), 3u);
    EXPECT_EQ(*buf.get(0), 10);
    EXPECT_EQ(*buf.get(1), 20);
    EXPECT_EQ(*buf.get(2), 30);
}

TEST(RingBufferBulkAppend, PartialFillDoesNotOverflow) {
    RingBuffer<int, 8> buf;
    const int data[] = {1, 2, 3};
    buf.append(data, 3);

    EXPECT_EQ(buf.size(), 3u);
    EXPECT_FALSE(buf.full());
}

TEST(RingBufferBulkAppend, CausesWrapAroundOverwrite) {
    RingBuffer<int, 3> buf;
    buf.append(1);
    buf.append(2);
    buf.append(3);  // full: [1, 2, 3]

    const int extra[] = {4, 5};
    buf.append(extra, 2);  // overwrites 1 then 2 → [3, 4, 5]

    EXPECT_EQ(buf.size(), 3u);
    EXPECT_EQ(*buf.get(0), 3);
    EXPECT_EQ(*buf.get(1), 4);
    EXPECT_EQ(*buf.get(2), 5);
}

TEST(RingBufferBulkAppend, MoreThanCapacityKeepsLastN) {
    RingBuffer<int, 3> buf;
    const int data[] = {1, 2, 3, 4, 5, 6, 7};
    buf.append(data, 7);

    // Only the last 3 elements survive
    EXPECT_EQ(buf.size(), 3u);
    EXPECT_EQ(*buf.get(0), 5);
    EXPECT_EQ(*buf.get(1), 6);
    EXPECT_EQ(*buf.get(2), 7);
}

TEST(RingBufferBulkAppend, ZeroLengthIsNoOp) {
    RingBuffer<int, 4> buf;
    buf.append(1);
    const int data[] = {99};
    buf.append(data, 0);

    EXPECT_EQ(buf.size(), 1u);
    EXPECT_EQ(*buf.get(0), 1);
}

// =============================================================================
// Bulk get interface
// =============================================================================

TEST(RingBufferBulkGet, RetrievesNOldestInOrder) {
    RingBuffer<int, 5> buf;
    for (int i = 1; i <= 5; ++i) buf.append(i);

    int out[3] = {};
    const size_t written = buf.get(out, 3);

    EXPECT_EQ(written, 3u);
    EXPECT_EQ(out[0], 1);
    EXPECT_EQ(out[1], 2);
    EXPECT_EQ(out[2], 3);
}

TEST(RingBufferBulkGet, ClampsToSizeWhenNIsTooLarge) {
    RingBuffer<int, 5> buf;
    buf.append(10);
    buf.append(20);

    int out[10] = {};
    const size_t written = buf.get(out, 10);

    EXPECT_EQ(written, 2u);
    EXPECT_EQ(out[0], 10);
    EXPECT_EQ(out[1], 20);
}

TEST(RingBufferBulkGet, ZeroLengthReturnsZero) {
    RingBuffer<int, 4> buf;
    buf.append(1);
    buf.append(2);

    int out[4] = {};
    const size_t written = buf.get(out, 0);

    EXPECT_EQ(written, 0u);
}

TEST(RingBufferBulkGet, EmptyBufferReturnsZero) {
    RingBuffer<int, 4> buf;
    int out[4] = {};
    EXPECT_EQ(buf.get(out, 4), 0u);
}

TEST(RingBufferBulkGet, CorrectAfterWrapAround) {
    RingBuffer<int, 3> buf;
    for (int i = 1; i <= 5; ++i) buf.append(i);
    // Holds: 3, 4, 5

    int out[3] = {};
    buf.get(out, 3);

    EXPECT_EQ(out[0], 3);
    EXPECT_EQ(out[1], 4);
    EXPECT_EQ(out[2], 5);
}

// =============================================================================
// Round-trip: bulk append → bulk get
// =============================================================================

TEST(RingBufferRoundTrip, BulkAppendThenBulkGet) {
    RingBuffer<int, 8> buf;
    const int src[] = {10, 20, 30, 40, 50};
    buf.append(src, 5);

    int dst[5] = {};
    const size_t n = buf.get(dst, 5);

    EXPECT_EQ(n, 5u);
    for (size_t i = 0; i < n; ++i) {
        EXPECT_EQ(dst[i], src[i]);
    }
}

TEST(RingBufferRoundTrip, BulkAppendWithOverwriteThenBulkGet) {
    RingBuffer<int, 4> buf;
    const int src[] = {1, 2, 3, 4, 5, 6};  // 6 elements into size-4 buffer
    buf.append(src, 6);                      // holds 3, 4, 5, 6

    int dst[4] = {};
    buf.get(dst, 4);

    EXPECT_EQ(dst[0], 3);
    EXPECT_EQ(dst[1], 4);
    EXPECT_EQ(dst[2], 5);
    EXPECT_EQ(dst[3], 6);
}
