#pragma once

#include <gtest/gtest.h>

// Minimal Unity assertion compatibility
#define TEST_ASSERT_TRUE(condition) EXPECT_TRUE((condition))
#define TEST_ASSERT_FALSE(condition) EXPECT_FALSE((condition))

#define TEST_ASSERT_EQUAL(expected, actual) EXPECT_EQ((expected), (actual))
#define TEST_ASSERT_NOT_EQUAL(expected, actual) EXPECT_NE((expected), (actual))

#define TEST_ASSERT_EQUAL_UINT16(expected, actual) \
  EXPECT_EQ(static_cast<uint16_t>(expected), static_cast<uint16_t>(actual))
#define TEST_ASSERT_EQUAL_UINT32(expected, actual) \
  EXPECT_EQ(static_cast<uint32_t>(expected), static_cast<uint32_t>(actual))
#define TEST_ASSERT_EQUAL_UINT64(expected, actual) \
  EXPECT_EQ(static_cast<uint64_t>(expected), static_cast<uint64_t>(actual))
#define TEST_ASSERT_EQUAL_UINT8(expected, actual) \
  EXPECT_EQ(static_cast<uint8_t>(expected), static_cast<uint8_t>(actual))

#define TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual) \
  EXPECT_NEAR(static_cast<double>(expected), static_cast<double>(actual), \
              static_cast<double>(delta))
#define TEST_ASSERT_DOUBLE_WITHIN(delta, expected, actual) \
  EXPECT_NEAR(static_cast<double>(expected), static_cast<double>(actual), \
              static_cast<double>(delta))

#define TEST_ASSERT_NULL(ptr) EXPECT_EQ((ptr), nullptr)
#define TEST_ASSERT_NOT_NULL(ptr) EXPECT_NE((ptr), nullptr)

#define TEST_IGNORE_MESSAGE(message) GTEST_SKIP() << (message)
