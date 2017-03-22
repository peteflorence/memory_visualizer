// Copyright 2016 Massachusetts Institute of Technology

#include "gtest/gtest.h"

#include "depthest/utils/circular_buffer.h"

namespace depthest {

namespace utils {

/**
 * @brief Test setting up the buffer.
 */
TEST(CircularBufferTest, ConstructorTest) {
  uint32_t capacity = 5;
  CircularBuffer<int> buffer(capacity);
  EXPECT_EQ(0, buffer.start());
  EXPECT_EQ(0, buffer.size());
  EXPECT_EQ(capacity, buffer.capacity());
}

/**
 * @brief Test pushing items into the buffer and make sure that the max capacity
 * is respected.
 */
TEST(CircularBufferTest, PushBackTest) {
  uint32_t capacity = 5;
  CircularBuffer<int> buffer(capacity);

  // Push some items into the buffer.
  buffer.push_back(1);
  buffer.push_back(2);
  buffer.push_back(3);

  EXPECT_EQ(3, buffer.size());
  EXPECT_EQ(0, buffer.start());
  EXPECT_EQ(1, buffer.front());
  EXPECT_EQ(3, buffer.back());

  // Add some more items.
  buffer.push_back(4);
  buffer.push_back(5);

  // Now test going past capacity.
  buffer.push_back(6);
  EXPECT_EQ(capacity, buffer.size());

  // Make sure the first element is correct.
  EXPECT_EQ(1, buffer.start());
  EXPECT_EQ(2, buffer.front());

  // Make sure the last element is correct.
  EXPECT_EQ(6, buffer.back());

  // Now do it a couple more times to be sure.
  buffer.push_back(7);
  buffer.push_back(8);
  buffer.push_back(9);
  buffer.push_back(10);

  EXPECT_EQ(capacity, buffer.size());
  EXPECT_EQ(0, buffer.start());
  EXPECT_EQ(6, buffer.front());
  EXPECT_EQ(10, buffer.back());

  return;
}

/**
 * @brief Test popping items off the buffer.
 */
TEST(CircularBufferTest, PopFrontTest) {
  uint32_t capacity = 5;
  CircularBuffer<int> buffer(capacity);

  // Test pushing and popping a single item.
  buffer.push_back(1);
  buffer.pop_front();
  EXPECT_EQ(0, buffer.size());
  EXPECT_EQ(1, buffer.start());

  // Now test with multiple items.
  buffer.push_back(2);
  buffer.push_back(3);
  buffer.push_back(4);

  EXPECT_EQ(3, buffer.size());
  EXPECT_EQ(1, buffer.start());

  buffer.pop_front();
  EXPECT_EQ(2, buffer.size());
  EXPECT_EQ(2, buffer.start());
  EXPECT_EQ(3, buffer.front());
  EXPECT_EQ(4, buffer.back());

  buffer.pop_front();
  EXPECT_EQ(1, buffer.size());
  EXPECT_EQ(3, buffer.start());
  EXPECT_EQ(4, buffer.front());
  EXPECT_EQ(4, buffer.back());

  buffer.pop_front();
  EXPECT_EQ(0, buffer.size());
  EXPECT_EQ(4, buffer.start());

  // Now go beyond capacity and test popping.
  buffer.push_back(1); // start = 4
  buffer.push_back(2);
  buffer.push_back(3);
  buffer.push_back(4);
  buffer.push_back(5);
  buffer.push_back(6); // start = 0
  buffer.push_back(7); // start = 1
  buffer.push_back(8); // start = 2
  buffer.pop_front(); // start = 3
  buffer.pop_front(); // start = 4
  buffer.pop_front(); // start = 0

  EXPECT_EQ(0, buffer.start());
  EXPECT_EQ(2, buffer.size());
  EXPECT_EQ(7, buffer.front());
  EXPECT_EQ(8, buffer.back());

  return;
}

}  // namespace utils

}  // namespace depthest
