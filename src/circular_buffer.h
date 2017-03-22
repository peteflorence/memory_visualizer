/**
 * Copyright 2016 Massachusetts Institute of Technology
 *
 * @file circular_buffer.h
 * @author W. Nicholas Greene
 * @date 2016-07-12 17:35:41 (Tue)
 */

#pragma once

#include <vector>

//#include "assert.h"

namespace nanomap {

/**
 * @brief Class to implement a simple circular buffer.
 *
 * https://en.wikipedia.org/wiki/Circular_buffer
 */
template <typename T>
class CircularBuffer final {
 public:
  // Convenience aliases.
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  /**
   * @brief Constructor.
   *
   * @param[in] capacity Maximum capacity of the buffer.
   */
  explicit CircularBuffer(uint32_t capacity) :
      start_(0),
      size_(0),
      capacity_(capacity),
      buffer_() {}
  ~CircularBuffer() = default;

  CircularBuffer(const CircularBuffer& rhs) = default;
  CircularBuffer& operator=(const CircularBuffer& rhs) = default;

  CircularBuffer(CircularBuffer&& rhs) = default;
  CircularBuffer& operator=(CircularBuffer&& rhs) = default;

  /**
   * @brief Random accessor.
   *
   * @param[in] idx Index relative to start.
   */
  T& operator[](size_t idx) {
    //NANOMAP_ASSERT(idx < size_);
    return buffer_[(start_ + idx) % capacity_];
  }

  const T& operator[](size_t idx) const {
    return operator[](idx);
  }

  /**
   * @brief Return the start index.
   */
  uint32_t start() const {
    return start_;
  }

  /**
   * @vruef Return the index of the last element.
   */
  uint32_t end() const {
    //NANOMAP_ASSERT(size_ > 0);
    return (start_ + size_ - 1) % capacity_;
  }

  /**
   * @brief Return the number of items in the buffer.
   */
  uint32_t size() const {
    return size_;
  }

  /**
   * @brief Return the capacity of the buffer.
   */
  uint32_t capacity() const {
    return capacity_;
  }

  /**
   * @brief Return a reference to the first element in the buffer.
   */
  const T& front() const {
    //NANOMAP_ASSERT(size_ > 0);
    return buffer_[start_];
  }
  T& front() {
    //NANOMAP_ASSERT(size_ > 0);
    return buffer_[start_];
  }

  /**
   * @brief Return a reference to the last element in the buffer.
   */
  const T& back() const {
    //NANOMAP_ASSERT(size_ > 0);
    return buffer_[end()];
  }
  T& back() {
    //NANOMAP_ASSERT(size_ > 0);
    return buffer_[end()];
  }

  /**
   * @brief Return a reference to the buffer itself.
   */
  const std::vector<T>& buffer() const {
    return buffer_;
  }

  /**
   * @brief Pop an item from the front of the buffer.
   */
  void pop_front() {
    //NANOMAP_ASSERT(size_ > 0);
    start_ = (start_ + 1) % capacity_;
    --size_;
    return;
  }

  /**
   * @brief Push an item to the end of the buffer.
   */
  void push_back(const T& new_item) {
    uint32_t new_idx = (start_ + size_) % capacity_;
    if (buffer_.size() <= new_idx) {
      buffer_.push_back(new_item);
    } else {
      buffer_[new_idx] = new_item; // Write value to new spot.
    }

    if (size_ == capacity_) {
      // We're at capacity and just overwrote an element. Increment the start
      // index.
      start_ = (start_ + 1) % capacity_;
    } else {
      ++size_;
    }

    return;
  }

 private:
  uint32_t start_; // Index to start of data.
  uint32_t size_; // Current number of elements in buffer.
  uint32_t capacity_; // Maximum buffer capacity.
  std::vector<T> buffer_; // Data!
};

}  // namespace nanomap
