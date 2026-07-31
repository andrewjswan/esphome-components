#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstring>

namespace esphome::music_leds {

#define RING_ATOMIC_SIZE_T std::atomic<size_t>
#define RING_LOAD(x) (x).load(std::memory_order_acquire)
#define RING_STORE(x, v) (x).store((v), std::memory_order_release)

/// Lock-free single-producer single-consumer ring buffer.
/// Producer calls write(), consumer calls peek()/advance()/read().
/// Thread safety is provided by atomic operations on size_.
template<typename T, size_t N> class RingBuffer {
 public:
  size_t write(const T *data, size_t count) {
    size_t sz = RING_LOAD(size_);
    size_t space = N - sz;
    size_t to_write = std::min(count, space);
    if (to_write == 0)
      return 0;

    // Split at wraparound boundary for memcpy optimization
    size_t first = std::min(to_write, N - write_pos_);
    std::memcpy(&buf_[write_pos_], data, first * sizeof(T));
    if (to_write > first) {
      std::memcpy(&buf_[0], data + first, (to_write - first) * sizeof(T));
    }
    write_pos_ = (write_pos_ + to_write) % N;
    RING_STORE(size_, sz + to_write);
    return to_write;
  }

  size_t write_overwrite(const T *data, size_t count) {
    if (count == 0)
      return 0;

    // If incoming batch is larger than total capacity, take only the last N elements
    if (count > N) {
      data += (count - N);
      count = N;
    }

    size_t sz = RING_LOAD(size_);
    size_t space = N - sz;

    // If there isn't enough space, forcefully advance the read pointer
    // to evict the exact amount of oldest samples before copying the new block
    if (count > space) {
      size_t to_evict = count - space;
      read_pos_ = (read_pos_ + to_evict) % N;
      sz -= to_evict;
    }

    // Fast block memory copy optimized at wraparound boundaries
    size_t first = std::min(count, N - write_pos_);
    std::memcpy(&buf_[write_pos_], data, first * sizeof(T));
    if (count > first) {
      std::memcpy(&buf_[0], data + first, (count - first) * sizeof(T));
    }

    write_pos_ = (write_pos_ + count) % N;
    RING_STORE(size_, sz + count);
    return count;
  }

  size_t read(T *out, size_t count) {
    size_t sz = RING_LOAD(size_);
    size_t to_read = std::min(count, sz);
    if (to_read == 0)
      return 0;

    // Split at wraparound boundary for memcpy optimization
    size_t first = std::min(to_read, N - read_pos_);
    std::memcpy(out, &buf_[read_pos_], first * sizeof(T));
    if (to_read > first) {
      std::memcpy(out + first, &buf_[0], (to_read - first) * sizeof(T));
    }
    read_pos_ = (read_pos_ + to_read) % N;
    RING_STORE(size_, sz - to_read);
    return to_read;
  }

  size_t peek(T *out, size_t count) const {
    size_t sz = RING_LOAD(size_);
    size_t to_read = std::min(count, sz);
    if (to_read == 0)
      return 0;

    // Split at wraparound boundary for memcpy optimization
    size_t first = std::min(to_read, N - read_pos_);
    std::memcpy(out, &buf_[read_pos_], first * sizeof(T));
    if (to_read > first) {
      std::memcpy(out + first, &buf_[0], (to_read - first) * sizeof(T));
    }
    return to_read;
  }

  void advance(size_t count) {
    size_t sz = RING_LOAD(size_);
    size_t to_advance = std::min(count, sz);
    read_pos_ = (read_pos_ + to_advance) % N;
    RING_STORE(size_, sz - to_advance);
  }

  size_t available() const { return RING_LOAD(size_); }
  size_t capacity() const { return N; }
  void clear() {
    read_pos_ = 0;
    write_pos_ = 0;
    RING_STORE(size_, static_cast<size_t>(0));
  }

 private:
  T buf_[N]{};
  size_t read_pos_{0};
  size_t write_pos_{0};
  RING_ATOMIC_SIZE_T size_{0};
};

}  // namespace esphome::music_leds
