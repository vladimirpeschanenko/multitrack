#pragma once
#include <atomic>
#include <cstddef>
template <typename T, std::size_t Capacity> class alignas(64) SpscRing {
  T buf_[Capacity];
  alignas(64) std::atomic<std::size_t> head_{0};
  char _pad1[64 - sizeof(std::atomic<std::size_t>)]{};
  alignas(64) std::atomic<std::size_t> tail_{0};
  char _pad2[64 - sizeof(std::atomic<std::size_t>)]{};

public:
  inline bool try_push(const T &v) noexcept {
    const std::size_t h = head_.load(std::memory_order_relaxed);
    const std::size_t t = tail_.load(std::memory_order_acquire);
    const std::size_t nxt = (h + 1) % Capacity;
    if (nxt == t)
      return false;
    buf_[h] = v;
    head_.store(nxt, std::memory_order_release);
    return true;
  }
  inline bool try_pop(T &out) noexcept {
    const std::size_t t = tail_.load(std::memory_order_relaxed);
    const std::size_t h = head_.load(std::memory_order_acquire);
    if (t == h)
      return false;
    out = buf_[t];
    tail_.store((t + 1) % Capacity, std::memory_order_release);
    return true;
  }
  inline bool empty() const noexcept {
    return tail_.load(std::memory_order_acquire) ==
           head_.load(std::memory_order_acquire);
  }
};
