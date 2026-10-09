// Bounded wait-free FIFO between exactly one producer and one consumer thread.
// Every pushed element is delivered, in order (the servo commands: write() -> sender thread).
#pragma once

#include <atomic>
#include <cstddef>

namespace fr_client_library
{
template<typename T, std::size_t Capacity>
class SpscQueue
{
  static_assert(Capacity >= 2 && (Capacity & (Capacity - 1)) == 0, "capacity: power of two");

public:
  // Producer thread only. False if full (the element is not stored).
  bool push(const T & value)
  {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail - head_.load(std::memory_order_acquire) == Capacity) {
      return false;
    }
    slots_[tail & (Capacity - 1)] = value;
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  // Consumer thread only. False if empty.
  bool pop(T & out)
  {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head == tail_.load(std::memory_order_acquire)) {
      return false;
    }
    out = slots_[head & (Capacity - 1)];
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  // Approximate from either side (the other side may move meanwhile).
  std::size_t size() const
  {
    return tail_.load(std::memory_order_acquire) - head_.load(std::memory_order_acquire);
  }

private:
  T slots_[Capacity]{};
  alignas(64) std::atomic<std::size_t> head_{0};  // consumer
  alignas(64) std::atomic<std::size_t> tail_{0};  // producer
};

}  // namespace fr_client_library
