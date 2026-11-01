#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace forward_command_controller_traced {
// Exactly one producer and one consumer. Never overwrite unread samples.
template<class T, size_t Capacity> class BoundedTelemetryQueue {
  static_assert(Capacity > 0);
  static_assert(std::is_trivially_copyable<T>::value);
  static_assert(std::atomic<size_t>::is_always_lock_free);
  std::array<T, Capacity+1> slots_{};
  alignas(64) std::atomic<size_t> head_{0};
  alignas(64) std::atomic<size_t> tail_{0};
public:
  bool push(const T& value) noexcept {
    const auto head=head_.load(std::memory_order_relaxed);
    const auto next=(head+1)%(Capacity+1);
    if (next==tail_.load(std::memory_order_acquire)) return false;
    slots_[head]=value;
    head_.store(next,std::memory_order_release);
    return true;
  }
  bool pop(T& value) noexcept {
    const auto tail=tail_.load(std::memory_order_relaxed);
    if (tail==head_.load(std::memory_order_acquire)) return false;
    value=slots_[tail];
    tail_.store((tail+1)%(Capacity+1),std::memory_order_release);
    return true;
  }
};
}
