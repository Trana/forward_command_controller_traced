#include "forward_command_controller_traced/bounded_telemetry_queue.hpp"
#include <cassert>
#include <cstdint>
#include <thread>
#include <iostream>
using namespace forward_command_controller_traced;
int main() {
  BoundedTelemetryQueue<uint64_t,3> small;
  uint64_t value=0;
  assert(!small.pop(value));
  assert(small.push(1) && small.push(2) && small.push(3));
  assert(!small.push(4)); // Full means reject, never overwrite.
  assert(small.pop(value) && value==1);
  assert(small.push(4));
  for (uint64_t expected=2;expected<=4;++expected) assert(small.pop(value) && value==expected);
  assert(!small.pop(value));
  BoundedTelemetryQueue<uint64_t,256> concurrent;
  constexpr uint64_t total=200000;
  std::thread producer([&]() {
    for (uint64_t i=1;i<=total;++i) while (!concurrent.push(i)) std::this_thread::yield();
  });
  for (uint64_t i=1;i<=total;++i) {
    while (!concurrent.pop(value)) std::this_thread::yield();
    assert(value==i);
  }
  producer.join();
  std::cout << "Bounded telemetry capacity, wraparound and concurrent FIFO passed\n";
}
