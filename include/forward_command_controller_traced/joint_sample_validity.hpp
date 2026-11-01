#pragma once
#include <cmath>
#include <cstdint>

namespace forward_command_controller_traced {
// Revisions can advance while the last complete sample remains valid.
// Lifecycle identity, all status flags and bounded age must still agree.
inline bool joint_sample_valid(uint64_t current, uint64_t cached,
  uint64_t activation, uint64_t cached_activation,
  double now, double sampled_at, double max_age)
{
  constexpr uint64_t flags = 255;
  return (current & 1) && (current & 4) && !(current & 8) &&
    (current & flags) == (cached & flags) &&
    (cached >> 8) <= (current >> 8) && activation == cached_activation &&
    std::isfinite(now) && std::isfinite(sampled_at) && std::isfinite(max_age) &&
    sampled_at > 0 && max_age > 0 && now >= sampled_at && now - sampled_at <= max_age;
}
}
