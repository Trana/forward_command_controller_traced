#include "forward_command_controller_traced/joint_sample_validity.hpp"
#include <cassert>
#include <limits>
using namespace forward_command_controller_traced;
int main() {
  constexpr uint64_t flags=1|4|16|32;
  constexpr uint64_t cached=256|flags, next=512|flags;
  assert(joint_sample_valid(cached,cached,1,1,10.01,10.,.15));
  // Deterministic regression: revision advanced before cache publication.
  assert(joint_sample_valid(next,cached,1,1,10.01,10.,.15));
  for (unsigned bit=0;bit<8;++bit)
    assert(!joint_sample_valid(next^(uint64_t{1}<<bit),cached,1,1,10.01,10.,.15));
  assert(!joint_sample_valid(next,cached,2,1,10.01,10.,.15));
  assert(!joint_sample_valid(cached,next,1,1,10.01,10.,.15));
  assert(!joint_sample_valid(next,cached,1,1,10.16,10.,.15));
  assert(!joint_sample_valid(next,cached,1,1,9.99,10.,.15));
  assert(!joint_sample_valid(next,cached,1,1,10.,0.,.15));
  assert(!joint_sample_valid(next,cached,1,1,10.,10.,0.));
  assert(!joint_sample_valid(next,cached,1,1,10.,10.,std::numeric_limits<double>::infinity()));
  assert(!joint_sample_valid(next,cached,1,1,std::numeric_limits<double>::quiet_NaN(),10.,.15));
  assert(!joint_sample_valid(next|8,next|8,1,1,10.01,10.,.15));
}
