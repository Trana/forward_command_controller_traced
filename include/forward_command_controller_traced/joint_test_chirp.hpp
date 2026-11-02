#pragma once
#include "forward_command_controller_traced/joint_test_trajectory.hpp"

namespace forward_command_controller_traced
{
// Offline preparation primitive only. No transport, ownership or motion permit.
// Linear frequency sweep with a Hann amplitude envelope; constant peak amplitude.
class JointTestChirp
{
public:
  JointTestChirp(std::vector<double> previous, std::vector<double> baseline,
    const std::vector<JointTestBound>& bounds, size_t selected, double amplitude,
    double transition, double duration, double start_hz, double end_hz, double max_total)
  : trajectory_(std::move(previous), std::move(baseline), bounds, selected, amplitude,
      transition, duration, 1, max_total, start_hz, end_hz)
  {
    if (start_hz <= 0) {
      throw std::invalid_argument("Invalid chirp frequencies");
    }
  }

  JointTestPhase sample(double elapsed, std::vector<double>& output) const
  {
    return trajectory_.sample(elapsed, output);
  }

private:
  JointTestTrajectory trajectory_;
};
}
