// Standalone numerical comparison helper; never loads ROS or hardware.
#include <iomanip>
#include <iostream>
#include "forward_command_controller_traced/joint_test_trajectory.hpp"
int main()
{
  using namespace forward_command_controller_traced;
  const std::vector<JointTestBound> bounds(3, {-1,1,.1,.2,.05});
  JointTestTrajectory plan({.21,.21,.21}, {.2,.2,.2}, bounds, 1, .02, 1, 4, 1, 5);
  std::vector<double> result(3);
  std::cout << std::setprecision(17);
  for (int tick=0; tick<=5000; ++tick) {
    plan.sample(tick/1000., result);
    std::cout << tick/1000. << ' ' << result[0] << ' ' << result[1] << ' ' << result[2] << '\n';
  }
}
