#include <gtest/gtest.h>
#include <limits>
#include "forward_command_controller_traced/joint_test_trajectory.hpp"
using namespace forward_command_controller_traced;

static std::vector<JointTestBound> bounds() {return std::vector<JointTestBound>(3, {-1,1,.1,.2,.05});}

TEST(JointTrajectory, BoundaryPhasesLimitsAndStorage)
{
  JointTestTrajectory plan({.21,.21,.21}, {.2,.2,.2}, bounds(), 1, .02, 1, 4, 1, 5);
  std::vector<double> output(3);
  const auto* storage = output.data();
  EXPECT_EQ(plan.sample(0, output), JointTestPhase::transition);
  EXPECT_DOUBLE_EQ(output[0], .21);
  EXPECT_EQ(plan.sample(1, output), JointTestPhase::excitation);
  EXPECT_DOUBLE_EQ(output[1], .2);
  for (int tick=1000; tick<5000; ++tick) {
    plan.sample(tick/1000., output);
    EXPECT_EQ(storage, output.data());
    EXPECT_DOUBLE_EQ(output[0], .2);
    EXPECT_DOUBLE_EQ(output[2], .2);
    EXPECT_GE(output[1], .18);
    EXPECT_LE(output[1], .22);
  }
  EXPECT_EQ(plan.sample(5, output), JointTestPhase::hold);
  EXPECT_DOUBLE_EQ(output[1], .2);
  EXPECT_EQ(plan.sample(-1, output), JointTestPhase::invalid);
  EXPECT_EQ(plan.sample(std::numeric_limits<double>::quiet_NaN(), output), JointTestPhase::invalid);
  EXPECT_DOUBLE_EQ(output[1], .2);
}

TEST(JointTrajectory, RejectBoundsAndFastPlans)
{
  for (double amplitude : {0., .9, std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(JointTestTrajectory({.21,.21,.21}, {.2,.2,.2}, bounds(), 1, amplitude, 1, 4, 1, 5), std::invalid_argument);
  }
  EXPECT_THROW(JointTestTrajectory({.21,.21,.21}, {.2,.2,.2}, bounds(), 3, .02, 1, 4, 1, 5), std::invalid_argument);
  EXPECT_THROW(JointTestTrajectory({.21,.21,.21}, {.2,.2,.2}, bounds(), 1, .02, .01, 4, 1, 5), std::invalid_argument);
  EXPECT_THROW(JointTestTrajectory({.21,.21,.21}, {.2,.2,.2}, bounds(), 1, .02, 1, 4, 100, 5), std::invalid_argument);
}
