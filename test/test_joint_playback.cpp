#include <gtest/gtest.h>
#include <limits>
#include "forward_command_controller_traced/joint_test_playback.hpp"
using namespace forward_command_controller_traced;

static JointTestPlayback make_playback()
{
  JointTestTrajectory trajectory({.21}, {.2}, {{-1,1,.1,.2,.05}}, 0, .02, 1, 4, 1, 5);
  return JointTestPlayback(std::move(trajectory), {1,2}, 10., .2, .05);
}

static JointTestPlayback make_prepared()
{
  return JointTestPlayback(JointTestTrajectory({.21},{.2},{{-1,1,.1,.2,.05}},0,.02,1,4,1,5),
    {1,2},.2,.05);
}

TEST(JointPlayback, PreparedPlaybackStartsClockOnceAtApplication)
{
  auto playback=make_prepared();
  std::vector<double> target{.9};
  EXPECT_FALSE(playback.tick(10.,true,true,target));
  EXPECT_DOUBLE_EQ(target[0],.9);
  EXPECT_FALSE(playback.renew({1,2},10.));
  EXPECT_FALSE(playback.begin({1,3},10.));
  ASSERT_TRUE(playback.begin({1,2},100.));
  ASSERT_TRUE(playback.tick(100.,true,true,target));
  EXPECT_DOUBLE_EQ(target[0],.21); // Starts at elapsed zero, not preparation time.
  EXPECT_FALSE(playback.begin({1,2},100.01));
  for (int i=1;i<20;++i) {ASSERT_TRUE(playback.tick(100.+i*.01,true,true,target));}
  EXPECT_FALSE(playback.tick(100.2,true,true,target));
  EXPECT_EQ(playback.reason(),JointTestStop::lease);
  EXPECT_FALSE(playback.begin({1,2},101.));
}

TEST(JointPlayback, PreparedCancellationAndInvalidStartCannotBeRevived)
{
  auto playback=make_prepared();
  ASSERT_TRUE(playback.cancel({1,2}));
  EXPECT_FALSE(playback.begin({1,2},10.));
  EXPECT_EQ(playback.reason(),JointTestStop::cancelled);
  for (double now : {-1.,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::max()}) {
    auto invalid=make_prepared();
    EXPECT_FALSE(invalid.begin({1,2},now));
    EXPECT_EQ(invalid.reason(),JointTestStop::clock);
    EXPECT_FALSE(invalid.begin({1,2},10.));
  }
}

TEST(JointPlayback, QueuedStartConsumesLeaseButBeginsTrajectoryAtElapsedZero)
{
  auto playback=make_prepared();
  ASSERT_TRUE(playback.begin({1,2},10.,10.1));
  std::vector<double> target{.9};
  ASSERT_TRUE(playback.tick(10.1,true,true,target));
  EXPECT_DOUBLE_EQ(target[0],.21);
  ASSERT_TRUE(playback.tick(10.14,true,true,target));
  ASSERT_TRUE(playback.tick(10.18,true,true,target));
  const auto prior=target;
  EXPECT_FALSE(playback.tick(10.201,true,true,target));
  EXPECT_EQ(playback.reason(),JointTestStop::lease);
  EXPECT_EQ(target,prior);

  auto expired=make_prepared();
  EXPECT_FALSE(expired.begin({1,2},10.,10.2));
  EXPECT_EQ(expired.reason(),JointTestStop::lease);
  EXPECT_FALSE(expired.begin({1,2},10.2,10.2));
  EXPECT_FALSE(expired.tick(10.21,true,true,target));
  EXPECT_EQ(target,prior);
}

TEST(JointPlayback, FencedCancellationIsLatchedAndDoesNotModifyTarget)
{
  auto playback = make_playback();
  std::vector<double> target{.21};
  ASSERT_TRUE(playback.tick(10.02, true, true, target));
  const auto prior = target;
  EXPECT_FALSE(playback.cancel({2,2}));
  EXPECT_FALSE(playback.cancel({1,3}));
  ASSERT_TRUE(playback.cancel({1,2}));
  EXPECT_TRUE(playback.cancel({1,2}));
  EXPECT_FALSE(playback.renew({1,2}, 10.03));
  EXPECT_FALSE(playback.tick(10.04, true, true, target));
  EXPECT_EQ(target, prior);
  EXPECT_EQ(playback.reason(), JointTestStop::cancelled);
}

TEST(JointPlayback, LeaseExpiresDespiteRegularTicksAndCannotBeRevived)
{
  auto playback = make_playback();
  std::vector<double> target{.21};
  EXPECT_FALSE(playback.renew({1,3}, 10.1));
  for (int i=1; i<20; ++i) {ASSERT_TRUE(playback.tick(10.+i*.01, true, true, target));}
  const auto prior = target;
  EXPECT_FALSE(playback.tick(10.2, true, true, target));
  EXPECT_EQ(playback.reason(), JointTestStop::lease);
  EXPECT_FALSE(playback.renew({1,2}, 10.21));
  EXPECT_EQ(target, prior);
}

TEST(JointPlayback, FreshPermitsDoNotExtendFixedDuration)
{
  auto playback = make_playback();
  std::vector<double> target{.21};
  for (int i=1; i<500; ++i) {
    const double now = 10.+i*.01;
    ASSERT_TRUE(playback.renew({1,2}, now));
    ASSERT_TRUE(playback.tick(now, true, true, target));
  }
  const auto prior = target;
  EXPECT_FALSE(playback.renew({1,2}, 15.));
  EXPECT_FALSE(playback.tick(15., true, true, target));
  EXPECT_EQ(playback.reason(), JointTestStop::completed);
  EXPECT_EQ(target, prior); // No jump-to-baseline command on completion.
}

TEST(JointPlayback, FaultsDoNotCatchUpOrResume)
{
  for (double now : {9., 10.1, std::numeric_limits<double>::quiet_NaN()}) {
    auto playback = make_playback();
    std::vector<double> target{.21};
    EXPECT_FALSE(playback.tick(now, true, true, target));
    EXPECT_EQ(playback.reason(), JointTestStop::clock);
    EXPECT_FALSE(playback.tick(10.01, true, true, target));
    EXPECT_DOUBLE_EQ(target[0], .21);
  }
  for (bool operator_ready : {false, true}) {
    auto playback = make_playback();
    std::vector<double> target{.21};
    EXPECT_FALSE(playback.tick(10.01, operator_ready, !operator_ready, target));
    EXPECT_EQ(playback.reason(), JointTestStop::readiness);
    EXPECT_FALSE(playback.tick(10.02, true, true, target));
  }
}

TEST(JointPlayback, QueuedRenewalUsesIssuanceTimeAndCannotReviveExpiredLease)
{
  auto playback=make_playback();
  std::vector<double> target{.21};
  for (int i=1;i<=15;++i) {ASSERT_TRUE(playback.tick(10.+i*.01,true,true,target));}
  ASSERT_TRUE(playback.renew({1,2},10.1,10.15));
  EXPECT_FALSE(playback.renew({1,2},10.1,10.16)); // Duplicate cannot extend again.
  EXPECT_FALSE(playback.renew({1,2},10.09,10.16));
  EXPECT_FALSE(playback.renew({1,2},10.2,10.16)); // Future issuance is invalid.
  for (int i=16;i<30;++i) {ASSERT_TRUE(playback.tick(10.+i*.01,true,true,target));}
  const auto prior=target;
  EXPECT_FALSE(playback.tick(10.301,true,true,target)); // Not application time + .2.
  EXPECT_EQ(playback.reason(),JointTestStop::lease);
  EXPECT_EQ(target,prior);
  EXPECT_FALSE(playback.renew({1,2},10.29,10.31));

  auto expired=make_playback();
  EXPECT_FALSE(expired.renew({1,2},10.19,10.21)); // Fresh challenge cannot revive old lease.
  EXPECT_EQ(expired.reason(),JointTestStop::lease);
  auto missed_tick=make_playback();
  EXPECT_FALSE(missed_tick.renew({1,2},10.05,10.1));
  EXPECT_EQ(missed_tick.reason(),JointTestStop::clock);
}
