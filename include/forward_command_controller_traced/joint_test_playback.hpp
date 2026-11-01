#pragma once
#include "forward_command_controller_traced/joint_test_trajectory.hpp"

namespace forward_command_controller_traced
{
struct JointTestFence {uint64_t activation, session;};
enum class JointTestStop {none, completed, cancelled, readiness, lease, clock, storage};

// Single-update-thread playback core; no ROS endpoints or automatic rearming.
// The adapter must supply authenticated, fresh permits and non-reused fences.
// Activation generation must be fenced by a unique controller boot identity at
// the transport boundary. Numeric IDs here are correlation, not authentication.
class JointTestPlayback
{
public:
  // Prepare storage outside the update loop. No output until begin() succeeds.
  JointTestPlayback(JointTestTrajectory trajectory, JointTestFence fence,
    double lease_seconds, double max_tick_gap)
  : trajectory_(std::move(trajectory)), fence_(fence),
    lease_seconds_(lease_seconds), max_tick_gap_(max_tick_gap)
  {
    if (!fence.activation || !fence.session || !positive(lease_seconds) || !positive(max_tick_gap)) {
      throw std::invalid_argument("Invalid bounded playback admission");
    }
  }

  JointTestPlayback(JointTestTrajectory trajectory, JointTestFence fence, double now,
    double lease_seconds, double max_tick_gap)
  : JointTestPlayback(std::move(trajectory),fence,lease_seconds,max_tick_gap)
  {
    if (!begin(fence,now)) {
      throw std::invalid_argument("Invalid bounded playback admission");
    }
  }

  // Single-use, no allocation. Adapter must first consume the start permit and
  // perform live-state checks in this same serialized update.
  bool begin(JointTestFence fence, double now) noexcept
  {return begin(fence,now,now);}

  bool begin(JointTestFence fence, double issued_at, double now) noexcept
  {
    if (!matches(fence) || begun_ || stopped()) {return false;}
    const double permit=issued_at+lease_seconds_, deadline=now+trajectory_.total_seconds();
    if (!std::isfinite(now) || now<0 || !std::isfinite(issued_at) || issued_at<0 || issued_at>now ||
        !std::isfinite(permit) || !std::isfinite(deadline) ||
        permit<=issued_at || deadline<=now) {stop_=JointTestStop::clock; return false;}
    if (now>=permit) {stop_=JointTestStop::lease; return false;}
    begun_=true;
    started_=last_tick_=last_renewal_=last_issued_=now;
    last_issued_=issued_at;
    permit_until_=permit;
    deadline_=deadline;
    return true;
  }

  bool renew(JointTestFence fence, double now)
  {return renew(fence,now,now);}

  // issued_at comes from the controller's one-use challenge, never a client clock.
  // Queue delay consumes permission lifetime. The adapter checks sequence/dedup.
  bool renew(JointTestFence fence, double issued_at, double now)
  {
    if (!matches(fence) || !begun_ || stopped()) {return false;}
    if (!std::isfinite(now) || now < last_renewal_ || now < last_tick_) {
      stop_ = JointTestStop::clock;
      return false;
    }
    if (now >= permit_until_) {stop_ = JointTestStop::lease; return false;}
    if (now >= deadline_) {stop_ = JointTestStop::completed; return false;}
    if (now-last_tick_>max_tick_gap_) {stop_=JointTestStop::clock; return false;}
    const double extension=issued_at+lease_seconds_;
    if (!std::isfinite(issued_at) || issued_at<=last_issued_ || issued_at>now ||
        !std::isfinite(extension) || extension<=now) {return false;}
    permit_until_ = extension;
    last_issued_ = issued_at;
    last_renewal_ = now;
    return true;
  }

  bool cancel(JointTestFence fence)
  {
    if (!matches(fence)) {return false;}
    if (!stopped()) {stop_ = JointTestStop::cancelled;}
    return true;
  }

  bool tick(double now, bool permitted, bool feedback_ready, std::vector<double>& output)
  {
    if (!begun_ || stopped()) {return false;}
    if (!std::isfinite(now) || now < last_tick_ || now < last_renewal_ ||
        now - last_tick_ > max_tick_gap_) {stop_ = JointTestStop::clock; return false;}
    if (!permitted || !feedback_ready) {stop_ = JointTestStop::readiness; return false;}
    if (now >= permit_until_) {stop_ = JointTestStop::lease; return false;}
    if (now >= deadline_) {stop_ = JointTestStop::completed; return false;}
    if (trajectory_.sample(now-started_, output) == JointTestPhase::invalid) {
      stop_ = JointTestStop::storage;
      return false;
    }
    last_tick_ = now;
    return true;
  }

  bool stopped() const {return stop_ != JointTestStop::none;}
  JointTestStop reason() const {return stop_;}
  double permit_until() const {return permit_until_;}

private:
  bool matches(JointTestFence other) const
  {return other.activation == fence_.activation && other.session == fence_.session;}
  static bool positive(double v) {return std::isfinite(v) && v > 0;}
  JointTestTrajectory trajectory_;
  JointTestFence fence_;
  double started_=0, last_tick_=0, last_renewal_=0, last_issued_=0,
    lease_seconds_, max_tick_gap_, permit_until_=0, deadline_=0;
  bool begun_=false;
  JointTestStop stop_ = JointTestStop::none;
};
}
