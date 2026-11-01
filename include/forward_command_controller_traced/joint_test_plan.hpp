#pragma once
#include "forward_command_controller_traced/joint_test_profile.hpp"

namespace forward_command_controller_traced
{
// The controller adapter must copy these fields coherently from one update.
// All times are same-host steady-clock seconds, never client timestamps.
struct JointTestSnapshot
{
  std::string instance;
  uint64_t activation, revision;
  double sampled_at;
  bool hold_applied, tracking_fault, feedback_fault;
  std::vector<std::string> joints;
  std::vector<double> commands, positions, receipts;
  double max_feedback_age=0.;
  uint64_t cancelled_session=0, applied_sequence=0;
};

enum class JointTestApplyCheck {ready, stale_controller, expired_plan, invalid_state,
  not_holding, stale_feedback, changed_command, tracking_error};

// Non-realtime preparation only. Does not acquire ownership or start playback.
// The adapter must recheck current fences/readiness and target continuity at apply.
class PreparedJointTestPlan
{
public:
  template<class Request>
  PreparedJointTestPlan(const Request& request, LoadedJointTestProfile profile,
    JointTestSnapshot snapshot, double now, double max_feedback_age)
  : profile_(std::move(profile)), snapshot_(std::move(snapshot)),
    specification_(validate_joint_test_prepare(request,profile_.value(),
      snapshot_.instance,snapshot_.activation)),
    trajectory_(make_trajectory(profile_.value(),snapshot_,specification_,now,max_feedback_age)),
    prepared_at_(now), max_feedback_age_(max_feedback_age) {}

  const LoadedJointTestProfile& profile() const {return profile_;}
  const JointTestSnapshot& snapshot() const {return snapshot_;}
  const JointTestTrajectory& trajectory() const {return trajectory_;}

  // Allocation-free check against controller-owned live state. Call in the same
  // serialized update that applies the plan, not from an earlier service callback.
  // Passing does not consume a start permit or make this plan one-use.
  JointTestApplyCheck check_apply(const JointTestSnapshot& state, double now) const noexcept
  {
    using Check=JointTestApplyCheck;
    if (state.instance!=snapshot_.instance || state.activation!=snapshot_.activation ||
        state.revision<snapshot_.revision) {return Check::stale_controller;}
    if (!std::isfinite(now) || now<prepared_at_ || now-prepared_at_>max_feedback_age_) {
      return Check::expired_plan;
    }
    if (state.joints!=profile_.value().joints || state.commands.size()!=snapshot_.commands.size() ||
        state.positions.size()!=snapshot_.positions.size() || state.receipts.size()!=snapshot_.receipts.size()) {
      return Check::invalid_state;
    }
    if (!state.hold_applied || state.tracking_fault || state.feedback_fault) {return Check::not_holding;}
    if (!std::isfinite(state.sampled_at) || state.sampled_at<snapshot_.sampled_at ||
        state.sampled_at>now || now-state.sampled_at>max_feedback_age_) {return Check::stale_feedback;}
    for (size_t i=0; i<state.commands.size(); ++i) {
      const double receipt=state.receipts[i], position=state.positions[i];
      const auto& bound=profile_.value().bounds[i];
      if (!std::isfinite(receipt) || receipt<=0 || receipt>state.sampled_at ||
          receipt<snapshot_.receipts[i] || now-receipt>max_feedback_age_) {return Check::stale_feedback;}
      // Exact equality is intentional: this controller must still hold the same target.
      if (state.commands[i]!=snapshot_.commands[i]) {return Check::changed_command;}
      if (!std::isfinite(position) || position<bound.lower || position>bound.upper ||
          std::abs(position-state.commands[i])>bound.handover_error ||
          std::abs(position-snapshot_.positions[i])>bound.handover_error) {return Check::tracking_error;}
    }
    return Check::ready;
  }

private:
  static JointTestTrajectory make_trajectory(const JointTestProfile& profile,
    const JointTestSnapshot& state, const PreparedJointTest& spec,
    double now, double max_age)
  {
    const auto reject=[](const char* code) {throw JointTestAdmissionError(code);};
    if (!std::isfinite(now) || now<=0 || !std::isfinite(max_age) || max_age<=0) {
      reject("INVALID_PREPARATION_CLOCK");
    }
    if (!state.revision || !state.hold_applied || state.tracking_fault || state.feedback_fault) {
      reject("HOLD_NOT_READY");
    }
    if (state.joints!=profile.joints || state.commands.size()!=profile.joints.size() ||
        state.positions.size()!=profile.joints.size() || state.receipts.size()!=profile.joints.size()) {
      reject("INVALID_SNAPSHOT");
    }
    if (!std::isfinite(state.sampled_at) || state.sampled_at<=0 || state.sampled_at>now ||
        now-state.sampled_at>max_age) {reject("STALE_SNAPSHOT");}
    for (double receipt : state.receipts) {
      if (!std::isfinite(receipt) || receipt<=0 || receipt>state.sampled_at || now-receipt>max_age) {
        reject("STALE_FEEDBACK");
      }
    }
    for (size_t i=0; i<state.positions.size(); ++i) {
      const auto& bound=profile.bounds[i];
      const double position=state.positions[i], command=state.commands[i];
      if (!std::isfinite(position) || !std::isfinite(command) ||
          position<bound.lower || position>bound.upper ||
          std::abs(position-command)>bound.handover_error) {
        reject("INVALID_TRACKING_SNAPSHOT");
      }
    }
    // Loaded actuators can have a legitimate position error. Turning measured
    // positions into targets would move every joint before the selected test.
    // Keep command continuity; the transition interval is now a pre-test hold.
    return JointTestTrajectory(state.commands,state.commands,profile.bounds,spec.selected,
      spec.amplitude,spec.transition,spec.duration,spec.cycles,profile.max_total_seconds,
      profile.chirp_start_hz,profile.chirp_end_hz,profile.simultaneous_directions);
  }
  const LoadedJointTestProfile profile_;
  const JointTestSnapshot snapshot_;
  const PreparedJointTest specification_;
  const JointTestTrajectory trajectory_;
  const double prepared_at_, max_feedback_age_;
};
}
