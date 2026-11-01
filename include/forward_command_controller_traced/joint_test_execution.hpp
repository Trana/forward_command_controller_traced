#pragma once
#include "forward_command_controller_traced/joint_test_plan.hpp"
#include "forward_command_controller_traced/joint_test_playback.hpp"

namespace forward_command_controller_traced
{
// Construct outside update. Thereafter one serialized update thread owns it.
// The adapter retains command-interface ownership throughout execution/hold.
class JointTestExecution
{
public:
  JointTestExecution(PreparedJointTestPlan plan,uint64_t session,double lease,double max_gap)
  : plan_(std::move(plan)), fence_{plan_.snapshot().activation,session},
    playback_(plan_.trajectory(),fence_,lease,max_gap),
    applied_(plan_.snapshot().commands), candidate_(applied_) {}
  JointTestExecution(const JointTestExecution&)=delete;
  JointTestExecution& operator=(const JointTestExecution&)=delete;

  // A fresh authenticated one-use permit must already be consumed by the adapter.
  bool begin(const JointTestSnapshot& live,const std::string& current_profile_hash,double now) noexcept
  {return begin(live,current_profile_hash,now,now);}

  bool begin(const JointTestSnapshot& live,const std::string& current_profile_hash,
    double issued_at,double now) noexcept
  {
    if (attempted_) {return false;}
    attempted_=true;
    if (current_profile_hash!=plan_.profile().value().sha256 ||
        plan_.check_apply(live,now)!=JointTestApplyCheck::ready) {
      playback_.cancel(fence_);
      return false;
    }
    begun_=playback_.begin(fence_,issued_at,now);
    return begun_;
  }

  bool cancel(JointTestFence fence) {return playback_.cancel(fence);}
  bool renew(JointTestFence fence,double now) {return playback_.renew(fence,now);}
  bool renew(JointTestFence fence,double issued_at,double now)
  {return playback_.renew(fence,issued_at,now);}

  // write(index,target) must not throw/allocate. False means the interface write
  // was not acknowledged. Successful interface writes are not physical motion proof.
  template<class Writer>
  bool update(double now,bool permitted,bool feedback_ready,Writer&& write)
  {
    if (!begun_) {return false;}
    const bool advancing=playback_.tick(now,permitted,feedback_ready,candidate_);
    for (size_t i=0;i<applied_.size();++i) {
      const double target=advancing ? candidate_[i] : applied_[i];
      if (!write(i,target)) {
        write_fault_=true;
        playback_.cancel(fence_);
        return false;
      }
      applied_[i]=target;
    }
    return true; // All writes in this update succeeded, including terminal hold.
  }
  const std::vector<double>& applied_targets() const {return applied_;}
  bool write_fault() const {return write_fault_;}
  JointTestStop reason() const {return playback_.reason();}
  double permit_until() const {return playback_.permit_until();}
private:
  const PreparedJointTestPlan plan_;
  const JointTestFence fence_;
  JointTestPlayback playback_;
  std::vector<double> applied_,candidate_;
  bool attempted_=false,begun_=false,write_fault_=false;
};
}
