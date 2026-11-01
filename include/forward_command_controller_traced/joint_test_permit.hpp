#pragma once
#include "forward_command_controller_traced/joint_test_admission.hpp"

namespace forward_command_controller_traced
{
enum class JointTestStartResult {accepted, invalid, expired, not_ready, conflict, consumed};
struct JointTestStartDecision {JointTestStartResult result; bool dispatch;};

// Serialized, non-realtime one-use START/RENEW permission. Not caller authentication.
// The adapter retains operation outcomes and supplies a unique challenge per gate.
class JointTestPermitGate
{
public:
  JointTestPermitGate(std::string instance, uint64_t activation, uint64_t session,
    std::string challenge, double issued_at, double lifetime, uint8_t action=1, uint64_t after_sequence=0)
  : instance_(std::move(instance)), activation_(activation), session_(session),
    challenge_(std::move(challenge)), issued_at_(issued_at), deadline_(issued_at+lifetime),
    action_(action), after_sequence_(after_sequence)
  {
    if (!canonical_uuid(instance_) || !canonical_uuid(challenge_) || !activation || !session ||
        !std::isfinite(issued_at) || issued_at<0 || !std::isfinite(lifetime) || lifetime<=0 ||
        !std::isfinite(deadline_) || (action!=1 && action!=2)) {throw std::invalid_argument("Invalid joint-test permit gate");}
  }
  JointTestPermitGate(const JointTestPermitGate&)=delete;
  JointTestPermitGate& operator=(const JointTestPermitGate&)=delete;
  double remaining(double now) const
  {return !used_ && std::isfinite(now) && now>=issued_at_ && now<deadline_ ? deadline_-now : 0.;}
  double issued_at() const {return issued_at_;}
  const std::string& challenge() const {return challenge_;}

  template<class Request>
  JointTestStartDecision consume(const Request& request, double now, bool ready)
  {
    const bool matches=request.action==action_ && request.controller_instance_id==instance_ &&
      request.activation_generation==activation_ && request.session_id==session_ &&
      request.permit_challenge==challenge_ && request.sequence>after_sequence_;
    if (used_) {
      if (request.operation_id==operation_) {
        if (!matches || request.sequence!=sequence_) {return {JointTestStartResult::conflict,false};}
        return {outcome_,false}; // Return the recorded outcome, never dispatch twice.
      }
      return {JointTestStartResult::consumed,false};
    }
    if (!matches || !canonical_uuid(request.operation_id)) {return {JointTestStartResult::invalid,false};}
    operation_=request.operation_id;
    sequence_=request.sequence;
    used_=true;
    if (!std::isfinite(now) || now<issued_at_ || now>=deadline_) {outcome_=JointTestStartResult::expired;}
    else if (!ready) {outcome_=JointTestStartResult::not_ready;}
    else {outcome_=JointTestStartResult::accepted;}
    return {outcome_,outcome_==JointTestStartResult::accepted};
  }
private:
  const std::string instance_;
  const uint64_t activation_, session_;
  const std::string challenge_;
  const double issued_at_, deadline_;
  const uint8_t action_;
  const uint64_t after_sequence_;
  std::string operation_;
  uint64_t sequence_=0;
  bool used_=false;
  JointTestStartResult outcome_=JointTestStartResult::invalid;
};
}
