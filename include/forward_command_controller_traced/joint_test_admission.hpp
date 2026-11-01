#pragma once
#include <algorithm>
#include <set>
#include <string>
#include "forward_command_controller_traced/joint_test_trajectory.hpp"

namespace forward_command_controller_traced
{
struct JointTestProfile
{
  std::string id, sha256;
  std::vector<std::string> joints;
  std::vector<JointTestBound> bounds;
  std::vector<double> max_amplitude;
  double max_total_seconds;
  uint32_t max_cycles;
  // Frequencies belong to a reviewed immutable profile, never a client request.
  // Both zero preserves the legacy fixed-frequency sine profile.
  double chirp_start_hz=0., chirp_end_hz=0.;
  // Explicit opt-in. Empty retains single-joint semantics and profile digest.
  std::vector<double> simultaneous_directions{};
};

inline bool lowercase_hex(const std::string& value, size_t size)
{
  return value.size() == size && std::all_of(value.begin(), value.end(), [](char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');});
}

inline bool canonical_uuid(const std::string& value)
{
  if (value.size() != 36) {return false;}
  std::string digits;
  for (size_t i=0; i<value.size(); ++i) {
    if (i==8 || i==13 || i==18 || i==23) {if (value[i] != '-') {return false;}}
    else {digits += value[i];}
  }
  return lowercase_hex(digits,32) && digits != std::string(32,'0');
}

class JointTestAdmissionError : public std::invalid_argument
{
public:
  explicit JointTestAdmissionError(const std::string& code) : std::invalid_argument(code) {}
};

struct PreparedJointTest
{
  size_t selected;
  double amplitude, transition, duration;
  uint32_t cycles;
};

inline void validate_joint_test_profile(const JointTestProfile& profile)
{
  const auto reject = []() {throw JointTestAdmissionError("INVALID_PROFILE");};
  if (profile.id.empty() || profile.id.size()>64 || !lowercase_hex(profile.sha256,64) ||
      profile.joints.empty() || profile.joints.size()>256 || profile.bounds.size()!=profile.joints.size() ||
      profile.max_amplitude.size()!=profile.joints.size() ||
      std::set<std::string>(profile.joints.begin(), profile.joints.end()).size()!=profile.joints.size() ||
      !std::isfinite(profile.max_total_seconds) || profile.max_total_seconds<=0 || profile.max_cycles==0) {reject();}
  if (!std::isfinite(profile.chirp_start_hz) || !std::isfinite(profile.chirp_end_hz) ||
      !((profile.chirp_start_hz==0 && profile.chirp_end_hz==0) ||
        (profile.chirp_start_hz>0 && profile.chirp_end_hz>=profile.chirp_start_hz && profile.max_cycles==1))) {reject();}
  if (!profile.simultaneous_directions.empty()) {
    if (profile.simultaneous_directions.size()!=profile.joints.size() || profile.chirp_start_hz<=0) {reject();}
    for (double direction : profile.simultaneous_directions) {
      if (direction!=1. && direction!=-1.) {reject();}
    }
  }
  for (size_t i=0; i<profile.joints.size(); ++i) {
    const auto& b=profile.bounds[i];
    if (profile.joints[i].empty() || profile.joints[i].size()>64 ||
        !std::isfinite(profile.max_amplitude[i]) || profile.max_amplitude[i]<=0 ||
        !std::isfinite(b.lower) || !std::isfinite(b.upper) || b.lower>=b.upper ||
        !std::isfinite(b.speed) || b.speed<=0 || !std::isfinite(b.acceleration) || b.acceleration<=0 ||
        !std::isfinite(b.handover_error) || b.handover_error<0) {reject();}
  }
}

// Non-realtime semantic validation for PrepareJointTest::Request. Passing this
// check grants no session/lease. The update loop must recheck fences/readiness
// against fresh controller-owned state. Construct trajectory objects outside
// the update loop from a coherent state snapshot; do not throw/allocate there.
template<class Request>
PreparedJointTest validate_joint_test_prepare(const Request& request,
  const JointTestProfile& profile, const std::string& instance, uint64_t generation)
{
  const auto reject = [](const char* code) {throw JointTestAdmissionError(code);};
  if (!canonical_uuid(request.controller_instance_id) || !canonical_uuid(request.operation_id) ||
      request.activation_generation == 0 || !lowercase_hex(request.profile_sha256,64) ||
      !std::isfinite(request.amplitude_rad) || request.amplitude_rad <= 0 ||
      !std::isfinite(request.transition_duration_s) || request.transition_duration_s <= 0 ||
      !std::isfinite(request.excitation_duration_s) || request.excitation_duration_s <= 0 ||
      request.cycles == 0) {reject("INVALID_REQUEST");}
  if (request.controller_instance_id != instance || request.activation_generation != generation) {
    reject("STALE_CONTROLLER");
  }
  validate_joint_test_profile(profile);
  if (request.profile_id!=profile.id || request.profile_sha256!=profile.sha256) {reject("PROFILE_MISMATCH");}
  const auto it=std::find(profile.joints.begin(),profile.joints.end(),request.joint_name);
  const bool simultaneous=!profile.simultaneous_directions.empty();
  if (simultaneous ? request.joint_name!="__all__" : it==profile.joints.end()) {reject("UNKNOWN_JOINT");}
  const auto selected=static_cast<size_t>(it-profile.joints.begin());
  const double max_amplitude=simultaneous ?
    *std::min_element(profile.max_amplitude.begin(),profile.max_amplitude.end()) : profile.max_amplitude[selected];
  const double total=request.transition_duration_s+request.excitation_duration_s;
  if (!std::isfinite(total) || total>profile.max_total_seconds || request.cycles>profile.max_cycles ||
      request.amplitude_rad>max_amplitude) {reject("PROFILE_LIMIT");}
  return {selected, request.amplitude_rad, request.transition_duration_s,
    request.excitation_duration_s, request.cycles};
}
}
