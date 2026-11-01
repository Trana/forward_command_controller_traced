#pragma once
#include <cstring>
#include <limits>
#include <openssl/evp.h>
#include "forward_command_controller_traced/joint_test_admission.hpp"

namespace forward_command_controller_traced
{
// Construct outside the update loop. Owns a validated, immutable copy; never
// trusts a supplied sha256 field. A digest identifies content, not authorization.
class LoadedJointTestProfile
{
public:
  explicit LoadedJointTestProfile(JointTestProfile input) : value_(load(std::move(input))) {}
  const JointTestProfile& value() const {return value_;}
private:
  static JointTestProfile load(JointTestProfile input)
  {
    input.sha256 = std::string(64,'0');
    validate_joint_test_profile(input);
    std::string bytes(!input.simultaneous_directions.empty() ? "joint-test-profile.v3-simultaneous" :
      (input.chirp_start_hz>0 ? "joint-test-profile.v2-chirp" : "joint-test-profile.v1"));
    bytes.push_back('\0');
    auto integer = [&](uint64_t value, unsigned size) {
      for (unsigned i=size; i>0; --i) {bytes.push_back(static_cast<char>((value >> ((i-1)*8)) & 255));}
    };
    auto text = [&](const std::string& value) {integer(value.size(),4); bytes.append(value);};
    auto real = [&](double value) {
      static_assert(sizeof(double)==8 && std::numeric_limits<double>::is_iec559);
      if (value==0) {value=0.;} // Normalize signed zero.
      uint64_t bits;
      std::memcpy(&bits,&value,8);
      integer(bits,8);
    };
    text(input.id);
    real(input.max_total_seconds);
    integer(input.max_cycles,4);
    if (input.chirp_start_hz>0) {real(input.chirp_start_hz); real(input.chirp_end_hz);}
    integer(input.joints.size(),4);
    for (size_t i=0; i<input.joints.size(); ++i) {
      text(input.joints[i]);
      const auto& b=input.bounds[i];
      for (double value : {b.lower,b.upper,b.speed,b.acceleration,b.handover_error,input.max_amplitude[i]}) {real(value);}
      if (!input.simultaneous_directions.empty()) {real(input.simultaneous_directions[i]);}
    }
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned length=0;
    if (EVP_Digest(bytes.data(),bytes.size(),digest,&length,EVP_sha256(),nullptr)!=1 || length!=32) {
      throw std::runtime_error("Profile SHA-256 calculation failed");
    }
    const char* hex="0123456789abcdef";
    input.sha256.clear();
    for (unsigned i=0; i<length; ++i) {
      input.sha256.push_back(hex[digest[i]>>4]);
      input.sha256.push_back(hex[digest[i]&15]);
    }
    return input;
  }
  const JointTestProfile value_;
};
}
