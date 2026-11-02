#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>
#include <utility>

namespace forward_command_controller_traced
{
struct JointTestBound {double lower, upper, speed, acceleration, handover_error;};
enum class JointTestPhase {invalid, transition, excitation, hold};

// Pure trajectory mathematics. Construction/validation is non-realtime.
// sample() uses caller-owned storage and performs no allocation or transport.
// This class does not grant ownership, check feedback or authorize movement.
class JointTestTrajectory
{
public:
  JointTestTrajectory(std::vector<double> previous, std::vector<double> baseline,
    const std::vector<JointTestBound>& bounds, size_t selected, double amplitude,
    double transition, double duration, uint32_t cycles, double max_total,
    double chirp_start_hz=0., double chirp_end_hz=0., std::vector<double> directions={})
  : previous_(std::move(previous)), baseline_(std::move(baseline)), selected_(selected),
    amplitude_(amplitude), transition_(transition), duration_(duration), cycles_(cycles),
    chirp_start_(chirp_start_hz), chirp_end_(chirp_end_hz), directions_(std::move(directions))
  {
    auto require = [](bool ok) {if (!ok) {throw std::invalid_argument("Invalid bounded joint trajectory");}};
    require(!baseline_.empty() && previous_.size() == baseline_.size() &&
      bounds.size() == baseline_.size() &&
      ((directions_.empty() && selected < baseline_.size()) ||
       (directions_.size()==baseline_.size() && selected==baseline_.size())));
    for (double direction : directions_) {require(direction==1. || direction==-1.);}
    require(positive(amplitude) && positive(transition) && positive(duration) &&
      positive(max_total) && std::isfinite(transition + duration) &&
      transition + duration <= max_total && cycles > 0);
    require(std::isfinite(chirp_start_) && std::isfinite(chirp_end_) &&
      ((chirp_start_==0 && chirp_end_==0) ||
       (chirp_start_>0 && chirp_end_>=chirp_start_ && cycles==1)));
    for (size_t i = 0; i < baseline_.size(); ++i) {
      const auto& b = bounds[i];
      require(std::isfinite(b.lower) && std::isfinite(b.upper) && b.lower < b.upper &&
        positive(b.speed) && positive(b.acceleration) &&
        std::isfinite(b.handover_error) && b.handover_error >= 0);
      const auto q = baseline_[i], p = previous_[i];
      require(std::isfinite(q) && std::isfinite(p) && b.lower <= q && q <= b.upper &&
        b.lower <= p && p <= b.upper);
      const double delta = std::abs(p-q);
      require(delta <= b.handover_error && 1.875*delta/transition <= b.speed &&
        6*delta/transition/transition <= b.acceleration);
      if (!directions_.empty() || i == selected) {
        require(std::isfinite(q-amplitude) && std::isfinite(q+amplitude) &&
          b.lower <= q-amplitude && q+amplitude <= b.upper);
        const double omega = chirp_start_>0 ? 2*pi*chirp_end_ : 2*pi*cycles/duration;
        const double sweep = chirp_start_>0 ? 2*pi*(chirp_end_-chirp_start_)/duration : 0.;
        const double envelope = pi/duration;
        const double speed = amplitude*(omega+envelope);
        const double acceleration = amplitude*(omega*omega+2*omega*envelope+2*envelope*envelope+sweep);
        require(std::isfinite(speed) && std::isfinite(acceleration) &&
          speed <= b.speed && acceleration <= b.acceleration);
      }
    }
  }

  JointTestPhase sample(double elapsed, std::vector<double>& output) const
  {
    if (!std::isfinite(elapsed) || elapsed < 0 || output.size() != baseline_.size()) {
      return JointTestPhase::invalid; // Invalid calls leave output unchanged.
    }
    if (elapsed < transition_) {
      const double u = elapsed/transition_;
      const double blend = u*u*u*(10-15*u+6*u*u);
      for (size_t i=0; i<output.size(); ++i) {output[i] = previous_[i]+(baseline_[i]-previous_[i])*blend;}
      return JointTestPhase::transition;
    }
    for (size_t i=0; i<output.size(); ++i) {output[i] = baseline_[i];}
    const double t = elapsed-transition_;
    if (t >= duration_) {return JointTestPhase::hold;}
    const double phase = pi*t/duration_;
    const double envelope = std::sin(phase);
    const double angle=chirp_start_>0 ?
      2*pi*(chirp_start_*t+0.5*(chirp_end_-chirp_start_)*t*t/duration_) : 2*cycles_*phase;
    const double excursion=amplitude_*std::sin(angle)*envelope*envelope;
    if (directions_.empty()) {output[selected_] += excursion;}
    else {
      for (size_t i=0;i<output.size();++i) {output[i] += directions_[i]*excursion;}
    }
    return JointTestPhase::excitation;
  }

  double total_seconds() const {return transition_ + duration_;}
  const std::vector<double>& initial_targets() const {return previous_;}

private:
  static bool positive(double value) {return std::isfinite(value) && value > 0;}
  static constexpr double pi = 3.14159265358979323846;
  std::vector<double> previous_, baseline_;
  size_t selected_;
  double amplitude_, transition_, duration_;
  uint32_t cycles_;
  double chirp_start_, chirp_end_;
  std::vector<double> directions_;
};
}
