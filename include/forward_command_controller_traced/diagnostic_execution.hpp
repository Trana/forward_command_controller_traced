#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace forward_command_controller_traced::diagnostic {
constexpr size_t joints = 12;
using Vector = std::array<double, joints>;
struct Key {
  uint64_t epoch{}, generation{}, operation{};
  bool operator==(const Key& b) const noexcept {
    return epoch == b.epoch && generation == b.generation && operation == b.operation;
  }
};
enum class Phase { idle, running, completed, cancelled, faulted };
enum class Fault { none, invalid_plan, identity, feedback, tracking, deadline, permission, continuity, write };
struct Limits {
  Vector lower{}, upper{}, amplitude{};
  double feedback_age_s{.2}, tracking_rad{.2}, update_gap_s{.2};
  double permission_s{.2}, step_rad{.3}, speed_rad_s{20.};
  bool tracking_report_only{false};
};
struct Plan {
  Key key;
  Vector baseline{}, amplitude{};
  double duration_s{}, start_hz{}, end_hz{};
  bool operator==(const Plan& b) const noexcept {
    return key == b.key && baseline == b.baseline && amplitude == b.amplitude &&
      duration_s == b.duration_s && start_hz == b.start_hz && end_hz == b.end_hz;
  }
};
struct Feedback { Vector position{}, receipt_s{}; };
struct Snapshot {
  Key key;
  Phase phase{Phase::idle};
  Fault fault{Fault::none};
  Vector targets{};
  uint64_t generated{}, applied{};
  double elapsed_s{};
  bool hold_applied{};
  uint64_t tracking_violations{};
  double peak_tracking_rad{};
  size_t peak_tracking_joint{};
};

inline bool finite(double x) noexcept { return std::isfinite(x); }
inline Vector sample(const Plan& p, double elapsed) noexcept {
  auto q = p.baseline;
  if (elapsed <= 0. || elapsed >= p.duration_s) return q;
  constexpr double pi = 3.14159265358979323846;
  const double envelope = std::sin(pi * elapsed / p.duration_s);
  const double phase = 2.*pi*(p.start_hz*elapsed +
    (p.end_hz-p.start_hz)*elapsed*elapsed/(2.*p.duration_s));
  const double wave = envelope*envelope*std::sin(phase);
  for (size_t i=0; i<joints; ++i) q[i] += p.amplitude[i]*wave;
  return q;
}

// Single update-thread owner. No ROS, allocation, mutex, logging or file access.
// The transport must fence the controller epoch before dispatching messages here.
class Execution {
 public:
  explicit Execution(Limits limits, uint64_t epoch) : limits_(limits), epoch_(epoch) {}
  const Snapshot& status() const noexcept { return state_; }
  bool owns_output() const noexcept { return state_.phase != Phase::idle; }
  bool valid(const Plan& p) const noexcept {
    if (!p.key.epoch || !p.key.generation || !p.key.operation ||
        !finite(p.duration_s) || p.duration_s < 1. || p.duration_s > 60. ||
        !finite(p.start_hz) || !finite(p.end_hz) || p.start_hz < .05 ||
        p.end_hz < .05 || p.start_hz > 10. || p.end_hz > 10.) return false;
    bool moving=false;
    for (size_t i=0; i<joints; ++i) {
      const double a=std::abs(p.amplitude[i]);
      if (!finite(p.baseline[i]) || !finite(a) || !finite(limits_.lower[i]) ||
          !finite(limits_.upper[i]) || limits_.lower[i]>=limits_.upper[i] ||
          !finite(limits_.amplitude[i]) || limits_.amplitude[i]<0. ||
          a>limits_.amplitude[i] || p.baseline[i]-a<limits_.lower[i] ||
          p.baseline[i]+a>limits_.upper[i]) return false;
      moving |= a>0.;
    }
    return moving;
  }
  bool start(const Plan& p, double now, const Vector& held, const Feedback& feedback) noexcept {
    // Retries cannot restart time, clear a fault, or renew permission.
    if (state_.key == p.key && state_.phase != Phase::idle) return p == plan_;
    if (owns_output() || p.key.epoch!=epoch_ || p.key.generation<=generation_ ||
        !valid(p) || !finite(now) || p.baseline != held) return false;
    generation_=p.key.generation;
    state_ = {}; state_.key=p.key; state_.targets=held;
    plan_=p; started_=last_update_=last_target_=last_permission_=now;
    permission_sequence_=0; next_target_=now+.02;
    budget_.fill(.05);
    state_.phase=Phase::running;
    if (const auto fault=check_feedback(now,feedback); fault!=Fault::none) latch(fault);
    return state_.phase==Phase::running;
  }
  bool renew(Key key, uint64_t sequence, double receipt) noexcept {
    if (!(key==state_.key) || state_.phase!=Phase::running || sequence<=permission_sequence_ ||
        !finite(receipt) || receipt<last_permission_ || receipt-last_permission_>limits_.permission_s) return false;
    permission_sequence_=sequence; last_permission_=receipt; return true;
  }
  bool cancel(Key key) noexcept {
    if (!(key==state_.key) || !owns_output()) return false;
    if (state_.phase==Phase::running || state_.phase==Phase::completed) {
      state_.phase=Phase::cancelled; state_.hold_applied=false;
    }
    return true;
  }
  // Output cannot return to stream mode until the final hold reached interfaces.
  bool release(Key key) noexcept {
    if (!(key==state_.key) || state_.phase==Phase::running || !state_.hold_applied) return false;
    state_.phase=Phase::idle; return true;
  }
  template<class Writer> void update(double now, const Feedback& feedback, Writer&& write) noexcept {
    if (!owns_output()) return;
    if (state_.phase==Phase::running) {
      if (!finite(now) || now<last_update_ || now-last_update_>limits_.update_gap_s) latch(Fault::deadline);
      else if (now-last_permission_>limits_.permission_s) latch(Fault::permission);
      else if (const auto fault=check_feedback(now,feedback,limits_.tracking_report_only); fault!=Fault::none) latch(fault);
    }
    bool new_target=false;
    Vector candidate=state_.targets;
    if (state_.phase==Phase::running) {
      state_.elapsed_s=now-started_;
      if (now+1e-9>=next_target_ || state_.elapsed_s>=plan_.duration_s) {
        candidate=sample(plan_,state_.elapsed_s);
        const double dt=now-last_target_;
        for (size_t i=0;i<joints;++i) {
          const double available=std::min(limits_.step_rad,budget_[i]+limits_.speed_rad_s*dt);
          if (std::abs(candidate[i]-state_.targets[i])>available) { latch(Fault::continuity); break; }
        }
        new_target=state_.phase==Phase::running;
        if (new_target) ++state_.generated;
      }
    }
    // Faults latch before the writer; a rejected sample can never resume later.
    if (!new_target) candidate=state_.targets;
    if (!write(candidate)) { latch(Fault::write); return; }
    if (new_target) {
      const double dt=now-last_target_;
      for (size_t i=0;i<joints;++i)
        budget_[i]=std::min(limits_.step_rad,budget_[i]+limits_.speed_rad_s*dt)-std::abs(candidate[i]-state_.targets[i]);
      state_.targets=candidate; ++state_.applied; last_target_=now;
      // Follow elapsed time. Never emit a burst of missed trajectory samples.
      next_target_=started_+(std::floor((now-started_)/.02+1e-8)+1.)*.02;
      if (state_.elapsed_s>=plan_.duration_s) state_.phase=Phase::completed;
    }
    state_.hold_applied=state_.phase!=Phase::running;
    last_update_=now;
  }
 private:
  Fault check_feedback(double now,const Feedback& f,bool report_only=false) noexcept {
    for (size_t i=0;i<joints;++i) {
      if (!finite(f.position[i]) || !finite(f.receipt_s[i]) || f.receipt_s[i]<=0. ||
          now<f.receipt_s[i] || now-f.receipt_s[i]>limits_.feedback_age_s) return Fault::feedback;
    }
    for (size_t i=0;i<joints;++i) {
      const double error=std::abs(f.position[i]-state_.targets[i]);
      if(error>limits_.tracking_rad) {
        if(!report_only)return Fault::tracking;
        ++state_.tracking_violations;
        if(error>state_.peak_tracking_rad) {
          state_.peak_tracking_rad=error;state_.peak_tracking_joint=i;
        }
      }
    }
    return Fault::none;
  }
  void latch(Fault f) noexcept {
    if (state_.phase==Phase::faulted) return;
    state_.phase=Phase::faulted; state_.fault=f; state_.hold_applied=false;
  }
  Limits limits_;
  uint64_t epoch_{}, generation_{};
  Plan plan_{};
  Snapshot state_{};
  Vector budget_{};
  uint64_t permission_sequence_{};
  double started_{},last_update_{},last_target_{},next_target_{},last_permission_{};
};
} // namespace forward_command_controller_traced::diagnostic
