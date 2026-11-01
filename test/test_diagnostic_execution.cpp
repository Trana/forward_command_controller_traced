#include "forward_command_controller_traced/diagnostic_execution.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using namespace forward_command_controller_traced::diagnostic;
Limits limits() { Limits l; l.lower.fill(-2.); l.upper.fill(2.); l.amplitude.fill(.4); return l; }
Feedback feedback(double now,Vector q={}) { Feedback f; f.position=q; f.receipt_s.fill(now); return f; }
Plan plan(uint64_t generation=1) { Plan p; p.key={42,generation,7}; p.duration_s=2.; p.start_hz=.1;p.end_hz=5.;p.amplitude.fill(.1);return p; }
int main() {
  auto p=plan();
  Execution e(limits(),42);
  assert(!e.start(p,1.,{},feedback(.5)));
  assert(e.status().fault==Fault::feedback);
  e.update(1.,feedback(1.),[](const Vector&){return true;});
  assert(e.release(p.key));
  assert(!e.start(p,1.,{},feedback(1.))); // No replay after release.
  p=plan(2); assert(e.start(p,1.,{},feedback(1.)));
  Vector held{}; int writes=0;
  auto write=[&](const Vector& q){held=q;++writes;return true;};
  for(int tick=1;tick<=400;++tick) {
    const double now=1.+tick*.005;
    assert(e.renew(p.key,static_cast<uint64_t>(tick),now));
    const auto before=held;
    e.update(now,feedback(now,held),write);
    assert(e.status().fault==Fault::none);
    if(tick%4) assert(held==before);
    else {const auto expected=sample(p,tick*.005);for(size_t i=0;i<joints;++i) assert(std::abs(held[i]-expected[i])<1e-12);}
  }
  assert(writes==400 && e.status().generated==100 && e.status().applied==100);
  assert(e.status().phase==Phase::completed && held==p.baseline);
  assert(e.status().hold_applied);
  assert(e.start(p,3.1,held,feedback(3.1))); // Idempotent retry stays completed.
  assert(e.status().phase==Phase::completed);
  assert(e.release(p.key));
  p=plan(3);assert(e.start(p,4.,held,feedback(4.)));
  e.update(4.021,feedback(4.021),write);
  const auto stopped=held;
  assert(!e.cancel({41,3,7}));assert(!e.cancel({42,2,7}));
  assert(e.cancel(p.key));assert(!e.release(p.key));
  e.update(4.03,feedback(4.03),write);
  assert(e.status().hold_applied && held==stopped);
  assert(e.release(p.key));
  for(uint64_t gen=4;gen<6;++gen) {
    p=plan(gen);p.baseline=held;
    assert(e.start(p,5.,held,feedback(5.,held)));
    e.update(5.3,feedback(5.3,held),write);
    assert(e.status().fault==Fault::deadline);
    const auto q=held;
    assert(!e.renew(p.key,1,5.31));
    e.update(5.31,feedback(5.31,held),write);
    assert(held==q && e.status().fault==Fault::deadline);
    assert(e.release(p.key));
  }
  p=plan(6);p.baseline=held;assert(e.start(p,6.,held,feedback(6.,held)));
  // A burst is never emitted after missed nominal ticks.
  e.update(6.105,feedback(6.105,held),write);
  assert(e.status().generated==1);
  e.update(6.11,feedback(6.11,held),write);
  assert(e.status().generated==1);
  e.update(6.12,feedback(6.12,held),write);
  assert(e.status().generated==2);
  e.update(6.201,feedback(6.201,held),write);
  assert(e.status().fault==Fault::permission);
  // Frozen receipt stays stale even when the enclosing state is read at 200 Hz.
  Execution frozen(limits(),42);p=plan();assert(frozen.start(p,10.,{},feedback(10.)));
  for(int i=1;i<=41;++i) {const double t=10.+i*.005;frozen.renew(p.key,static_cast<uint64_t>(i),t);frozen.update(t,feedback(10.),[](const Vector&){return true;});}
  assert(frozen.status().fault==Fault::feedback);
  auto bad=p;bad.amplitude[0]=.401;assert(!frozen.valid(bad));
  bad=p;bad.duration_s=std::numeric_limits<double>::quiet_NaN();assert(!frozen.valid(bad));
  Execution restarted(limits(),43);assert(!restarted.start(p,11.,{},feedback(11.)));
  Execution failed_write(limits(),42);p=plan();
  assert(failed_write.start(p,20.,{},feedback(20.)));
  failed_write.update(20.02,feedback(20.02),[](const Vector&){return false;});
  assert(failed_write.status().fault==Fault::write && !failed_write.status().hold_applied);
  failed_write.update(20.03,feedback(20.03),[](const Vector& q){assert(q==Vector{});return true;});
  assert(failed_write.status().fault==Fault::write && failed_write.status().hold_applied);
  auto tight=limits();tight.step_rad=0.;tight.speed_rad_s=0.;
  Execution continuity(tight,42);assert(continuity.start(p,30.,{},feedback(30.)));
  continuity.update(30.02,feedback(30.02),[](const Vector& q){assert(q==Vector{});return true;});
  assert(continuity.status().fault==Fault::continuity);
  continuity.update(30.03,feedback(30.03),[](const Vector& q){assert(q==Vector{});return true;});
  assert(continuity.status().fault==Fault::continuity);
  auto report_limits=limits();report_limits.tracking_report_only=true;
  Vector lag{};lag[3]=.35;
  Execution initial_tracking(report_limits,42);
  assert(!initial_tracking.start(plan(),40.,{},feedback(40.,lag)));
  assert(initial_tracking.status().fault==Fault::tracking);
  Execution reporting(report_limits,42);
  assert(reporting.start(plan(),40.,{},feedback(40.)));
  reporting.update(40.02,feedback(40.02,lag),[](const Vector&){return true;});
  assert(reporting.status().phase==Phase::running);
  assert(reporting.status().tracking_violations==1);
  assert(reporting.status().peak_tracking_joint==3);
  assert(reporting.status().peak_tracking_rad==.35);
  // Report-only tracking must not mask stale feedback on another joint.
  auto stale=feedback(40.03,lag);stale.receipt_s[11]=39.;
  reporting.update(40.03,stale,[](const Vector&){return true;});
  assert(reporting.status().fault==Fault::feedback);
  Execution strict(limits(),42);
  assert(strict.start(plan(),41.,{},feedback(41.)));
  strict.update(41.02,feedback(41.02,lag),[](const Vector&){return true;});
  assert(strict.status().fault==Fault::tracking);
  std::cout<<"native diagnostic: cadence, endpoints, idempotency, cancellation, epochs, repeated faults, permission, freshness passed\n";
}
