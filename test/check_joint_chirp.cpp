// Standalone no-hardware numerical checks; compile with C++17.
#include "forward_command_controller_traced/joint_test_chirp.hpp"
#include "forward_command_controller_traced/joint_test_playback.hpp"
#include <cassert>
#include <limits>
using namespace forward_command_controller_traced;

int main()
{
  for (size_t selected=0;selected<12;++selected) {
    const std::vector<double> pose(12,0.);
    const std::vector<JointTestBound> limits(12,{-2.,2.,.1,.2,.05235987755982989});
    JointTestChirp trial(pose,pose,limits,selected,.08726646259971647,1.,20.,.05,.15,21.);
    std::vector<double> sample(12),last(12,0.),last_v(12,0.);
    for (int n=0;n<=42000;++n) {
      assert(trial.sample(n*.0005,sample)!=JointTestPhase::invalid);
      for (size_t j=0;j<12;++j) {
        if (j!=selected) assert(sample[j]==0.);
        assert(std::abs(sample[j])<=.08726646259971648);
        double v=(sample[j]-last[j])/.0005;
        assert(std::abs(v)<.1);
        if (n>1) assert(std::abs((v-last_v[j])/.0005)<.2);
        last[j]=sample[j];last_v[j]=v;
      }
    }
    assert(sample==pose);
  }
  // Approved half-degree thigh progression: unchanged speed/acceleration limits.
  for (size_t selected=4; selected<8; ++selected) {
    std::vector<double> pose(12,0.), sample(12), previous(12,0.), last_v(12,0.);
    const std::vector<JointTestBound> limits(12,{-2.,2.,.1,.2,.05235987755982989});
    JointTestChirp trial(pose,pose,limits,selected,.008726646259971648,1.,20.,.2,.4,21.);
    constexpr double step=.0005;
    for (int n=0;n<=42000;++n) {
      assert(trial.sample(n*step,sample)!=JointTestPhase::invalid);
      for (size_t j=0;j<12;++j) {
        if (j!=selected) assert(sample[j]==0.);
        assert(std::abs(sample[j])<=.008726646259971649);
        const double v=(sample[j]-previous[j])/step;
        assert(std::abs(v)<.1);
        if (n>1) assert(std::abs((v-last_v[j])/step)<.2);
        previous[j]=sample[j]; last_v[j]=v;
      }
    }
    assert(sample==pose);
  }
  const std::vector<JointTestBound> bounds(3, {-1,1,0.15,0.6,0.05});
  const std::vector<double> q(3,0.2);
  JointTestChirp chirp(q,q,bounds,1,0.02,1,20,0.2,0.8,21);
  std::vector<double> out(3), previous(3,0.2), velocity(3,0), last_velocity(3,0);
  constexpr double dt=0.0005;
  for (int k=0;k<=42002;++k) {
    assert(chirp.sample(k*dt,out)!=JointTestPhase::invalid);
    assert(out[0]==0.2 && out[2]==0.2);
    assert(std::abs(out[1]-0.2)<=0.02000001);
    if (k) {
      for (size_t j=0;j<3;++j) {
        velocity[j]=(out[j]-previous[j])/dt;
        assert(std::abs(velocity[j])<bounds[j].speed);
        if (k>1) {assert(std::abs((velocity[j]-last_velocity[j])/dt)<bounds[j].acceleration);}
      }
    }
    previous=out;last_velocity=velocity;
  }
  assert(chirp.sample(21,out)==JointTestPhase::hold && out==q);
  out={7,8,9};
  assert(chirp.sample(-1,out)==JointTestPhase::invalid && out==std::vector<double>({7,8,9}));
  assert(chirp.sample(std::numeric_limits<double>::quiet_NaN(),out)==JointTestPhase::invalid);
  // Equal frequencies reproduce the established windowed sine exactly.
  JointTestChirp constant(q,q,bounds,1,0.02,1,20,0.2,0.2,21);
  JointTestTrajectory sine(q,q,bounds,1,0.02,1,20,4,21);
  std::vector<double> other(3);
  for (int k=0;k<2100;++k) {
    constant.sample(k*.01,out);sine.sample(k*.01,other);
    assert(std::abs(out[1]-other[1])<1e-12);
  }
  for (double end : {-1.,0.,0.1,20.,std::numeric_limits<double>::infinity()}) {
    bool rejected=false;
    try {JointTestChirp bad(q,q,bounds,1,0.02,1,20,0.2,end,21);}
    catch (const std::invalid_argument&) {rejected=true;}
    assert(rejected);
  }
  JointTestTrajectory bounded(q,q,bounds,1,0.02,1,20,1,21,0.2,0.8);
  JointTestPlayback playback(bounded,{1,1},0.,0.2,0.02);
  assert(playback.tick(.01,true,true,out));
  assert(playback.cancel({1,1}));
  const auto held=out;
  assert(!playback.tick(.02,true,true,out) && out==held);
  assert(playback.reason()==JointTestStop::cancelled);
  JointTestPlayback lost_feedback(bounded,{1,2},0.,0.2,0.02);
  assert(!lost_feedback.tick(.01,true,false,out) && out==held);
  assert(lost_feedback.reason()==JointTestStop::readiness);
  JointTestPlayback expired(bounded,{1,3},0.,0.01,0.02);
  assert(!expired.tick(.011,true,true,out) && out==held);
  assert(expired.reason()==JointTestStop::lease);
}
