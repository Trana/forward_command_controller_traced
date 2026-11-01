#include <cassert>
#include <cmath>
#include <iostream>
#include "forward_command_controller_traced/joint_test_profile.hpp"
using namespace forward_command_controller_traced;

int main()
{
  for (int setting : {5,7,10,14,15}) {
  const bool five=setting==15;
  const bool fast=setting>=14;
  const int degrees=fast ? 10 : setting;
  const bool wide=degrees>5;
  const double amplitude=degrees*3.141592653589793/180.;
  const double speed=five ? 5.7 : fast ? 4.6 : degrees==10 ? 3.5 : wide ? 2.5 : 1.2;
  const double acceleration=five ? 180. : fast ? 120. : degrees==10 ? 70. : wide ? 50. : 15.;
  const double end_hz=five ? 5. : fast ? 4. : wide ? 3. : 2.;
  std::vector<double> pose(12,0.), directions={1,1,-1,-1,1,1,-1,-1,-1,-1,1,1};
  std::vector<JointTestBound> bounds(12,{-1.,1.,speed,acceleration,.05235987755982989});
  for (double duration : {8.,20.,60.}) {
  JointTestTrajectory trajectory(pose,pose,bounds,12,amplitude,1.,duration,1,duration+1.,.1,end_hz,directions);
  std::vector<double> output(12), previous(12), velocity(12), old_velocity(12);
  const double dt=.0005;
  for (int step=0;step<=static_cast<int>((duration+1.)/dt);++step) {
    const auto phase=trajectory.sample(step*dt,output);
    assert(phase!=JointTestPhase::invalid);
    for (size_t j=0;j<12;++j) {
      assert(std::abs(output[j])<=amplitude+1e-12);
      assert(std::abs(output[j]-output[0]*directions[j])<1e-12);
      if (step) {
        velocity[j]=(output[j]-previous[j])/dt;
        assert(std::abs(velocity[j])<speed);
        if (step>1) {assert(std::abs((velocity[j]-old_velocity[j])/dt)<acceleration);}
      }
    }
    previous=output;old_velocity=velocity;
  }
  for (double q:output) {assert(q==0.);}
  }
  bool rejected=false;
  pose[11]=.99;
  try {JointTestTrajectory bad(pose,pose,bounds,12,amplitude,1.,20.,1,21.,.1,2.,directions);}
  catch (const std::invalid_argument&) {rejected=true;}
  assert(rejected);
  }
  std::cout<<"Simultaneous twelve-joint waveform bounds passed\n";
}
