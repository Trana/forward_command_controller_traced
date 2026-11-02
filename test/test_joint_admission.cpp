#include <gtest/gtest.h>
#include <limits>
#include "runtime_interfaces/srv/prepare_joint_test.hpp"
#include "runtime_interfaces/srv/control_joint_test.hpp"
#include "forward_command_controller_traced/joint_test_permit.hpp"
#include "forward_command_controller_traced/joint_test_playback.hpp"
#include "forward_command_controller_traced/joint_test_execution.hpp"
#include "forward_command_controller_traced/joint_test_admission.hpp"
#include "forward_command_controller_traced/joint_test_profile.hpp"
#include "forward_command_controller_traced/joint_test_plan.hpp"
#include "forward_command_controller_traced/experimental_joint_hold_controller.hpp"
#include <cstdlib>
#include <thread>
#include <cstdio>
using namespace forward_command_controller_traced;
using Request = runtime_interfaces::srv::PrepareJointTest::Request;
static const std::string instance="12345678-1234-4234-8234-123456789abc";
static JointTestProfile profile()
{return {"probe", std::string(64,'0'), {"hip","calf"}, {{-1,1,.1,.2,.05},{-1,1,.1,.2,.05}}, {.02,.03}, 5., 2};}
static Request request()
{
  Request r;
  r.controller_instance_id=instance; r.activation_generation=1;
  r.operation_id="12345678-1234-4234-8234-123456789abd";
  r.profile_id="probe"; r.profile_sha256=std::string(64,'0'); r.joint_name="calf";
  r.amplitude_rad=.02; r.transition_duration_s=1; r.excitation_duration_s=4; r.cycles=1;
  return r;
}

TEST(JointAdmission, SimultaneousRequiresOptInAndChecksEveryAmplitude)
{
  auto p=profile();auto r=request();r.joint_name="__all__";
  EXPECT_THROW(validate_joint_test_prepare(r,p,instance,1),JointTestAdmissionError);
  p.chirp_start_hz=.1;p.chirp_end_hz=2.;p.max_cycles=1;
  p.simultaneous_directions={1.,-1.};
  const auto loaded=LoadedJointTestProfile(p).value();
  r.profile_sha256=loaded.sha256;
  EXPECT_EQ(validate_joint_test_prepare(r,loaded,instance,1).selected,2u);
  r.amplitude_rad=.025;
  EXPECT_THROW(validate_joint_test_prepare(r,loaded,instance,1),JointTestAdmissionError);
  p.simultaneous_directions={1.,0.};
  EXPECT_THROW(LoadedJointTestProfile{p},JointTestAdmissionError);
}

TEST(JointAdmission, SimultaneousDirectionsAreHashed)
{
  auto p=profile();p.chirp_start_hz=.1;p.chirp_end_hz=2.;p.max_cycles=1;
  const auto legacy=LoadedJointTestProfile(p).value().sha256;
  p.simultaneous_directions={1.,-1.};
  const auto mirrored=LoadedJointTestProfile(p).value().sha256;
  EXPECT_NE(legacy,mirrored);
  p.simultaneous_directions={-1.,1.};
  EXPECT_NE(mirrored,LoadedJointTestProfile(p).value().sha256);
}

static runtime_interfaces::srv::ControlJointTest::Request start_request()
{
  runtime_interfaces::srv::ControlJointTest::Request r;
  r.action=r.START; r.controller_instance_id=instance; r.activation_generation=1; r.session_id=7;
  r.operation_id="12345678-1234-4234-8234-123456789abd";
  r.permit_challenge="12345678-1234-4234-8234-123456789abe"; r.sequence=1;
  return r;
}

TEST(JointStart, ExactRetryReturnsOutcomeWithoutDispatch)
{
  auto r=start_request();
  JointTestPermitGate gate(instance,1,7,r.permit_challenge,10.,.2);
  auto result=gate.consume(r,10.1,true);
  EXPECT_EQ(result.result,JointTestStartResult::accepted); EXPECT_TRUE(result.dispatch);
  result=gate.consume(r,12.,false);
  EXPECT_EQ(result.result,JointTestStartResult::accepted); EXPECT_FALSE(result.dispatch);
  r.sequence++;
  EXPECT_EQ(gate.consume(r,10.1,true).result,JointTestStartResult::conflict);
  r.operation_id="12345678-1234-4234-8234-123456789abf";
  EXPECT_EQ(gate.consume(r,10.1,true).result,JointTestStartResult::consumed);
}

TEST(JointStart, RenewalChallengeRequiresRenewActionAndIncreasingSequence)
{
  auto r=start_request();
  JointTestPermitGate gate(instance,1,7,r.permit_challenge,10.,.2,r.RENEW,5);
  r.sequence=6;
  EXPECT_EQ(gate.consume(r,10.01,true).result,JointTestStartResult::invalid);
  r.action=r.RENEW; r.sequence=5;
  EXPECT_EQ(gate.consume(r,10.01,true).result,JointTestStartResult::invalid);
  r.sequence=6;
  EXPECT_TRUE(gate.consume(r,10.01,true).dispatch);
  EXPECT_EQ(gate.remaining(10.02),0.);
  EXPECT_FALSE(gate.consume(r,10.02,true).dispatch);
  r.sequence=7;
  EXPECT_EQ(gate.consume(r,10.02,true).result,JointTestStartResult::conflict);
}

TEST(JointStart, InvalidRequestsDoNotConsumeButValidFailedAttemptsDo)
{
  auto r=start_request();
  JointTestPermitGate gate(instance,1,7,r.permit_challenge,10.,.2);
  r.session_id=8;
  EXPECT_EQ(gate.consume(r,10.1,true).result,JointTestStartResult::invalid);
  r.session_id=7;
  auto result=gate.consume(r,10.1,false);
  EXPECT_EQ(result.result,JointTestStartResult::not_ready); EXPECT_FALSE(result.dispatch);
  result=gate.consume(r,10.1,true);
  EXPECT_EQ(result.result,JointTestStartResult::not_ready); EXPECT_FALSE(result.dispatch);
  JointTestPermitGate expired(instance,1,7,r.permit_challenge,10.,.2);
  result=expired.consume(r,10.2,true);
  EXPECT_EQ(result.result,JointTestStartResult::expired); EXPECT_FALSE(result.dispatch);
  EXPECT_FALSE(expired.consume(r,10.1,true).dispatch);
}
static void rejects(const Request& r, const char* code, JointTestProfile p=profile())
{
  try {validate_joint_test_prepare(r,p,instance,1); FAIL()<<"Expected rejection";}
  catch (const JointTestAdmissionError& error) {EXPECT_STREQ(error.what(),code);}
}

TEST(JointAdmission, GeneratedRequestSelectsServerJoint)
{
  const auto spec=validate_joint_test_prepare(request(),profile(),instance,1);
  EXPECT_EQ(spec.selected,1u);
  EXPECT_DOUBLE_EQ(spec.amplitude,.02);
}
TEST(JointAdmission, RejectsInvalidNumbersAndIdentifiers)
{
  for (double value : {0., -1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    auto r=request(); r.amplitude_rad=value; rejects(r,"INVALID_REQUEST");
    r=request(); r.transition_duration_s=value; rejects(r,"INVALID_REQUEST");
    r=request(); r.excitation_duration_s=value; rejects(r,"INVALID_REQUEST");
  }
  auto r=request(); r.operation_id=std::string(36,'0'); rejects(r,"INVALID_REQUEST");
  r=request(); r.controller_instance_id="00000000-0000-0000-0000-000000000000"; rejects(r,"INVALID_REQUEST");
  r=request(); r.cycles=0; rejects(r,"INVALID_REQUEST");
}
TEST(JointAdmission, RejectsStaleAndMismatchedProfiles)
{
  auto r=request(); r.activation_generation=2; rejects(r,"STALE_CONTROLLER");
  r=request(); r.controller_instance_id=r.operation_id; rejects(r,"STALE_CONTROLLER");
  r=request(); r.profile_sha256=std::string(64,'a'); rejects(r,"PROFILE_MISMATCH");
  r=request(); r.profile_id="other"; rejects(r,"PROFILE_MISMATCH");
  r=request(); r.joint_name="unknown"; rejects(r,"UNKNOWN_JOINT");
  r=request(); r.amplitude_rad=.04; rejects(r,"PROFILE_LIMIT");
  r=request(); r.cycles=3; rejects(r,"PROFILE_LIMIT");
  r=request(); r.excitation_duration_s=5; rejects(r,"PROFILE_LIMIT");
}
TEST(JointAdmission, InvalidServerProfilesFailClosed)
{
  auto p=profile(); p.joints[1]="hip"; rejects(request(),"INVALID_PROFILE",p);
  p=profile(); p.bounds[0].speed=0; rejects(request(),"INVALID_PROFILE",p);
  p=profile(); p.max_amplitude.clear(); rejects(request(),"INVALID_PROFILE",p);
}

TEST(JointProfile, ChirpFrequenciesAreValidatedAndBoundIntoDigest)
{
  auto p=profile();
  const auto sine_hash=LoadedJointTestProfile(p).value().sha256;
  p.max_cycles=1; p.chirp_start_hz=.2; p.chirp_end_hz=.8;
  const auto chirp_hash=LoadedJointTestProfile(p).value().sha256;
  EXPECT_NE(sine_hash,chirp_hash);
  p.chirp_end_hz=.9;
  EXPECT_NE(chirp_hash,LoadedJointTestProfile(p).value().sha256);
  p.chirp_end_hz=.1;
  EXPECT_THROW((void)LoadedJointTestProfile{p},JointTestAdmissionError);
  p.chirp_end_hz=.8; p.max_cycles=2;
  EXPECT_THROW((void)LoadedJointTestProfile{p},JointTestAdmissionError);
}

TEST(JointProfile, ImmutableContentsDetermineHash)
{
  auto input=profile();
  LoadedJointTestProfile loaded(input);
  const auto original=loaded.value().sha256;
  EXPECT_TRUE(lowercase_hex(original,64));
  EXPECT_NE(original,input.sha256);
  input.sha256=std::string(64,'a');
  EXPECT_EQ(LoadedJointTestProfile(input).value().sha256,original);
  input.bounds[0].speed=.08;
  EXPECT_NE(LoadedJointTestProfile(input).value().sha256,original);
  EXPECT_EQ(loaded.value().bounds[0].speed,.1);
  EXPECT_EQ(loaded.value().sha256,original);
  auto request_value=request();
  request_value.profile_sha256=original;
  EXPECT_NO_THROW(validate_joint_test_prepare(request_value,loaded.value(),instance,1));
  input.bounds[0].speed=NAN;
  EXPECT_THROW(LoadedJointTestProfile rejected(input),JointTestAdmissionError);
}

TEST(JointProfile, CanonicalEncodingMatchesIndependentHash)
{
  // Independent hashlib/struct implementation is recorded in the README.
  const auto loaded=LoadedJointTestProfile(profile());
  EXPECT_EQ(loaded.value().sha256,"eec3550d44977b68bdba039fa405c37318c1ba4836bdef0840c4a67791e497a5");
  auto input=profile();
  input.bounds[0].handover_error=0.;
  const auto positive=LoadedJointTestProfile(input).value().sha256;
  input.bounds[0].handover_error=-0.;
  EXPECT_EQ(LoadedJointTestProfile(input).value().sha256,positive);
}

TEST(JointProfile, EveryLimitAndJointOrderAffectHash)
{
  const auto original=LoadedJointTestProfile(profile()).value().sha256;
  const auto changed=[&](const JointTestProfile& input) {
    EXPECT_NE(LoadedJointTestProfile(input).value().sha256,original);
  };
  auto input=profile(); input.id="other"; changed(input);
  input=profile(); input.max_total_seconds=6.; changed(input);
  input=profile(); input.max_cycles=3; changed(input);
  input=profile(); input.joints[0]="other_joint"; changed(input);
  input=profile(); input.max_amplitude[0]=.01; changed(input);
  for (auto member : {&JointTestBound::lower, &JointTestBound::upper,
      &JointTestBound::speed, &JointTestBound::acceleration, &JointTestBound::handover_error}) {
    input=profile(); input.bounds[0].*member+=.001; changed(input);
  }
  input=profile();
  std::swap(input.joints[0],input.joints[1]);
  std::swap(input.bounds[0],input.bounds[1]);
  std::swap(input.max_amplitude[0],input.max_amplitude[1]);
  changed(input);
}

static JointTestSnapshot snapshot()
{return {instance,1,4,10.,true,false,false,{"hip","calf"},{.01,.01},{0.,0.},{9.99,9.99}};}
static Request hashed_request()
{
  auto r=request(); r.profile_sha256=LoadedJointTestProfile(profile()).value().sha256;
  return r;
}

static Request early_excitation_request()
{
  auto r=hashed_request();
  r.transition_duration_s=.001;  // Finish pre-test hold before exercising faults.
  return r;
}

TEST(JointPlan, OwnsProfileAndPreservesCommandBaseline)
{
  auto state=snapshot();
  PreparedJointTestPlan plan(hashed_request(),LoadedJointTestProfile(profile()),state,10.,.1);
  state.positions[0]=.5;
  EXPECT_EQ(plan.snapshot().positions[0],0.);
  EXPECT_EQ(plan.profile().value().sha256,hashed_request().profile_sha256);
  std::vector<double> output(2);
  EXPECT_EQ(plan.trajectory().sample(0,output),JointTestPhase::transition);
  EXPECT_EQ(output,plan.snapshot().commands);
  EXPECT_EQ(plan.trajectory().sample(2,output),JointTestPhase::excitation);
  EXPECT_DOUBLE_EQ(output[0],.01);
  EXPECT_NE(output[1],.01);
  for (int tick=0; tick<=500; ++tick) {
    const double elapsed=tick*.01;
    plan.trajectory().sample(elapsed,output);
    EXPECT_DOUBLE_EQ(output[0],.01) << elapsed;
    if (elapsed<=1. || elapsed>=5.) {EXPECT_DOUBLE_EQ(output[1],.01);}
    EXPECT_LE(std::abs(output[1]-.01),.02+1e-12);
  }
}

TEST(JointPlan, RejectsStaleInvalidAndUnreadySnapshots)
{
  const auto rejects_state=[](const JointTestSnapshot& state,const char* code) {
    try {
      PreparedJointTestPlan plan(hashed_request(),LoadedJointTestProfile(profile()),state,10.,.1);
      FAIL()<<"Expected rejection";
    } catch (const JointTestAdmissionError& error) {EXPECT_STREQ(error.what(),code);}
  };
  auto s=snapshot(); s.hold_applied=false; rejects_state(s,"HOLD_NOT_READY");
  s=snapshot(); s.tracking_fault=true; rejects_state(s,"HOLD_NOT_READY");
  s=snapshot(); s.feedback_fault=true; rejects_state(s,"HOLD_NOT_READY");
  s=snapshot(); s.revision=0; rejects_state(s,"HOLD_NOT_READY");
  s=snapshot(); std::swap(s.joints[0],s.joints[1]); rejects_state(s,"INVALID_SNAPSHOT");
  s=snapshot(); s.commands.clear(); rejects_state(s,"INVALID_SNAPSHOT");
  for (double t : {0.,9.,10.1,std::numeric_limits<double>::quiet_NaN()}) {
    s=snapshot(); s.sampled_at=t; rejects_state(s,"STALE_SNAPSHOT");
    s=snapshot(); s.receipts[0]=t; rejects_state(s,"STALE_FEEDBACK");
  }
  s=snapshot(); s.activation=2; rejects_state(s,"STALE_CONTROLLER");
  for (double position : {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity(), .2, -1.1}) {
    s=snapshot(); s.positions[0]=position;
    rejects_state(s,"INVALID_TRACKING_SNAPSHOT");
  }
}

TEST(JointPlan, RechecksMathematicalBoundsAgainstSnapshot)
{
  auto edge=snapshot(); edge.commands[1]=.99; edge.positions[1]=.98;
  EXPECT_THROW(PreparedJointTestPlan(hashed_request(),LoadedJointTestProfile(profile()),edge,10.,.1),
    std::invalid_argument); // Command-centered excursion would cross upper limit.
  auto s=snapshot(); s.positions[1]=.999;
  EXPECT_THROW(PreparedJointTestPlan(hashed_request(),LoadedJointTestProfile(profile()),s,10.,.1),
    std::invalid_argument);
  s=snapshot(); s.commands[0]=NAN;
  EXPECT_THROW(PreparedJointTestPlan(hashed_request(),LoadedJointTestProfile(profile()),s,10.,.1),
    std::invalid_argument);
  EXPECT_THROW(PreparedJointTestPlan(hashed_request(),LoadedJointTestProfile(profile()),snapshot(),10.,0.),
    JointTestAdmissionError);
}

TEST(JointPlan, ApplyRejectsChangesAfterPreparation)
{
  using Check=JointTestApplyCheck;
  const PreparedJointTestPlan plan(hashed_request(),LoadedJointTestProfile(profile()),snapshot(),10.,.1);
  EXPECT_EQ(plan.check_apply(snapshot(),10.),Check::ready);
  auto s=snapshot(); s.revision++; s.sampled_at=10.01; s.receipts={10.005,10.005};
  EXPECT_EQ(plan.check_apply(s,10.02),Check::ready);
  s=snapshot(); s.activation++; EXPECT_EQ(plan.check_apply(s,10.),Check::stale_controller);
  s=snapshot(); s.instance="other"; EXPECT_EQ(plan.check_apply(s,10.),Check::stale_controller);
  s=snapshot(); s.revision--; EXPECT_EQ(plan.check_apply(s,10.),Check::stale_controller);
  EXPECT_EQ(plan.check_apply(snapshot(),10.2),Check::expired_plan);
  EXPECT_EQ(plan.check_apply(snapshot(),9.9),Check::expired_plan);
  s=snapshot(); s.commands[0]+=.00001; EXPECT_EQ(plan.check_apply(s,10.),Check::changed_command);
  s=snapshot(); s.commands[0]=NAN; EXPECT_EQ(plan.check_apply(s,10.),Check::changed_command);
  s=snapshot(); s.positions[0]=.2; EXPECT_EQ(plan.check_apply(s,10.),Check::tracking_error);
  s=snapshot(); s.positions[0]=NAN; EXPECT_EQ(plan.check_apply(s,10.),Check::tracking_error);
  s=snapshot(); s.receipts[0]=9.; EXPECT_EQ(plan.check_apply(s,10.),Check::stale_feedback);
  s=snapshot(); s.receipts[0]=10.1; EXPECT_EQ(plan.check_apply(s,10.),Check::stale_feedback);
  s=snapshot(); s.hold_applied=false; EXPECT_EQ(plan.check_apply(s,10.),Check::not_holding);
  s=snapshot(); s.feedback_fault=true; EXPECT_EQ(plan.check_apply(s,10.),Check::not_holding);
  s=snapshot(); s.positions.clear(); EXPECT_EQ(plan.check_apply(s,10.),Check::invalid_state);
}

TEST(JointPlan, OneUseStartBeginsPreparedPlaybackWithoutRetryRestart)
{
  const PreparedJointTestPlan plan(early_excitation_request(),LoadedJointTestProfile(profile()),snapshot(),10.,.1);
  auto control=start_request();
  JointTestPermitGate gate(instance,1,7,control.permit_challenge,10.,.1);
  JointTestPlayback playback(plan.trajectory(),{1,7},.2,.05);
  std::vector<double> target=plan.snapshot().commands;
  const bool ready=plan.check_apply(snapshot(),10.01)==JointTestApplyCheck::ready;
  const auto decision=gate.consume(control,10.01,ready);
  ASSERT_TRUE(decision.dispatch);
  ASSERT_TRUE(playback.begin({1,7},10.01));
  ASSERT_TRUE(playback.tick(10.01,true,true,target));
  EXPECT_EQ(target,plan.snapshot().commands);
  EXPECT_FALSE(gate.consume(control,10.02,true).dispatch);
  EXPECT_FALSE(playback.begin({1,7},10.02));
  ASSERT_TRUE(playback.tick(10.03,true,true,target));
  EXPECT_NE(target,plan.snapshot().commands);
  const auto last=target;
  ASSERT_TRUE(playback.cancel({1,7}));
  EXPECT_FALSE(playback.tick(10.04,true,true,target));
  EXPECT_EQ(target,last);
  EXPECT_FALSE(gate.consume(control,10.04,true).dispatch);
}

TEST(JointExecution, PartialWritesLatchFaultAndRetainEachLastAcknowledgedTarget)
{
  auto request=early_excitation_request();
  request.joint_name="hip";  // First write moves; second write deliberately fails.
  PreparedJointTestPlan plan(request,LoadedJointTestProfile(profile()),snapshot(),10.,.1);
  JointTestExecution execution(plan,7,.2,.05);
  std::vector<double> hardware=snapshot().commands;
  const auto write=[&](size_t i,double value) {hardware[i]=value; return true;};
  EXPECT_FALSE(execution.update(10.,true,true,write));
  ASSERT_TRUE(execution.begin(snapshot(),plan.profile().value().sha256,10.));
  EXPECT_FALSE(execution.begin(snapshot(),plan.profile().value().sha256,10.));
  ASSERT_TRUE(execution.update(10.01,true,true,write));
  const auto prior=hardware;
  const auto fail_second=[&](size_t i,double value) {
    if (i==1) {return false;}
    hardware[i]=value; return true;
  };
  EXPECT_FALSE(execution.update(10.02,true,true,fail_second));
  EXPECT_TRUE(execution.write_fault());
  EXPECT_NE(hardware[0],prior[0]);
  EXPECT_EQ(hardware[1],prior[1]);
  EXPECT_EQ(execution.applied_targets(),hardware);
  const auto partial=hardware;
  ASSERT_TRUE(execution.update(10.03,true,true,write));
  EXPECT_EQ(hardware,partial); // Hold the actual mixed write result, not the proposed vector.
  EXPECT_TRUE(execution.write_fault());
  EXPECT_FALSE(execution.renew({1,7},10.03));
}

TEST(JointExecution, ChangedProfileRejectsStartAndCancellationHoldsAppliedTargets)
{
  PreparedJointTestPlan plan(early_excitation_request(),LoadedJointTestProfile(profile()),snapshot(),10.,.1);
  JointTestExecution rejected(plan,7,.2,.05);
  EXPECT_FALSE(rejected.begin(snapshot(),std::string(64,'a'),10.));
  EXPECT_FALSE(rejected.begin(snapshot(),plan.profile().value().sha256,10.));
  JointTestExecution execution(plan,8,.2,.05);
  ASSERT_TRUE(execution.begin(snapshot(),plan.profile().value().sha256,10.));
  std::vector<double> hardware(2);
  const auto write=[&](size_t i,double value) {hardware[i]=value; return true;};
  ASSERT_TRUE(execution.update(10.01,true,true,write));
  const auto prior=hardware;
  EXPECT_FALSE(execution.cancel({1,7}));
  ASSERT_TRUE(execution.cancel({1,8}));
  ASSERT_TRUE(execution.update(10.02,true,true,write));
  EXPECT_EQ(hardware,prior);
  EXPECT_EQ(execution.reason(),JointTestStop::cancelled);
}

TEST(JointExecution, ReadinessAndTimingFaultsHoldWrittenTargetsWithoutAutomaticResume)
{
  for (int fault=0; fault<3; ++fault) {
    SCOPED_TRACE(fault);
    PreparedJointTestPlan plan(early_excitation_request(),LoadedJointTestProfile(profile()),snapshot(),10.,.1);
    JointTestExecution execution(plan,7,.2,.05);
    ASSERT_TRUE(execution.begin(snapshot(),plan.profile().value().sha256,10.));
    auto hardware=snapshot().commands;
    size_t writes=0;
    const auto write=[&](size_t i,double value) {hardware[i]=value; ++writes; return true;};
    ASSERT_TRUE(execution.update(10.02,true,true,write));
    ASSERT_NE(hardware,snapshot().commands);
    const auto prior=hardware;
    const double fault_time=fault==2 ? 10.1 : 10.03;
    writes=0;
    ASSERT_TRUE(execution.update(fault_time,fault!=0,fault!=1,write));
    EXPECT_EQ(writes,hardware.size()); // Terminal hold still writes every joint.
    EXPECT_EQ(hardware,prior);
    EXPECT_EQ(execution.reason(),fault==2 ? JointTestStop::clock : JointTestStop::readiness);
    EXPECT_FALSE(execution.renew({1,7},fault_time+.001));
    ASSERT_TRUE(execution.update(fault_time+.01,true,true,write));
    EXPECT_EQ(hardware,prior); // Restored readiness cannot resume this session.
    EXPECT_FALSE(execution.write_fault());
  }
}

class JointPlanController : public ::testing::Test
{
protected:
  void SetUp() override {
    const char* domain=std::getenv("ROS_DOMAIN_ID");
    const char* local=std::getenv("ROS_LOCALHOST_ONLY");
    if (!domain || std::string(domain)!="231" || !local || std::string(local)!="1") {
      GTEST_SKIP()<<"Requires private localhost ROS domain 231";
    }
    rclcpp::init(0,nullptr);
  }
  void TearDown() override {if (rclcpp::ok()) {rclcpp::shutdown();}}
};

TEST_F(JointPlanController, CachedStatePreparesPlanAndReactivationInvalidatesIt)
{
  ExperimentalJointHoldController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("joints",std::vector<std::string>{"hip","calf"}),
    rclcpp::Parameter("max_handover_error",std::vector<double>{.05,.05}),
    rclcpp::Parameter("feedback_receipt_interface","receipt"),
    rclcpp::Parameter("max_feedback_age_s",1.)}); // Synthetic limit only.
  ASSERT_EQ(controller.init("plan_integration","",200,"",options),controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  const auto now=[]() {return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();};
  double commands[2]={.01,.01}, positions[2]={0.,0.}, receipts[2]={now(),now()};
  std::vector<hardware_interface::CommandInterface> command_handles;
  std::vector<hardware_interface::StateInterface> state_handles;
  command_handles.reserve(2); state_handles.reserve(4);
  const std::vector<std::string> names={"hip","calf"};
  for (size_t i=0;i<2;++i) {
    command_handles.emplace_back(names[i],"position",&commands[i]);
    state_handles.emplace_back(names[i],"position",&positions[i]);
  }
  for (size_t i=0;i<2;++i) {state_handles.emplace_back(names[i],"receipt",&receipts[i]);}
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  for (auto& handle:command_handles) {loaned_commands.emplace_back(handle);}
  for (auto& handle:state_handles) {loaned_states.emplace_back(handle);}
  controller.assign_interfaces(std::move(loaned_commands),std::move(loaned_states));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  JointTestSnapshot captured;
  ASSERT_TRUE(controller.read_plan_snapshot(captured));
  auto r=hashed_request(); r.controller_instance_id=captured.instance;
  r.activation_generation=captured.activation;
  PreparedJointTestPlan plan(r,LoadedJointTestProfile(profile()),captured,now(),1.);
  EXPECT_EQ(plan.check_apply(captured,now()),JointTestApplyCheck::ready);
  EXPECT_EQ(plan.trajectory().initial_targets(),captured.commands);

  receipts[0]=now()-2.;
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  ASSERT_TRUE(controller.read_plan_snapshot(captured));
  EXPECT_EQ(plan.check_apply(captured,now()),JointTestApplyCheck::not_holding);
  EXPECT_THROW(PreparedJointTestPlan(r,LoadedJointTestProfile(profile()),captured,now(),1.),JointTestAdmissionError);
  controller.on_deactivate(rclcpp_lifecycle::State());
  EXPECT_FALSE(controller.read_plan_snapshot(captured));
  receipts[0]=receipts[1]=now();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  ASSERT_TRUE(controller.read_plan_snapshot(captured));
  EXPECT_EQ(captured.instance,r.controller_instance_id);
  EXPECT_GT(captured.activation,r.activation_generation);
  EXPECT_EQ(plan.check_apply(captured,now()),JointTestApplyCheck::stale_controller);
  EXPECT_THROW(PreparedJointTestPlan(r,LoadedJointTestProfile(profile()),captured,now(),1.),JointTestAdmissionError);
  controller.on_deactivate(rclcpp_lifecycle::State());
  controller.release_interfaces();
}

TEST_F(JointPlanController, TypedStateServiceIsOptInAndReadOnly)
{
  using Service=runtime_interfaces::srv::GetJointTestState;
  ExperimentalJointHoldController enabled, disabled;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("enable_joint_test_state_service",true),
    rclcpp::Parameter("joints",std::vector<std::string>{"hip"}),
    rclcpp::Parameter("max_handover_error",std::vector<double>{.05}),
    rclcpp::Parameter("feedback_receipt_interface","receipt"),
    rclcpp::Parameter("max_feedback_age_s",.25)});
  ASSERT_EQ(enabled.init("state_enabled","",200,"",options),controller_interface::return_type::OK);
  ASSERT_EQ(disabled.init("state_disabled","",200,"",rclcpp::NodeOptions()),controller_interface::return_type::OK);
  auto node=std::make_shared<rclcpp::Node>("state_probe");
  auto client=node->create_client<Service>("/state_enabled/joint_test/state");
  auto absent=node->create_client<Service>("/state_disabled/joint_test/state");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); executor.add_node(enabled.get_node()->get_node_base_interface());
  executor.add_node(disabled.get_node()->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  EXPECT_FALSE(absent->wait_for_service(std::chrono::milliseconds(50)));
  auto first=client->async_send_request(std::make_shared<Service::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(first,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
  const auto state=first.get()->state;
  EXPECT_TRUE(canonical_uuid(state.controller_instance_id));
  EXPECT_EQ(state.phase,"sample_unavailable");
  EXPECT_FALSE(state.motion_available);
  EXPECT_FALSE(state.hold_applied);
  EXPECT_TRUE(state.permit_challenge.empty());
  auto second=client->async_send_request(std::make_shared<Service::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(second,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(second.get()->state,state);
  const auto read=[&]() {
    auto future=client->async_send_request(std::make_shared<Service::Request>());
    if (executor.spin_until_future_complete(future,std::chrono::seconds(2))!=rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("State service timed out");
    }
    return future.get()->state;
  };
  const auto now=[]() {return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();};
  ASSERT_EQ(enabled.on_configure(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  double target=.01, position=0., receipt=now();
  hardware_interface::CommandInterface command("hip","position",&target);
  hardware_interface::StateInterface measured("hip","position",&position), stamp("hip","receipt",&receipt);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  commands.emplace_back(command); states.emplace_back(measured); states.emplace_back(stamp);
  enabled.assign_interfaces(std::move(commands),std::move(states));
  ASSERT_EQ(enabled.on_activate(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  enabled.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  const auto fresh=read();
  EXPECT_EQ(fresh.phase,"holding");
  EXPECT_TRUE(fresh.feedback_receipt_verified);
  EXPECT_TRUE(fresh.hold_applied);
  EXPECT_FALSE(fresh.feedback_fault_latched);
  EXPECT_FALSE(fresh.motion_available);
  EXPECT_EQ(read(),fresh);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  const auto stale=read(); // No controller update: only query-time age can catch this.
  EXPECT_EQ(stale.phase,"sample_unavailable");
  EXPECT_FALSE(stale.feedback_receipt_verified);
  EXPECT_FALSE(stale.hold_applied);
  EXPECT_EQ(stale.revision,0u); // Expired snapshots are no longer returned.
  EXPECT_TRUE(stale.feedback_fault_latched); // Conservative unavailable response, not a latch mutation.
  enabled.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_TRUE(read().feedback_fault_latched);
  receipt=now();
  enabled.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  const auto recovered=read();
  EXPECT_TRUE(recovered.feedback_receipt_verified);
  EXPECT_TRUE(recovered.feedback_fault_latched);
  EXPECT_FALSE(recovered.motion_available);
  enabled.on_deactivate(rclcpp_lifecycle::State());
  EXPECT_EQ(read().phase,"sample_unavailable");
  EXPECT_FALSE(read().hold_applied);
  enabled.release_interfaces();
}

TEST_F(JointPlanController, OptInRosStartIsOneUseAndAcknowledgedOnlyByUpdate)
{
  ExperimentalJointHoldController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("enable_joint_test_preparation_service",true),
    rclcpp::Parameter("enable_joint_test_state_service",true),
    rclcpp::Parameter("enable_experimental_joint_playback",true),
    rclcpp::Parameter("enable_experimental_joint_start",true),
    rclcpp::Parameter("joint_test_lease_s",.15),
    rclcpp::Parameter("joint_test_max_update_gap_s",.1),
    rclcpp::Parameter("joints",std::vector<std::string>{"hip"}),
    rclcpp::Parameter("max_handover_error",std::vector<double>{.05}),
    rclcpp::Parameter("feedback_receipt_interface","receipt"),
    rclcpp::Parameter("max_feedback_age_s",.5),
    rclcpp::Parameter("joint_test_profile.enabled",true),
    rclcpp::Parameter("joint_test_profile.id","synthetic"),
    rclcpp::Parameter("joint_test_profile.lower_rad",std::vector<double>{-1.}),
    rclcpp::Parameter("joint_test_profile.upper_rad",std::vector<double>{1.}),
    rclcpp::Parameter("joint_test_profile.speed_rad_s",std::vector<double>{.1}),
    rclcpp::Parameter("joint_test_profile.acceleration_rad_s2",std::vector<double>{.2}),
    rclcpp::Parameter("joint_test_profile.max_amplitude_rad",std::vector<double>{.02}),
    rclcpp::Parameter("joint_test_profile.max_total_seconds",5.),
    rclcpp::Parameter("joint_test_profile.max_cycles",int64_t{2})});
  ASSERT_EQ(controller.init("start_test","",200,"",options),controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  const auto now=[]() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();};
  double target=.01,position=0.,receipt=now();
  hardware_interface::CommandInterface command("hip","position",&target);
  hardware_interface::StateInterface measured("hip","position",&position),stamp("hip","receipt",&receipt);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  commands.emplace_back(command); states.emplace_back(measured); states.emplace_back(stamp);
  controller.assign_interfaces(std::move(commands),std::move(states));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  const auto update=[&]() {receipt=now(); controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));};
  update();
  JointTestSnapshot snapshot;
  ASSERT_TRUE(controller.read_plan_snapshot(snapshot));
  auto node=std::make_shared<rclcpp::Node>("start_probe");
  using Prepare=runtime_interfaces::srv::PrepareJointTest;
  using Control=runtime_interfaces::srv::ControlJointTest;
  using State=runtime_interfaces::srv::GetJointTestState;
  auto prepare=node->create_client<Prepare>("/start_test/joint_test/prepare");
  auto control=node->create_client<Control>("/start_test/joint_test/control");
  auto state=node->create_client<State>("/start_test/joint_test/state");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); executor.add_node(controller.get_node()->get_node_base_interface());
  const auto call=[&](auto client,auto request) {
    auto future=client->async_send_request(request);
    if (executor.spin_until_future_complete(future,std::chrono::seconds(2))!=rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Joint-test service timed out");
    }
    return *future.get();
  };
  const auto read=[&]() {return call(state,std::make_shared<State::Request>()).state;};
  auto r=std::make_shared<Request>(request());
  r->controller_instance_id=snapshot.instance; r->activation_generation=snapshot.activation;
  r->profile_id=controller.test_profile()->value().id;
  r->profile_sha256=controller.test_profile()->value().sha256; r->joint_name="hip";
  r->transition_duration_s=.001;
  const auto prepared=call(prepare,r);
  ASSERT_TRUE(prepared.accepted)<<prepared.error_code;
  ASSERT_TRUE(canonical_uuid(prepared.state.permit_challenge));
  EXPECT_EQ(call(prepare,r),prepared);
  auto observed=read();
  EXPECT_TRUE(observed.motion_available);
  EXPECT_EQ(observed.permit_challenge,prepared.state.permit_challenge);
  EXPECT_LE(read().lease_remaining_s,observed.lease_remaining_s); // Reading cannot renew.
  auto start=std::make_shared<Control::Request>(start_request());
  start->controller_instance_id=snapshot.instance; start->activation_generation=snapshot.activation;
  start->session_id=prepared.state.session_id; start->permit_challenge=prepared.state.permit_challenge;
  start->operation_id="12345678-1234-4234-8234-123456789ac1";
  const auto queued=call(control,start);
  ASSERT_TRUE(queued.accepted)<<queued.error_code;
  EXPECT_EQ(queued.state.phase,"start_pending");
  EXPECT_EQ(read().phase,"start_pending");
  EXPECT_EQ(read().applied_sequence,0u);
  EXPECT_FALSE(read().motion_available);
  EXPECT_DOUBLE_EQ(target,.01);
  EXPECT_EQ(call(control,start),queued);
  start->sequence=2;
  EXPECT_EQ(call(control,start).error_code,"OPERATION_CONFLICT");
  start->operation_id="12345678-1234-4234-8234-123456789ac2";
  EXPECT_EQ(call(control,start).error_code,"PERMIT_CONSUMED");
  update();
  EXPECT_EQ(read().phase,"executing");
  EXPECT_EQ(read().applied_sequence,1u);
  JointHoldSample initial_execution;
  ASSERT_TRUE(controller.read_sample(initial_execution));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  update();
  EXPECT_NE(target,.01);
  for (uint64_t sequence=2;sequence<=8;++sequence) {
    update();
    const auto current=read();
    ASSERT_EQ(current.phase,"executing");
    ASSERT_FALSE(current.permit_challenge.empty());
    auto renew=std::make_shared<Control::Request>(*start);
    renew->action=Control::Request::RENEW; renew->sequence=sequence;
    char operation[37];
    std::snprintf(operation,sizeof(operation),"12345678-1234-4234-8234-%012llu",
      static_cast<unsigned long long>(100+sequence));
    renew->operation_id=operation; renew->permit_challenge=current.permit_challenge;
    const auto pending=call(control,renew);
    ASSERT_TRUE(pending.accepted)<<pending.error_code;
    EXPECT_EQ(pending.state.phase,"renewal_pending");
    EXPECT_EQ(read().applied_sequence,sequence-1);
    EXPECT_EQ(read().phase,"renewal_pending");
    EXPECT_TRUE(read().permit_challenge.empty());
    EXPECT_EQ(call(control,renew),pending);
    update();
    EXPECT_EQ(read().applied_sequence,sequence);
    EXPECT_NE(read().permit_challenge,renew->permit_challenge);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  update();
  EXPECT_GT(now(),initial_execution.execution_permit_until);
  EXPECT_EQ(read().phase,"executing"); // Renewals extend the initial permission, not the trajectory duration.
  const double prior=target;
  auto queued_renewal=std::make_shared<Control::Request>(*start);
  queued_renewal->action=Control::Request::RENEW; queued_renewal->sequence=9;
  queued_renewal->operation_id="12345678-1234-4234-8234-123456789ac5";
  queued_renewal->permit_challenge=read().permit_challenge;
  ASSERT_TRUE(call(control,queued_renewal).accepted);
  start->action=Control::Request::CANCEL;
  start->operation_id="12345678-1234-4234-8234-123456789ac3";
  ASSERT_TRUE(call(control,start).accepted);
  update();
  EXPECT_EQ(read().phase,"cancelled");
  EXPECT_DOUBLE_EQ(target,prior);
  JointHoldSample cancelled_execution;
  ASSERT_TRUE(controller.read_sample(cancelled_execution));
  EXPECT_EQ(cancelled_execution.execution_sequence,8u); // Cancellation wins over queued renewal.
  r->operation_id="12345678-1234-4234-8234-123456789ab2";
  const auto second=call(prepare,r);
  ASSERT_TRUE(second.accepted);
  EXPECT_NE(second.state.permit_challenge,prepared.state.permit_challenge);
  std::this_thread::sleep_for(std::chrono::milliseconds(180));
  update();
  EXPECT_FALSE(read().motion_available);
  EXPECT_TRUE(read().permit_challenge.empty());
  start->action=Control::Request::START; start->session_id=second.state.session_id;
  start->permit_challenge=second.state.permit_challenge;
  start->operation_id="12345678-1234-4234-8234-123456789ac4";
  EXPECT_EQ(call(control,start).error_code,"PERMIT_EXPIRED");
  EXPECT_DOUBLE_EQ(target,prior);
  controller.on_deactivate(rclcpp_lifecycle::State());
  controller.release_interfaces();
}

TEST_F(JointPlanController, RosStartRequiresExplicitTimingAndPlaybackOptIn)
{
  for (int invalid=0;invalid<3;++invalid) {
    SCOPED_TRACE(invalid);
    ExperimentalJointHoldController controller;
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("enable_experimental_joint_start",true),
      rclcpp::Parameter("enable_experimental_joint_playback",invalid!=0),
      rclcpp::Parameter("enable_joint_test_preparation_service",true),
      rclcpp::Parameter("enable_joint_test_state_service",true),
      rclcpp::Parameter("joint_test_lease_s",invalid==1 ? 0. : .15),
      rclcpp::Parameter("joint_test_max_update_gap_s",invalid==2 ? std::numeric_limits<double>::infinity() : .1)});
    EXPECT_EQ(controller.init("invalid_start_test","",200,"",options),controller_interface::return_type::ERROR);
  }
}

TEST_F(JointPlanController, ConfiguredProfileIsValidatedAndImmutable)
{
  ExperimentalJointHoldController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("enable_joint_test_preparation_service",true),
    rclcpp::Parameter("enable_experimental_joint_playback",true),
    rclcpp::Parameter("enable_joint_test_state_service",true),
    rclcpp::Parameter("joints",std::vector<std::string>{"hip"}),
    rclcpp::Parameter("max_handover_error",std::vector<double>{.05}),
    rclcpp::Parameter("feedback_receipt_interface","receipt"),
    rclcpp::Parameter("max_feedback_age_s",.25),
    rclcpp::Parameter("joint_test_profile.enabled",true),
    rclcpp::Parameter("joint_test_profile.id","synthetic"),
    rclcpp::Parameter("joint_test_profile.lower_rad",std::vector<double>{-1.}),
    rclcpp::Parameter("joint_test_profile.upper_rad",std::vector<double>{1.}),
    rclcpp::Parameter("joint_test_profile.speed_rad_s",std::vector<double>{.1}),
    rclcpp::Parameter("joint_test_profile.acceleration_rad_s2",std::vector<double>{.2}),
    rclcpp::Parameter("joint_test_profile.max_amplitude_rad",std::vector<double>{.02}),
    rclcpp::Parameter("joint_test_profile.max_total_seconds",5.),
    rclcpp::Parameter("joint_test_profile.max_cycles",int64_t{2})});
  ASSERT_EQ(controller.init("profile_test","",200,"",options),controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  const auto loaded=controller.test_profile();
  ASSERT_NE(loaded,nullptr);
  EXPECT_EQ(loaded->value().joints,std::vector<std::string>{"hip"});
  EXPECT_DOUBLE_EQ(loaded->value().bounds[0].handover_error,.05);
  EXPECT_TRUE(lowercase_hex(loaded->value().sha256,64));
  using Service=runtime_interfaces::srv::PrepareJointTest;
  auto node=std::make_shared<rclcpp::Node>("prepare_probe");
  auto client=node->create_client<Service>("/profile_test/joint_test/prepare");
  using Control=runtime_interfaces::srv::ControlJointTest;
  using State=runtime_interfaces::srv::GetJointTestState;
  auto control_client=node->create_client<Control>("/profile_test/joint_test/control");
  auto state_client=node->create_client<State>("/profile_test/joint_test/state");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); executor.add_node(controller.get_node()->get_node_base_interface());
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));
  ASSERT_TRUE(control_client->wait_for_service(std::chrono::seconds(2)));
  ASSERT_TRUE(state_client->wait_for_service(std::chrono::seconds(2)));
  const auto call=[&](const Request& r) {
    auto future=client->async_send_request(std::make_shared<Request>(r));
    if (executor.spin_until_future_complete(future,std::chrono::seconds(2))!=rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Prepare service timed out");
    }
    return *future.get();
  };
  auto r=request(); r.profile_id=loaded->value().id; r.profile_sha256=loaded->value().sha256; r.joint_name="hip";
  const auto missing=call(r);
  EXPECT_FALSE(missing.accepted); EXPECT_EQ(missing.error_code,"SNAPSHOT_UNAVAILABLE");
  double target=.01, position=0., receipt=std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  hardware_interface::CommandInterface command("hip","position",&target);
  hardware_interface::StateInterface measured("hip","position",&position), stamp("hip","receipt",&receipt);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  std::vector<hardware_interface::LoanedStateInterface> states;
  commands.emplace_back(command); states.emplace_back(measured); states.emplace_back(stamp);
  controller.assign_interfaces(std::move(commands),std::move(states));
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_EQ(call(r),missing); // Failed operation does not silently become successful.
  JointTestSnapshot state;
  ASSERT_TRUE(controller.read_plan_snapshot(state));
  r.controller_instance_id=state.instance; r.activation_generation=state.activation;
  r.operation_id="12345678-1234-4234-8234-123456789ab1";
  const auto prepared=call(r);
  ASSERT_TRUE(prepared.accepted)<<prepared.error_code;
  EXPECT_EQ(prepared.state.phase,"prepared");
  EXPECT_FALSE(prepared.state.motion_available);
  EXPECT_TRUE(prepared.state.permit_challenge.empty());
  EXPECT_EQ(call(r),prepared);
  EXPECT_DOUBLE_EQ(target,.01); // Preparation did not issue a new target.
  r.amplitude_rad=.01;
  EXPECT_EQ(call(r).error_code,"OPERATION_CONFLICT");
  r.operation_id="12345678-1234-4234-8234-123456789ab2";
  EXPECT_EQ(call(r).error_code,"PREPARATION_BUSY");
  const auto read_state=[&]() {
    auto future=state_client->async_send_request(std::make_shared<State::Request>());
    if (executor.spin_until_future_complete(future,std::chrono::seconds(2))!=rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("State service timed out");
    }
    return future.get()->state;
  };
  EXPECT_EQ(read_state().session_id,prepared.state.session_id);
  EXPECT_EQ(read_state().phase,"prepared");
  const auto control_call=[&](const Control::Request& value) {
    auto future=control_client->async_send_request(std::make_shared<Control::Request>(value));
    if (executor.spin_until_future_complete(future,std::chrono::seconds(2))!=rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Control service timed out");
    }
    return *future.get();
  };
  auto cancel=start_request(); cancel.action=cancel.CANCEL;
  cancel.controller_instance_id=state.instance; cancel.activation_generation=state.activation;
  cancel.session_id=prepared.state.session_id; cancel.permit_challenge.clear(); cancel.sequence=41;
  cancel.operation_id="12345678-1234-4234-8234-123456789ac1";
  const auto cancelled=control_call(cancel);
  ASSERT_TRUE(cancelled.accepted)<<cancelled.error_code;
  EXPECT_EQ(cancelled.state.phase,"cancel_pending");
  EXPECT_EQ(read_state().phase,"cancel_pending");
  EXPECT_EQ(read_state().applied_sequence,0u);
  auto early=r; early.operation_id="12345678-1234-4234-8234-123456789aa9";
  EXPECT_EQ(call(early).error_code,"PREPARATION_BUSY");
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_EQ(read_state().phase,"cancelled");
  EXPECT_EQ(read_state().applied_sequence,41u);
  EXPECT_EQ(control_call(cancel),cancelled);
  EXPECT_DOUBLE_EQ(target,.01);
  r.operation_id="12345678-1234-4234-8234-123456789ab3";
  const auto replacement=call(r);
  ASSERT_TRUE(replacement.accepted)<<replacement.error_code;
  EXPECT_GT(replacement.state.session_id,prepared.state.session_id);
  EXPECT_EQ(control_call(cancel),cancelled); // Historical retry cannot cancel replacement.
  EXPECT_EQ(read_state().phase,"prepared");
  cancel.operation_id="12345678-1234-4234-8234-123456789ac2";
  EXPECT_EQ(control_call(cancel).error_code,"STALE_SESSION");
  cancel.session_id=replacement.state.session_id; cancel.action=cancel.START;
  cancel.operation_id="12345678-1234-4234-8234-123456789ac3";
  EXPECT_EQ(control_call(cancel).error_code,"EXPERIMENTAL_MOTION_DISABLED");
  bool capacity_seen=false;
  for (unsigned i=0;i<4098;++i) {
    char operation[37];
    std::snprintf(operation,sizeof(operation),"12345678-1234-4234-8234-%012x",i+1000);
    cancel.operation_id=operation;
    const auto result=control_call(cancel);
    EXPECT_FALSE(result.accepted);
    if (result.error_code=="OPERATION_CAPACITY") {capacity_seen=true;}
    else {EXPECT_EQ(result.error_code,"EXPERIMENTAL_MOTION_DISABLED");}
  }
  ASSERT_TRUE(capacity_seen);
  cancel.action=cancel.CANCEL;
  cancel.operation_id="12345678-1234-4234-8234-123456789ac4";
  const auto full_cancel=control_call(cancel);
  ASSERT_TRUE(full_cancel.accepted)<<full_cancel.error_code;
  EXPECT_EQ(read_state().phase,"cancel_pending");
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_EQ(read_state().phase,"cancelled");
  EXPECT_EQ(control_call(cancel),full_cancel);
  EXPECT_DOUBLE_EQ(target,.01);
  cancel.sequence++;
  EXPECT_EQ(control_call(cancel).error_code,"OPERATION_CONFLICT");
  receipt=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  r.operation_id="12345678-1234-4234-8234-123456789ab4";
  r.transition_duration_s=.001;
  const auto next=call(r);
  ASSERT_TRUE(next.accepted)<<next.error_code;
  EXPECT_FALSE(controller.stage_prepared_execution(next.state.session_id+1,71,.2,.1));
  ASSERT_TRUE(controller.stage_prepared_execution(next.state.session_id,71,.2,.1));
  EXPECT_FALSE(controller.stage_prepared_execution(next.state.session_id,71,.2,.1));
  const double before_playback=target;
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(target,before_playback); // Applied start is elapsed zero.
  EXPECT_EQ(read_state().applied_sequence,71u);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  receipt=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_NE(target,before_playback);
  EXPECT_EQ(read_state().phase,"executing");
  cancel.sequence=41;
  EXPECT_EQ(control_call(cancel),full_cancel);
  EXPECT_EQ(read_state().phase,"executing");
  cancel.session_id=next.state.session_id;
  cancel.operation_id="12345678-1234-4234-8234-123456789ac5";
  ASSERT_TRUE(control_call(cancel).accepted);
  const auto moving_target=target;
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(target,moving_target);
  EXPECT_EQ(read_state().phase,"cancelled");
  EXPECT_EQ(read_state().applied_sequence,41u);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(target,moving_target);
  r.operation_id="12345678-1234-4234-8234-123456789ab5";
  const auto later=call(r);
  ASSERT_TRUE(later.accepted)<<later.error_code;
  ASSERT_TRUE(controller.stage_prepared_execution(later.state.session_id,72,.2,.1));
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  cancel.session_id=later.state.session_id;
  cancel.operation_id="12345678-1234-4234-8234-123456789ac7";
  const auto pending=control_call(cancel);
  EXPECT_TRUE(pending.accepted); // One allowance for each new session.
  controller.on_deactivate(rclcpp_lifecycle::State());
  EXPECT_EQ(control_call(cancel),pending); // Recorded acceptance is not a new dispatch.
  cancel.operation_id="12345678-1234-4234-8234-123456789ac6";
  EXPECT_EQ(control_call(cancel).error_code,"STALE_ACTIVATION");
  receipt=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  JointTestSnapshot reactivated;
  ASSERT_TRUE(controller.read_plan_snapshot(reactivated));
  EXPECT_GT(reactivated.activation,cancel.activation_generation);
  EXPECT_EQ(reactivated.cancelled_session,0u); // Old mailbox cannot acknowledge in new activation.
  EXPECT_EQ(read_state().applied_sequence,0u);
  EXPECT_EQ(control_call(cancel).error_code,"STALE_ACTIVATION");
  r.activation_generation=reactivated.activation;
  r.operation_id="12345678-1234-4234-8234-123456789ab6";
  const auto lease_plan=call(r);
  ASSERT_TRUE(lease_plan.accepted)<<lease_plan.error_code;
  ASSERT_TRUE(controller.stage_prepared_execution(lease_plan.state.session_id,81,.02,5.));
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  const double lease_target=target;
  EXPECT_EQ(read_state().phase,"executing");
  std::this_thread::sleep_for(std::chrono::milliseconds(35));
  receipt=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(target,lease_target);
  const auto expired=read_state();
  EXPECT_EQ(expired.phase,"execution_hold");
  EXPECT_EQ(expired.reason_code,"LEASE_EXPIRED");
  EXPECT_EQ(expired.applied_sequence,81u);
  EXPECT_TRUE(expired.hold_applied);
  EXPECT_FALSE(controller.stage_prepared_execution(lease_plan.state.session_id,82,.2,.1));
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(target,lease_target);
  cancel.activation_generation=reactivated.activation;
  cancel.session_id=lease_plan.state.session_id;
  cancel.operation_id="12345678-1234-4234-8234-123456789ac8";
  ASSERT_TRUE(control_call(cancel).accepted); // Cancellation remains available after expiry.
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  EXPECT_EQ(read_state().phase,"cancelled");
  EXPECT_DOUBLE_EQ(target,lease_target);
  receipt=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  r.operation_id="12345678-1234-4234-8234-123456789ab7";
  const auto queued_plan=call(r);
  ASSERT_TRUE(queued_plan.accepted)<<queued_plan.error_code;
  ASSERT_TRUE(controller.stage_prepared_execution(queued_plan.state.session_id,91,.02,5.,receipt-.03));
  controller.update(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005));
  const auto rejected_start=read_state();
  EXPECT_EQ(rejected_start.phase,"start_rejected");
  EXPECT_EQ(rejected_start.reason_code,"LEASE_EXPIRED");
  EXPECT_EQ(rejected_start.applied_sequence,0u);
  EXPECT_TRUE(rejected_start.hold_applied);
  EXPECT_DOUBLE_EQ(target,lease_target);
  EXPECT_FALSE(controller.stage_prepared_execution(queued_plan.state.session_id,92,.2,.1));
  controller.on_deactivate(rclcpp_lifecycle::State());
  controller.release_interfaces();
  ASSERT_TRUE(controller.get_node()->set_parameter(rclcpp::Parameter("joint_test_profile.speed_rad_s",std::vector<double>{0.})).successful);
  EXPECT_DOUBLE_EQ(controller.test_profile()->value().bounds[0].speed,.1); // No implicit hot reload.
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::ERROR);
  EXPECT_EQ(controller.test_profile(),nullptr);
  EXPECT_DOUBLE_EQ(loaded->value().bounds[0].speed,.1); // Existing snapshots stay immutable.
}
