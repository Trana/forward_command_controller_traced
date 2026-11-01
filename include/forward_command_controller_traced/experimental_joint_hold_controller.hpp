#pragma once

#include <vector>
#include <string>
#include <atomic>
#include <array>
#include <mutex>
#include <memory>
#include <map>
#include <thread>
#include "forward_command_controller_traced/bounded_telemetry_queue.hpp"
#include "forward_command_controller_traced/joint_sample_validity.hpp"
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#if __has_include("realtime_tools/realtime_publisher.hpp")
#include "realtime_tools/realtime_publisher.hpp"
#else
#include "realtime_tools/realtime_publisher.h"
#endif
#include "forward_command_controller_traced/joint_test_plan.hpp"
#include "forward_command_controller_traced/joint_test_execution.hpp"
#include "forward_command_controller_traced/joint_test_permit.hpp"
#include "runtime_interfaces/srv/get_joint_test_state.hpp"
#include "runtime_interfaces/srv/prepare_joint_test.hpp"
#include "runtime_interfaces/srv/control_joint_test.hpp"
#include "runtime_interfaces/msg/joint_test_target_sample.hpp"

namespace forward_command_controller_traced
{
struct JointHoldSample
{
  uint64_t status_word = 0, activation = 0;
  uint64_t cancelled_session=0, applied_sequence=0;
  uint64_t execution_session=0, execution_sequence=0;
  bool execution_started=false;
  JointTestStop execution_stop=JointTestStop::none;
  double execution_permit_until=0.;
  double sampled_at = 0;
  double max_feedback_age = 0;
  size_t count = 0;
  std::array<double,256> commands{}, positions{}, receipts{};
  std::array<char,37> instance{};
  std::array<std::array<char,65>,256> joints{};
};
// Experimental exclusive hold/playback foundation. ROS start requires separate
// startup opt-in and trusted network isolation. No physical-safety claim.
class ExperimentalJointHoldController : public controller_interface::ControllerInterface
{
public:
  ~ExperimentalJointHoldController() override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
  bool tracking_fault_latched() const {return tracking_fault_.load();}
  bool feedback_fault_latched() const {return feedback_fault_.load();}
  // Non-realtime, cache-only read. False means no current complete sample.
  bool read_sample(JointHoldSample& output);
  bool read_plan_snapshot(JointTestSnapshot& output);
  std::shared_ptr<const LoadedJointTestProfile> test_profile() const
  {return std::atomic_load(&test_profile_);}
  // Internal management adapter only, serialized with the service callback group.
  // Caller must consume an authenticated one-use permit before staging.
  bool stage_prepared_execution(uint64_t session,uint64_t sequence,double lease,double max_gap);
  bool stage_prepared_execution(uint64_t session,uint64_t sequence,double lease,double max_gap,double issued_at);
private:
  struct ExecutionSlot {
    ExecutionSlot(std::shared_ptr<const PreparedJointTestPlan> p,uint64_t id,uint64_t seq,double lease,double gap,double issued)
    : plan(std::move(p)), execution(*plan,id,lease,gap), live(plan->snapshot()), session(id), sequence(seq), issued_at(issued) {}
    std::shared_ptr<const PreparedJointTestPlan> plan;
    JointTestExecution execution;
    JointTestSnapshot live;
    uint64_t session,sequence;
    const double issued_at;
    uint64_t renewal_sequence=0;
    double renewal_issued_at=0.;
    bool attempted=false,begun=false;
  };
  bool playback_enabled_=false;
  bool short_tracking_report_only_=false;
  bool remote_start_enabled_=false;
  double remote_lease_=0., remote_max_gap_=0.;
  std::unique_ptr<JointTestPermitGate> start_gate_, renewal_gate_; // Management callbacks only.
  uint64_t requested_sequence_=0;
  bool renewal_ready(const JointHoldSample&,double now) const;
  void issue_renewal_challenge(uint64_t sequence,double now);
  bool start_queued_=false;
  std::mutex execution_mutex_;
  std::unique_ptr<ExecutionSlot> execution_;
  uint64_t execution_session_=0, execution_sequence_=0;
  bool execution_started_=false;
  JointTestStop execution_stop_=JointTestStop::none;
  double execution_permit_until_=0.;
  struct CancelMailbox {uint64_t activation=0, session=0, sequence=0;};
  std::mutex cancel_mutex_;
  CancelMailbox pending_cancel_;
  uint64_t applied_cancel_session_=0, applied_cancel_sequence_=0;
  using PrepareService=runtime_interfaces::srv::PrepareJointTest;
  using ControlService=runtime_interfaces::srv::ControlJointTest;
  void control_test(const ControlService::Request&,ControlService::Response&);
  rclcpp::Service<ControlService>::SharedPtr control_service_;
  std::map<std::string,std::pair<ControlService::Request,ControlService::Response>> controls_;
  bool prepared_cancelled_=false;
  void prepare_test(const PrepareService::Request&,PrepareService::Response&);
  rclcpp::Service<PrepareService>::SharedPtr prepare_service_;
  // Accessed only by the default mutually-exclusive service callback group.
  std::map<std::string,std::pair<PrepareService::Request,PrepareService::Response>> preparations_;
  std::shared_ptr<const PreparedJointTestPlan> prepared_plan_;
  uint64_t prepared_session_=0;
  std::shared_ptr<const LoadedJointTestProfile> test_profile_;
  rclcpp::Service<runtime_interfaces::srv::GetJointTestState>::SharedPtr state_service_;
  std::array<char,37> instance_{};
  std::vector<std::string> joints_;
  void cache_sample();
  std::mutex sample_mutex_;
  JointHoldSample sample_;
  std::atomic<uint64_t> activation_{0};
  void check_tracking();
  void check_feedback();
  void snapshot(bool written = false, bool write_failed = false);
  std::atomic<uint64_t> status_word_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> target_publisher_;
  struct TargetSample {
    uint64_t activation=0, sequence=0;
    double sampled_at=0.;
    int32_t stamp_sec=0;
    uint32_t stamp_nanosec=0;
    std::array<double,256> positions{};
  };
  std::unique_ptr<BoundedTelemetryQueue<TargetSample,256>> target_queue_;
  rclcpp::Publisher<runtime_interfaces::msg::JointTestTargetSample>::SharedPtr sample_publisher_;
  std::thread telemetry_worker_;
  std::atomic<bool> stop_telemetry_{false};
  std::atomic<uint64_t> telemetry_overflow_{0}, telemetry_publish_errors_{0};
  void stop_telemetry();
  uint64_t target_sequence_=0;
  std::vector<std::string> names_;
  std::vector<std::string> state_names_;
  std::string receipt_interface_;
  double max_feedback_age_ = 0.;
  bool feedback_fresh_ = false;
  std::atomic<bool> feedback_fault_{true};
  std::vector<double> max_error_, held_;
  bool active_ = false;
  std::atomic<bool> tracking_fault_{true};
};
}
