#include "forward_command_controller_traced/experimental_joint_hold_controller.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <set>
#include <optional>
#include <type_traits>
#include <openssl/rand.h>
#include "rclcpp/create_publisher.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace forward_command_controller_traced
{
using Return = controller_interface::CallbackReturn;

ExperimentalJointHoldController::~ExperimentalJointHoldController() {stop_telemetry();}
void ExperimentalJointHoldController::stop_telemetry()
{
  stop_telemetry_.store(true);
  if (telemetry_worker_.joinable()) telemetry_worker_.join();
}

template<class T> auto read_value(T & interface, int) -> decltype(interface.get_optional())
{
  return interface.get_optional();
}
template<class T> std::optional<double> read_value(T & interface, long)
{
  return interface.get_value();  // Humble API fallback; validate finiteness below.
}

static bool random_uuid(std::array<char,37>& output)
{
  unsigned char bytes[16];
  if (RAND_bytes(bytes,sizeof(bytes))!=1) {return false;}
  bytes[6]=(bytes[6]&15)|64;
  bytes[8]=(bytes[8]&63)|128;
  const char* hex="0123456789abcdef";
  size_t offset=0;
  for (size_t i=0; i<16; ++i) {
    if (i==4 || i==6 || i==8 || i==10) {output[offset++]='-';}
    output[offset++]=hex[bytes[i]>>4];
    output[offset++]=hex[bytes[i]&15];
  }
  output[36]='\0';
  return true;
}

Return ExperimentalJointHoldController::on_init()
{
  if (!random_uuid(instance_)) {return Return::ERROR;}
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<std::vector<double>>("max_handover_error", {});
  auto_declare<std::string>("feedback_receipt_interface", "");
  auto_declare<double>("max_feedback_age_s", 0.);
  auto_declare<bool>("enable_joint_test_target_telemetry", false);
  playback_enabled_=auto_declare<bool>("enable_experimental_joint_playback",false);
  auto_declare<bool>("joint_test_profile.enabled", false);
  auto_declare<std::string>("joint_test_profile.id", "");
  for (const auto* field : {"lower_rad","upper_rad","speed_rad_s","acceleration_rad_s2","max_amplitude_rad"}) {
    auto_declare<std::vector<double>>(std::string("joint_test_profile.")+field,{});
  }
  auto_declare<double>("joint_test_profile.max_total_seconds",0.);
  auto_declare<int64_t>("joint_test_profile.max_cycles",0);
  auto_declare<double>("joint_test_profile.chirp_start_hz",0.);
  auto_declare<double>("joint_test_profile.chirp_end_hz",0.);
  auto_declare<std::vector<double>>("joint_test_profile.simultaneous_directions",{});
  auto_declare<bool>("enable_joint_test_state_service", false);
  auto_declare<bool>("enable_joint_test_preparation_service",false);
  remote_start_enabled_=auto_declare<bool>("enable_experimental_joint_start",false);
  remote_lease_=auto_declare<double>("joint_test_lease_s",0.);
  remote_max_gap_=auto_declare<double>("joint_test_max_update_gap_s",0.);
  if (remote_start_enabled_ && (!playback_enabled_ ||
      !get_node()->get_parameter("enable_joint_test_preparation_service").as_bool() ||
      !get_node()->get_parameter("enable_joint_test_state_service").as_bool() ||
      !std::isfinite(remote_lease_) || remote_lease_<=0 ||
      !std::isfinite(remote_max_gap_) || remote_max_gap_<=0)) {return Return::ERROR;}
  if (get_node()->get_parameter("enable_joint_test_preparation_service").as_bool()) {
    prepare_service_=get_node()->create_service<PrepareService>("~/joint_test/prepare",
      [this](const PrepareService::Request::SharedPtr request,PrepareService::Response::SharedPtr response) {
        prepare_test(*request,*response);
      });
    control_service_=get_node()->create_service<ControlService>("~/joint_test/control",
      [this](const ControlService::Request::SharedPtr request,ControlService::Response::SharedPtr response) {
        control_test(*request,*response);
      });
  }
  if (get_node()->get_parameter("enable_joint_test_state_service").as_bool()) {
    using Service=runtime_interfaces::srv::GetJointTestState;
    state_service_=get_node()->create_service<Service>("~/joint_test/state",
      [this](const Service::Request::SharedPtr, Service::Response::SharedPtr response) {
        auto& state=response->state;
        state.controller_instance_id=instance_.data();
        // Identity remains valid while inactive even though no hold sample exists.
        state.activation_generation=activation_.load();
        state.motion_available=false;
        state.phase="sample_unavailable";
        state.reason_code=remote_start_enabled_ ? "NOT_READY" : "EXPERIMENTAL_MOTION_DISABLED";
        state.tracking_fault_latched=true;
        state.feedback_fault_latched=true;
        JointHoldSample sample;
        if (!read_sample(sample)) {return;}
        state.activation_generation=sample.activation;
        state.revision=sample.status_word>>8;
        state.tracking_fault_latched=sample.status_word&2;
        state.feedback_fault_latched=sample.status_word&64;
        const double now=std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        bool fresh=sample.max_feedback_age>0 && now>=sample.sampled_at &&
          now-sample.sampled_at<=sample.max_feedback_age;
        for (size_t i=0;i<sample.count;++i) {
          const double receipt=sample.receipts[i];
          fresh=fresh && std::isfinite(receipt) && receipt>0 && receipt<=now &&
            now-receipt<=sample.max_feedback_age;
        }
        state.feedback_receipt_verified=fresh;
        state.hold_applied=fresh && (sample.status_word&4);
        state.phase=fresh ? "holding" : "stale_or_unverified";
        if (prepared_plan_ && prepared_plan_->snapshot().activation==sample.activation) {
          state.session_id=prepared_session_;
          state.phase=prepared_cancelled_ ?
            (sample.cancelled_session==prepared_session_ ? "cancelled" : "cancel_pending") : "prepared_unready";
          if (sample.cancelled_session==prepared_session_) {state.applied_sequence=sample.applied_sequence;}
          else if (sample.execution_session==prepared_session_ && sample.execution_started) {
            state.applied_sequence=sample.execution_sequence;
          }
          if (!prepared_cancelled_) {
            const auto profile=test_profile();
            JointTestSnapshot live;
            if (profile && profile->value().sha256==prepared_plan_->profile().value().sha256 &&
                read_plan_snapshot(live) && prepared_plan_->check_apply(live,now)==JointTestApplyCheck::ready) {
              state.phase=start_queued_ ? "start_pending" : "prepared";
              if (remote_start_enabled_ && start_gate_ && start_gate_->remaining(now)>0.) {
                state.motion_available=true;
                state.reason_code="START_AVAILABLE";
                state.permit_challenge=start_gate_->challenge();
                state.lease_remaining_s=start_gate_->remaining(now);
              }
            }
            if (sample.execution_session==prepared_session_ &&
                (sample.execution_started || sample.execution_stop!=JointTestStop::none)) {
              state.phase=!sample.execution_started ? "start_rejected" :
                (sample.execution_stop==JointTestStop::none ? "executing" : "execution_hold");
              if (sample.execution_started && sample.execution_stop==JointTestStop::none &&
                  requested_sequence_>sample.execution_sequence) {state.phase="renewal_pending";}
              if (sample.execution_started && sample.execution_stop==JointTestStop::none) {
                state.lease_remaining_s=std::max(0.,sample.execution_permit_until-now);
              }
              switch (sample.execution_stop) {
                case JointTestStop::none: state.reason_code="PLAYBACK_ACTIVE"; break;
                case JointTestStop::completed: state.reason_code="TEST_COMPLETED"; break;
                case JointTestStop::cancelled: state.reason_code="CANCELLED"; break;
                case JointTestStop::readiness: state.reason_code="READINESS_LOST"; break;
                case JointTestStop::lease: state.reason_code="LEASE_EXPIRED"; break;
                case JointTestStop::clock: state.reason_code="UPDATE_TIMING_FAULT"; break;
                case JointTestStop::storage: state.reason_code="INVALID_OUTPUT_STORAGE"; break;
              }
              if (renewal_gate_ && renewal_ready(sample,now) && renewal_gate_->remaining(now)>0.) {
                state.permit_challenge=renewal_gate_->challenge();
                state.lease_remaining_s=sample.execution_permit_until-now;
                state.motion_available=true;
              }
            }
          }
        }
      });
  }
  // Publish cached flags outside the realtime update, including while inactive.
  status_publisher_ = rclcpp::create_publisher<std_msgs::msg::String>(get_node(), "~/status", 1);
  status_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(200), [this]() {
    const auto word = status_word_.load();  // One coherent flags/revision snapshot.
    const bool active = word & 1, fault = word & 2, written = word & 4, failed = word & 8;
    const char * phase = !active ? "inactive" : failed ? "write_failed" :
      written ? "holding" : "pending_first_write";
    const char * freshness = !(word & 16) ? "unverified" :
      (word & 32) ? "fresh_receipt" : "stale_or_invalid_receipt";
    std_msgs::msg::String message;
    message.data = std::string("{\"schema\":1,\"phase\":\"") + phase +
      "\",\"revision\":" + std::to_string(word >> 8) +
      ",\"tracking_fault_latched\":" + (fault ? "true" : "false") +
      ",\"feedback_fault_latched\":" + ((word & 64) ? "true" : "false") +
      ",\"target_queue_overflow\":" + std::to_string(telemetry_overflow_.load()) +
      ",\"target_publish_errors\":" + std::to_string(telemetry_publish_errors_.load()) +
      ",\"motion_available\":false,\"measurement_freshness\":\"" + freshness + "\"}";
    status_publisher_->publish(message);
  });
  snapshot();
  return Return::SUCCESS;
}

void ExperimentalJointHoldController::snapshot(bool written, bool write_failed)
{
  const uint64_t flags = (active_ ? 1u : 0u) | (tracking_fault_.load() ? 2u : 0u) |
    (written ? 4u : 0u) | (write_failed ? 8u : 0u) |
    (!receipt_interface_.empty() ? 16u : 0u) | (feedback_fresh_ ? 32u : 0u) |
    (feedback_fault_.load() ? 64u : 0u);
  status_word_.store(((status_word_.load() & ~uint64_t{255}) + 256) | flags);
}

Return ExperimentalJointHoldController::on_configure(const rclcpp_lifecycle::State &)
{
  stop_telemetry();
  target_publisher_.reset();
  sample_publisher_.reset();
  target_queue_.reset();
  std::atomic_store(&test_profile_,std::shared_ptr<const LoadedJointTestProfile>{});
  active_ = false;
  tracking_fault_.store(true);
  feedback_fault_.store(true);
  feedback_fresh_ = false;
  snapshot();
  const auto joints = get_node()->get_parameter("joints").as_string_array();
  max_error_ = get_node()->get_parameter("max_handover_error").as_double_array();
  short_tracking_report_only_=false;
  receipt_interface_ = get_node()->get_parameter("feedback_receipt_interface").as_string();
  max_feedback_age_ = get_node()->get_parameter("max_feedback_age_s").as_double();
  if (!receipt_interface_.empty() &&
      (receipt_interface_ == "position" || receipt_interface_.find('/') != std::string::npos ||
       !std::isfinite(max_feedback_age_) || max_feedback_age_ <= 0)) {
    return Return::ERROR;
  }
  if (joints.empty() || joints.size()>256 || max_error_.size() != joints.size() ||
      std::set<std::string>(joints.begin(), joints.end()).size() != joints.size()) {
    return Return::ERROR;
  }
  names_.clear();
  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].empty() || joints[i].size()>64 || joints[i].find('\0')!=std::string::npos ||
        !std::isfinite(max_error_[i]) || max_error_[i] < 0) {
      return Return::ERROR;
    }
    names_.push_back(joints[i] + "/position");
  }
  held_.resize(names_.size());
  joints_=joints;
  state_names_ = names_;
  if (!receipt_interface_.empty()) {
    for (const auto & joint : joints) {state_names_.push_back(joint + "/" + receipt_interface_);}
  }
  if (get_node()->get_parameter("joint_test_profile.enabled").as_bool()) {
    if (receipt_interface_.empty()) {return Return::ERROR;}
    try {
      const auto array=[this](const char* name) {
        return get_node()->get_parameter(std::string("joint_test_profile.")+name).as_double_array();
      };
      const auto lower=array("lower_rad"), upper=array("upper_rad"), speed=array("speed_rad_s"),
        acceleration=array("acceleration_rad_s2"), amplitude=array("max_amplitude_rad");
      for (const auto* values : {&lower,&upper,&speed,&acceleration,&amplitude}) {
        if (values->size()!=joints.size()) {throw JointTestAdmissionError("INVALID_PROFILE");}
      }
      const auto cycles=get_node()->get_parameter("joint_test_profile.max_cycles").as_int();
      if (cycles<=0 || cycles>std::numeric_limits<uint32_t>::max()) {throw JointTestAdmissionError("INVALID_PROFILE");}
      JointTestProfile profile;
      profile.id=get_node()->get_parameter("joint_test_profile.id").as_string();
      profile.joints=joints;
      profile.max_amplitude=amplitude;
      profile.max_cycles=static_cast<uint32_t>(cycles);
      profile.max_total_seconds=get_node()->get_parameter("joint_test_profile.max_total_seconds").as_double();
      profile.chirp_start_hz=get_node()->get_parameter("joint_test_profile.chirp_start_hz").as_double();
      profile.chirp_end_hz=get_node()->get_parameter("joint_test_profile.chirp_end_hz").as_double();
      profile.simultaneous_directions=array("simultaneous_directions");
      // Reserved, hash-bound diagnostic identity. Initial handover checks stay
      // enabled. Only reserved sweeps can omit the dynamic tracking abort.
      const bool long_sweep=profile.id=="marvin-stand-simultaneous-10deg-5hz-report-only-60s-v1";
      const bool five_short=long_sweep || profile.id=="marvin-stand-simultaneous-10deg-5hz-report-only-8s-v1";
      const bool fast_short=five_short || profile.id=="marvin-stand-simultaneous-10deg-4hz-report-only-8s-v1";
      const bool ten_short=fast_short || profile.id=="marvin-stand-simultaneous-10deg-3hz-report-only-8s-v1";
      const bool wide_short=ten_short || profile.id=="marvin-stand-simultaneous-7deg-3hz-report-only-8s-v1";
      if (wide_short || profile.id=="marvin-stand-simultaneous-report-only-8s-v1") {
        const double amplitude_cap=ten_short ? 0.17453292519943295 : wide_short ? 0.12217304763960307 : 0.08726646259971647;
        if (profile.max_total_seconds>(long_sweep ? 61. : 9.) || profile.max_cycles!=1 ||
            profile.simultaneous_directions.empty() || profile.chirp_start_hz!=.1 ||
            profile.chirp_end_hz!=(five_short ? 5. : fast_short ? 4. : wide_short ? 3. : 2.) ||
            std::any_of(amplitude.begin(),amplitude.end(),[amplitude_cap](double a){return a>amplitude_cap;})) {
          throw JointTestAdmissionError("INVALID_SHORT_DIAGNOSTIC_PROFILE");
        }
        short_tracking_report_only_=true;
      }
      for (size_t i=0;i<joints.size();++i) {
        profile.bounds.push_back({lower[i],upper[i],speed[i],acceleration[i],max_error_[i]});
      }
      std::shared_ptr<const LoadedJointTestProfile> loaded=std::make_shared<LoadedJointTestProfile>(std::move(profile));
      std::atomic_store(&test_profile_,std::move(loaded));
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_node()->get_logger(),"Joint-test profile rejected: %s",error.what());
      return Return::ERROR;
    }
  }
  if (get_node()->get_parameter("enable_joint_test_target_telemetry").as_bool()) {
    auto publisher=rclcpp::create_publisher<sensor_msgs::msg::JointState>(
      get_node(),"~/joint_test/applied_targets",rclcpp::SensorDataQoS());
    target_publisher_=std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(publisher);
    // Allocate message storage during configuration, never during update.
    target_publisher_->msg_.name=joints_;
    target_publisher_->msg_.position.resize(joints_.size());
    sample_publisher_=rclcpp::create_publisher<runtime_interfaces::msg::JointTestTargetSample>(
      get_node(),"~/joint_test/target_samples",rclcpp::QoS(64).reliable());
    target_queue_=std::make_unique<BoundedTelemetryQueue<TargetSample,256>>();
    telemetry_overflow_=0;
    telemetry_publish_errors_=0;
    stop_telemetry_=false;
    telemetry_worker_=std::thread([this]() {
      runtime_interfaces::msg::JointTestTargetSample message;
      message.controller_instance_id=instance_.data();
      message.joint_names=joints_;
      message.positions.resize(joints_.size());
      TargetSample sample;
      while (!stop_telemetry_.load()) {
        if (!target_queue_->pop(sample)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        message.activation_generation=sample.activation;
        message.update_sequence=sample.sequence;
        message.sampled_at_monotonic_s=sample.sampled_at;
        message.stamp_sec=sample.stamp_sec;
        message.stamp_nanosec=sample.stamp_nanosec;
        std::copy_n(sample.positions.begin(),joints_.size(),message.positions.begin());
        try {sample_publisher_->publish(message);}
        catch (const std::exception&) {++telemetry_publish_errors_;}
      }
    });
  }
  snapshot();
  return Return::SUCCESS;
}

controller_interface::InterfaceConfiguration
ExperimentalJointHoldController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names_};
}

controller_interface::InterfaceConfiguration
ExperimentalJointHoldController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, state_names_};
}

Return ExperimentalJointHoldController::on_activate(const rclcpp_lifecycle::State &)
{
  execution_session_=execution_sequence_=0;
  execution_started_=false;
  execution_permit_until_=0.;
  execution_stop_=JointTestStop::none;
  ++activation_;
  applied_cancel_session_=applied_cancel_sequence_=0;
  active_ = false;
  tracking_fault_.store(true);
  feedback_fault_.store(true);
  feedback_fresh_ = false;
  snapshot();
  if (names_.empty() || command_interfaces_.size() != names_.size() ||
      state_interfaces_.size() != state_names_.size()) {
    return Return::ERROR;
  }
  for (size_t i = 0; i < state_names_.size(); ++i) {
    if (state_interfaces_[i].get_name() != state_names_[i]) {return Return::ERROR;}
  }
  for (size_t i = 0; i < names_.size(); ++i) {
    // Reject unexpected ordering instead of silently commanding another joint.
    if (command_interfaces_[i].get_name() != names_[i] ||
        state_interfaces_[i].get_name() != names_[i]) {
      return Return::ERROR;
    }
    const auto target = read_value(command_interfaces_[i], 0);
    if (!target || !std::isfinite(*target)) {
      return Return::ERROR;
    }
    held_[i] = *target;  // Preserve command continuity; do not jump to measurement.
  }
  tracking_fault_.store(false);
  check_tracking();
  feedback_fault_.store(false);
  check_feedback();
  active_ = true;
  snapshot();
  return Return::SUCCESS;
}

Return ExperimentalJointHoldController::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  tracking_fault_.store(true);
  feedback_fault_.store(true);
  feedback_fresh_ = false;
  snapshot();
  return Return::SUCCESS;
}

void ExperimentalJointHoldController::check_tracking()
{
  for (size_t i = 0; i < held_.size(); ++i) {
    const auto measured = read_value(state_interfaces_[i], 0);
    if (!measured || !std::isfinite(*measured) ||
        ((!short_tracking_report_only_ || !execution_started_) &&
        std::abs(held_[i] - *measured) > max_error_[i])) {
      // Reject future test movement, not ownership of the inherited hold.
      // Recovery of feedback alone does not clear this activation's latch.
      tracking_fault_.store(true);
    }
  }
}

void ExperimentalJointHoldController::check_feedback()
{
  feedback_fresh_ = !receipt_interface_.empty();
  const double now = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  for (size_t i = names_.size(); i < state_names_.size(); ++i) {
    const auto receipt = read_value(state_interfaces_[i], 0);
    if (!receipt || !std::isfinite(*receipt) || *receipt <= 0 ||
        now < *receipt || now - *receipt > max_feedback_age_) {
      feedback_fresh_ = false;
    }
  }
  if (!feedback_fresh_) {feedback_fault_.store(true);}
}

template<class T> bool write_value(T & interface, double value)
{
  if constexpr (std::is_same_v<decltype(interface.set_value(value)), bool>) {
    return interface.set_value(value);
  } else {
    interface.set_value(value);
    return true;
  }
}

controller_interface::return_type ExperimentalJointHoldController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!active_) {return controller_interface::return_type::OK;}
  check_tracking();
  check_feedback();
  std::unique_lock<std::mutex> execution_lock(execution_mutex_,std::try_to_lock);
  std::unique_lock<std::mutex> cancel_lock(cancel_mutex_,std::try_to_lock);
  bool wrote=false, success=true;
  if (execution_lock.owns_lock() && cancel_lock.owns_lock() && execution_ &&
      execution_->plan->snapshot().activation==activation_.load()) {
    auto& slot=*execution_;
    if (pending_cancel_.activation==activation_ && pending_cancel_.session==slot.session) {
      slot.execution.cancel({activation_.load(),slot.session});
    }
    const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (!slot.attempted) {
      slot.attempted=true;
      auto& live=slot.live; // Storage and names were allocated while staging.
      live.activation=activation_.load(); live.revision=status_word_.load()>>8;
      live.hold_applied=status_word_.load()&4;
      live.tracking_fault=tracking_fault_.load(); live.feedback_fault=feedback_fault_.load();
      for (size_t i=0;i<held_.size();++i) {
        live.commands[i]=held_[i];
        live.positions[i]=read_value(state_interfaces_[i],0).value_or(NAN);
        live.receipts[i]=read_value(state_interfaces_[names_.size()+i],0).value_or(NAN);
      }
      live.sampled_at=now;
      // Configuration cannot change during an active lifecycle update. Staging
      // verifies the loaded profile; activation fences out old staged objects.
      slot.begun=slot.execution.begin(live,slot.plan->profile().value().sha256,slot.issued_at,now);
      if (!slot.begun) {
        execution_session_=slot.session; execution_sequence_=0;
        execution_started_=false; execution_stop_=slot.execution.reason();
      }
    }
    if (slot.begun) {
      if (slot.renewal_sequence) {
        if (!tracking_fault_.load() && !feedback_fault_.load() &&
            slot.execution.renew({activation_.load(),slot.session},slot.renewal_issued_at,now)) {
          slot.sequence=slot.renewal_sequence;
        }
        slot.renewal_sequence=0;
      }
      wrote=true;
      success=slot.execution.update(now,true,!tracking_fault_.load()&&!feedback_fault_.load(),
        [&](size_t i,double target) {
          if (!write_value(command_interfaces_[i],target)) {return false;}
          held_[i]=target;
          return true;
        });
      if (success) {
        execution_session_=slot.session; execution_sequence_=slot.sequence;
        execution_started_=true; execution_stop_=slot.execution.reason();
        execution_permit_until_=slot.execution.permit_until();
      }
    }
  }
  if (!wrote) {
    for (size_t i=0;i<held_.size();++i) {
      if (!write_value(command_interfaces_[i],held_[i])) {success=false; break;}
    }
  }
  if (!success) {snapshot(false,true); return controller_interface::return_type::ERROR;}
  if (execution_lock.owns_lock() && cancel_lock.owns_lock()) {
    if (pending_cancel_.session && pending_cancel_.activation==activation_) {
      // Acknowledge only after all held targets were written successfully.
      applied_cancel_session_=pending_cancel_.session;
      applied_cancel_sequence_=pending_cancel_.sequence;
      pending_cancel_.session=0;
    }
  }
  snapshot(true);
  cache_sample();
  // Only complete successful interface writes qualify. Enqueue never waits for
  // DDS. Overflow remains visible as a sequence gap, not CAN delivery evidence.
  ++target_sequence_;
  if (target_queue_) {
    TargetSample sample;
    sample.activation=activation_.load();
    sample.sequence=target_sequence_;
    sample.sampled_at=std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const builtin_interfaces::msg::Time stamp=time;
    sample.stamp_sec=stamp.sec; sample.stamp_nanosec=stamp.nanosec;
    std::copy(held_.begin(),held_.end(),sample.positions.begin());
    if (!target_queue_->push(sample)) ++telemetry_overflow_;
  }
  if (target_publisher_ && target_publisher_->trylock()) {
    target_publisher_->msg_.header.stamp=time;
    std::copy(held_.begin(),held_.end(),target_publisher_->msg_.position.begin());
    target_publisher_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

void ExperimentalJointHoldController::cache_sample()
{
  // Do not wait for readers and do not allocate in the update loop.
  std::unique_lock<std::mutex> lock(sample_mutex_,std::try_to_lock);
  if (!lock.owns_lock()) {return;}
  sample_.count=held_.size();
  sample_.max_feedback_age=max_feedback_age_;
  sample_.activation=activation_;
  sample_.cancelled_session=applied_cancel_session_;
  sample_.applied_sequence=applied_cancel_sequence_;
  sample_.execution_session=execution_session_;
  sample_.execution_sequence=execution_sequence_;
  sample_.execution_started=execution_started_;
  sample_.execution_stop=execution_stop_;
  sample_.execution_permit_until=execution_permit_until_;
  sample_.status_word=status_word_.load();
  sample_.instance=instance_;
  for (size_t i=0; i<held_.size(); ++i) {
    std::copy(joints_[i].begin(),joints_[i].end(),sample_.joints[i].begin());
    sample_.joints[i][joints_[i].size()]='\0';
    sample_.commands[i]=held_[i];
    sample_.positions[i]=read_value(state_interfaces_[i],0).value_or(NAN);
    sample_.receipts[i]=receipt_interface_.empty() ? NAN :
      read_value(state_interfaces_[names_.size()+i],0).value_or(NAN);
  }
  sample_.sampled_at=std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool ExperimentalJointHoldController::read_sample(JointHoldSample& output)
{
  std::lock_guard<std::mutex> lock(sample_mutex_);
  const auto word=status_word_.load();
  const double now=std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  // A newer revision alone does not invalidate the last complete snapshot.
  // Preserve lifecycle, safety-flag and age checks even when cache updates skip.
  if (!joint_sample_valid(word,sample_.status_word,activation_.load(),sample_.activation,
      now,sample_.sampled_at,sample_.max_feedback_age)) {return false;}
  output=sample_;
  return true;
}

bool ExperimentalJointHoldController::read_plan_snapshot(JointTestSnapshot& output)
{
  JointHoldSample sample;
  if (!read_sample(sample)) {return false;}
  JointTestSnapshot state;
  state.instance=sample.instance.data();
  state.activation=sample.activation;
  state.revision=sample.status_word>>8;
  state.sampled_at=sample.sampled_at;
  state.max_feedback_age=sample.max_feedback_age;
  state.cancelled_session=sample.cancelled_session;
  state.applied_sequence=sample.applied_sequence;
  state.hold_applied=sample.status_word&4;
  state.tracking_fault=sample.status_word&2;
  state.feedback_fault=sample.status_word&64;
  for (size_t i=0; i<sample.count; ++i) {
    state.joints.emplace_back(sample.joints[i].data());
    state.commands.push_back(sample.commands[i]);
    state.positions.push_back(sample.positions[i]);
    state.receipts.push_back(sample.receipts[i]);
  }
  output=std::move(state);
  return true;
}

void ExperimentalJointHoldController::prepare_test(const PrepareService::Request& request,
  PrepareService::Response& response)
{
  response.state.controller_instance_id=instance_.data();
  response.state.motion_available=false;
  response.state.phase="not_prepared";
  const auto reject=[&](const char* code) {response.error_code=code; response.message=code;};
  if (!canonical_uuid(request.operation_id)) {reject("INVALID_REQUEST"); return;}
  if (controls_.count(request.operation_id)) {reject("OPERATION_CONFLICT"); return;}
  const auto found=preparations_.find(request.operation_id);
  if (found!=preparations_.end()) {
    const auto& old=found->second.first;
    const auto same=[](const double& a,const double& b) {return std::memcmp(&a,&b,sizeof(double))==0;};
    if (old.controller_instance_id==request.controller_instance_id && old.activation_generation==request.activation_generation &&
        old.profile_id==request.profile_id && old.profile_sha256==request.profile_sha256 && old.joint_name==request.joint_name &&
        old.cycles==request.cycles && same(old.amplitude_rad,request.amplitude_rad) &&
        same(old.transition_duration_s,request.transition_duration_s) && same(old.excitation_duration_s,request.excitation_duration_s)) {
      response=found->second.second;
    } else {reject("OPERATION_CONFLICT");}
    return;
  }
  // Do not evict old operation IDs and accidentally permit their reuse.
  if (preparations_.size()>=128) {reject("OPERATION_CAPACITY"); return;}
  try {
    const auto profile=test_profile();
    if (!profile) {throw JointTestAdmissionError("PROFILE_UNAVAILABLE");}
    JointTestSnapshot state;
    if (!read_plan_snapshot(state)) {throw JointTestAdmissionError("SNAPSHOT_UNAVAILABLE");}
    const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    auto plan=std::make_shared<PreparedJointTestPlan>(request,*profile,state,now,state.max_feedback_age);
    if (test_profile()!=profile) {throw JointTestAdmissionError("PROFILE_CHANGED");}
    if (prepared_plan_ && prepared_plan_->snapshot().activation==state.activation &&
        (!prepared_cancelled_ || state.cancelled_session!=prepared_session_)) {
      throw JointTestAdmissionError("PREPARATION_BUSY");
    }
    std::unique_ptr<JointTestPermitGate> gate;
    if (remote_start_enabled_) {
      std::array<char,37> challenge{};
      if (!random_uuid(challenge)) {throw JointTestAdmissionError("CHALLENGE_UNAVAILABLE");}
      gate=std::make_unique<JointTestPermitGate>(instance_.data(),state.activation,prepared_session_+1,
        challenge.data(),now,remote_lease_);
    }
    prepared_plan_=std::move(plan);
    start_gate_=std::move(gate);
    renewal_gate_.reset(); requested_sequence_=0;
    start_queued_=false;
    prepared_cancelled_=false;
    ++prepared_session_;
    response.accepted=true;
    response.message="Prepared only. Motion is disabled.";
    response.state.activation_generation=state.activation;
    response.state.revision=state.revision;
    response.state.session_id=prepared_session_;
    response.state.phase="prepared";
    if (start_gate_) {
      response.message="Prepared only. Use the one-use challenge to request start.";
      response.state.motion_available=true;
      response.state.permit_challenge=start_gate_->challenge();
      response.state.lease_remaining_s=start_gate_->remaining(now);
    }
  } catch (const JointTestAdmissionError& error) {reject(error.what());}
    catch (const std::invalid_argument&) {reject("TRAJECTORY_LIMIT");}
  preparations_.emplace(request.operation_id,std::make_pair(request,response));
}

bool ExperimentalJointHoldController::stage_prepared_execution(uint64_t session,uint64_t sequence,double lease,double max_gap)
{
  const double issued=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  return stage_prepared_execution(session,sequence,lease,max_gap,issued);
}

bool ExperimentalJointHoldController::stage_prepared_execution(uint64_t session,uint64_t sequence,double lease,double max_gap,double issued_at)
{
  if (!playback_enabled_ || !sequence || !prepared_plan_ || prepared_cancelled_ || session!=prepared_session_) {return false;}
  const auto profile=test_profile();
  if (!profile || profile->value().sha256!=prepared_plan_->profile().value().sha256 ||
      prepared_plan_->snapshot().activation!=activation_.load() || !(status_word_.load()&1)) {return false;}
  auto staged=std::make_unique<ExecutionSlot>(prepared_plan_,session,sequence,lease,max_gap,issued_at);
  std::lock_guard<std::mutex> lock(execution_mutex_);
  if (execution_ && execution_->plan->snapshot().activation==activation_.load() &&
      (execution_->session==session || execution_->execution.reason()==JointTestStop::none)) {return false;}
  execution_=std::move(staged);
  return true;
}

bool ExperimentalJointHoldController::renewal_ready(const JointHoldSample& sample,double now) const
{
  if (!remote_start_enabled_ || prepared_cancelled_ || !prepared_plan_ ||
      sample.activation!=activation_.load() || sample.execution_session!=prepared_session_ ||
      !sample.execution_started || sample.execution_stop!=JointTestStop::none ||
      sample.execution_sequence!=requested_sequence_ || !(sample.status_word&4) ||
      (sample.status_word&(2|8|64)) || !std::isfinite(now) || now<sample.sampled_at ||
      now-sample.sampled_at>remote_max_gap_ || now>=sample.execution_permit_until) {return false;}
  for (size_t i=0;i<sample.count;++i) {
    if (!std::isfinite(sample.receipts[i]) || sample.receipts[i]<=0 || sample.receipts[i]>now ||
        now-sample.receipts[i]>sample.max_feedback_age) {return false;}
  }
  return true;
}

void ExperimentalJointHoldController::issue_renewal_challenge(uint64_t sequence,double now)
{
  requested_sequence_=sequence;
  renewal_gate_.reset();
  std::array<char,37> challenge{};
  if (!random_uuid(challenge)) {return;} // No new permission; existing lease remains bounded.
  renewal_gate_=std::make_unique<JointTestPermitGate>(instance_.data(),prepared_plan_->snapshot().activation,
    prepared_session_,challenge.data(),now,remote_lease_,ControlService::Request::RENEW,sequence);
}

void ExperimentalJointHoldController::control_test(const ControlService::Request& request,
  ControlService::Response& response)
{
  response.state.controller_instance_id=instance_.data();
  response.state.motion_available=false;
  response.state.phase="unavailable";
  const auto reject=[&](const char* code) {response.error_code=code; response.message=code;};
  if (!canonical_uuid(request.operation_id)) {reject("INVALID_REQUEST"); return;}
  if (preparations_.count(request.operation_id)) {reject("OPERATION_CONFLICT"); return;}
  const auto found=controls_.find(request.operation_id);
  if (found!=controls_.end()) {
    if (found->second.first==request) {response=found->second.second;}
    else {reject("OPERATION_CONFLICT");}
    return;
  }
  const bool matching_session=prepared_plan_ && request.controller_instance_id==instance_.data() &&
    request.activation_generation==prepared_plan_->snapshot().activation && request.session_id==prepared_session_;
  // Cached preparation is not evidence that its controller activation is live.
  // Historical exact retries above still return their recorded outcome.
  if ((request.action==ControlService::Request::CANCEL ||
       (remote_start_enabled_ && (request.action==ControlService::Request::START ||
                                 request.action==ControlService::Request::RENEW))) &&
      (!(status_word_.load()&1) || request.activation_generation!=activation_.load())) {
    reject("STALE_ACTIVATION"); return;
  }
  const bool first_cancel=request.action==ControlService::Request::CANCEL && matching_session && !prepared_cancelled_;
  // Reserve one cancellation outcome per admitted session beyond normal history.
  // At most 128 sessions can be admitted by the non-evicting preparation ledger,
  // so total control history remains bounded by 4224 records. The ordinary 4096
  // slots cover twelve 21-second tests at 100 ms renewal intervals without eviction.
  if (controls_.size()>=4096 && !first_cancel) {reject("OPERATION_CAPACITY"); return;}
  if (request.action==ControlService::Request::START && remote_start_enabled_) {
    if (!matching_session || !start_gate_ || prepared_cancelled_) {reject("STALE_SESSION");}
    else {
      JointTestSnapshot live;
      const auto profile=test_profile();
      const bool have_live=read_plan_snapshot(live);
      const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      const bool ready=profile && profile->value().sha256==prepared_plan_->profile().value().sha256 &&
        have_live && prepared_plan_->check_apply(live,now)==JointTestApplyCheck::ready;
      const auto decision=start_gate_->consume(request,now,ready);
      if (decision.dispatch && stage_prepared_execution(request.session_id,request.sequence,
          remote_lease_,remote_max_gap_,start_gate_->issued_at())) {
        start_queued_=true;
        issue_renewal_challenge(request.sequence,now);
        response.accepted=true;
        response.message="Start queued. Read state for applied acknowledgement.";
        response.state.activation_generation=request.activation_generation;
        response.state.session_id=request.session_id;
        response.state.phase="start_pending";
      } else {
        switch (decision.result) {
          case JointTestStartResult::invalid: reject("INVALID_PERMIT"); break;
          case JointTestStartResult::expired: reject("PERMIT_EXPIRED"); break;
          case JointTestStartResult::not_ready: reject("NOT_READY"); break;
          case JointTestStartResult::conflict: reject("OPERATION_CONFLICT"); break;
          case JointTestStartResult::consumed: reject("PERMIT_CONSUMED"); break;
          case JointTestStartResult::accepted: reject("START_NOT_STAGED"); break;
        }
      }
    }
  }
  else if (request.action==ControlService::Request::RENEW && remote_start_enabled_) {
    if (!matching_session || !renewal_gate_ || prepared_cancelled_) {reject("STALE_SESSION");}
    else {
      JointHoldSample sample;
      const bool have_sample=read_sample(sample);
      const double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      const bool ready=have_sample && renewal_ready(sample,now);
      const auto decision=renewal_gate_->consume(request,now,ready);
      if (decision.dispatch) {
        bool staged=false;
        {
          std::lock_guard<std::mutex> lock(execution_mutex_);
          if (execution_ && execution_->session==request.session_id &&
              execution_->plan->snapshot().activation==request.activation_generation &&
              execution_->begun && execution_->execution.reason()==JointTestStop::none &&
              !execution_->renewal_sequence) {
            execution_->renewal_sequence=request.sequence;
            execution_->renewal_issued_at=renewal_gate_->issued_at();
            staged=true;
          }
        }
        if (staged) {
          issue_renewal_challenge(request.sequence,now);
          response.accepted=true;
          response.message="Renewal queued. Read state for applied acknowledgement.";
          response.state.activation_generation=request.activation_generation;
          response.state.session_id=request.session_id;
          response.state.phase="renewal_pending";
        } else {reject("RENEWAL_NOT_STAGED");}
      } else {
        switch (decision.result) {
          case JointTestStartResult::invalid: reject("INVALID_PERMIT"); break;
          case JointTestStartResult::expired: reject("PERMIT_EXPIRED"); break;
          case JointTestStartResult::not_ready: reject("NOT_READY"); break;
          case JointTestStartResult::conflict: reject("OPERATION_CONFLICT"); break;
          default: reject("PERMIT_CONSUMED"); break;
        }
      }
    }
  }
  else if (request.action!=ControlService::Request::CANCEL) {reject("EXPERIMENTAL_MOTION_DISABLED");}
  else if (!matching_session) {
    reject("STALE_SESSION");
  } else {
    if (!prepared_cancelled_) {
      std::lock_guard<std::mutex> lock(cancel_mutex_);
      pending_cancel_={request.activation_generation,request.session_id,request.sequence};
    }
    prepared_cancelled_=true;
    response.accepted=true;
    response.message="Cancellation queued. Read state for applied acknowledgement.";
    response.state.activation_generation=request.activation_generation;
    response.state.session_id=prepared_session_;
    response.state.phase="cancel_pending";
  }
  controls_.emplace(request.operation_id,std::make_pair(request,response));
}
}

PLUGINLIB_EXPORT_CLASS(forward_command_controller_traced::ExperimentalJointHoldController,
                      controller_interface::ControllerInterface)
