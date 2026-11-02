// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "forward_command_controller_traced/forward_controllers_base.hpp"
#include <cstdlib>
#include <sstream>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace
{  // utility
constexpr const char * kTraceEventsTopic = "/trace/events";
constexpr const char * kTraceSource = "forward_command_controller_traced";

template <typename InterfaceT>
bool set_interface_value_compat(InterfaceT & interface, double value)
{
  using RetT = decltype(interface.set_value(value));
  if constexpr (std::is_same<RetT, bool>::value)
  {
    return interface.set_value(value);
  }
  else
  {
    interface.set_value(value);
    return true;
  }
}

}  // namespace

namespace forward_command_controller_traced
{
ForwardControllersBase::ForwardControllersBase()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  last_trace_id_(0),
  last_applied_trace_id_(0),
  joints_command_subscriber_(nullptr),
  traced_joints_command_subscriber_(nullptr),
  trace_event_publisher_(nullptr)
{
}

controller_interface::CallbackReturn ForwardControllersBase::on_init()
{
  try
  {
    declare_parameters();
    const char * mode = std::getenv("MARVIN_TRACING_MODE");
    const char * duration = std::getenv("MARVIN_TRACE_CAPTURE_SECONDS");
    auto_declare<std::string>("tracing_mode", mode ? mode : "metrics");
    auto_declare<double>("trace_capture_seconds", duration ? std::stod(duration) : 30.0);
    auto_declare<bool>("accept_legacy_commands", false);
    auto_declare<bool>("export_command_receipt", false);
    auto_declare<double>("expected_update_period_ms", 5.0);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardControllersBase::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  export_command_receipt_ = get_node()->get_parameter("export_command_receipt").as_bool();
  if (export_command_receipt_ && std::any_of(command_interface_types_.begin(),
      command_interface_types_.end(), [](const auto& name) {
        return name.substr(name.find_last_of('/') + 1) != "position";
      })) {
    RCLCPP_ERROR(get_node()->get_logger(), "Command receipt export requires position interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  diagnostics_timer_.reset();
  metrics_publisher_.reset();
  trace_event_publisher_.reset();
  joints_command_subscriber_.reset();
  traced_joints_command_subscriber_.reset();
  event_write_.store(0);
  event_read_.store(0);
  dropped_events_.store(0);
  received_count_.store(0);
  applied_count_.store(0);
  update_count_.store(0);
  late_updates_.store(0);

  tracing_mode_ = get_node()->get_parameter("tracing_mode").as_string();
  const auto seconds = get_node()->get_parameter("trace_capture_seconds").as_double();
  const auto period_ms = get_node()->get_parameter("expected_update_period_ms").as_double();
  if ((tracing_mode_ != "off" && tracing_mode_ != "metrics" && tracing_mode_ != "capture") ||
      !std::isfinite(seconds) || seconds <= 0 || seconds > 300 ||
      !std::isfinite(period_ms) || period_ms <= 0 || period_ms > 1000)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid tracing mode or capture duration (0,300]");
    return controller_interface::CallbackReturn::ERROR;
  }
  metrics_enabled_ = tracing_mode_ != "off";
  expected_update_ns_ = static_cast<int64_t>(period_ms * 1000000);
  capture_enabled_.store(tracing_mode_ == "capture");
  last_metrics_ = std::chrono::steady_clock::now();
  capture_until_ = last_metrics_ + std::chrono::milliseconds(static_cast<int64_t>(seconds * 1000));
  if (metrics_enabled_) {
    metrics_publisher_ = get_node()->create_publisher<std_msgs::msg::String>("/trace/metrics", 10);
    diagnostics_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100), [this]() { export_diagnostics(); });
  }
  if (capture_enabled_.load()) {
    trace_event_publisher_ = get_node()->create_publisher<TraceEventType>(
      kTraceEventsTopic, rclcpp::QoS(256).best_effort());
  }
  if (get_node()->get_parameter("accept_legacy_commands").as_bool()) {
    joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg)
    {
      const auto cmd = *msg;

      if (!std::all_of(
            cmd.data.cbegin(), cmd.data.cend(),
            [](const auto & value) { return std::isfinite(value); }))
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "Non-finite value received. Dropping message");
        return;
      }
      auto record = std::make_shared<TraceCmdType>();
      record->data = msg->data;
      record->stamp = get_node()->now();
      receive_command(record);
      if (metrics_enabled_) { received_count_.fetch_add(1, std::memory_order_relaxed); }
    });
  }

  traced_joints_command_subscriber_ = get_node()->create_subscription<TraceCmdType>(
    "~/commands_traced", rclcpp::QoS(1).best_effort(),
    [this](const TraceCmdType::SharedPtr msg)
    {
      const auto & cmd_values = msg->data;
      if (!std::all_of(
            cmd_values.cbegin(), cmd_values.cend(),
            [](const auto & value) { return std::isfinite(value); }))
      {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "Non-finite value received on traced command topic. Dropping message");
        return;
      }

      receive_command(msg);
      if (metrics_enabled_) { received_count_.fetch_add(1, std::memory_order_relaxed); }
      if (capture_enabled_.load(std::memory_order_relaxed)) {
        publish_trace_event(msg->trace_id, "command_received");
      }
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "configure successful (canonical ~/commands_traced; tracing=%s)", tracing_mode_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

void ForwardControllersBase::receive_command(const std::shared_ptr<TraceCmdType>& message)
{
  const auto receipt = std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
  rt_command_ptr_.writeFromNonRT(std::make_shared<ReceivedCommand>(ReceivedCommand{message, receipt}));
}

std::vector<std::string> ForwardControllersBase::claimed_command_names() const
{
  auto names = command_interface_types_;
  if (export_command_receipt_) {
    for (const auto& name : command_interface_types_) {
      names.push_back(name.substr(0, name.find_last_of('/') + 1) + "command_receipt_monotonic_s");
    }
  }
  return names;
}

controller_interface::InterfaceConfiguration
ForwardControllersBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = claimed_command_names();

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ForwardControllersBase::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn ForwardControllersBase::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  ordered_commands_.clear();
  const auto names = claimed_command_names();
  if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, names, std::string(""), ordered_commands_) ||
      ordered_commands_.size() != names.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Missing required command interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset buffers in case a command arrived while inactive.
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReceivedCommand>>(nullptr);
  last_trace_id_ = 0;
  last_applied_trace_id_ = 0;

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardControllersBase::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<ReceivedCommand>>(nullptr);
  last_trace_id_ = 0;
  last_applied_trace_id_ = 0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardControllersBase::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (metrics_enabled_) {
    update_count_.fetch_add(1, std::memory_order_relaxed);
    if (period.nanoseconds() > expected_update_ns_ * 3 / 2) {
      late_updates_.fetch_add(1, std::memory_order_relaxed);
    }
  }
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }
  const auto& command = (*joint_commands)->message;
  last_trace_id_ = command->trace_id;
  if (metrics_enabled_) {
    const auto & stamp = command->stamp;
    command_age_ns_.store(time.nanoseconds() -
      (static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec), std::memory_order_relaxed);
  }

  if (command->data.size() != command_interface_types_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      command->data.size(), command_interface_types_.size());
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interface_types_.size(); ++index)
  {
    if (!set_interface_value_compat(ordered_commands_[index].get(), command->data[index]))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Unable to set the command interface value %s: value = %f",
        command_interfaces_[index].get_name().c_str(), command->data[index]);
      return controller_interface::return_type::OK;
    }
  }

  if (export_command_receipt_) {
    for (size_t index = command_interface_types_.size(); index < ordered_commands_.size(); ++index) {
      if (!set_interface_value_compat(ordered_commands_[index].get(),
          (*joint_commands)->receipt_monotonic_s)) {
        return controller_interface::return_type::ERROR;
      }
    }
  }

  if (last_trace_id_ != 0 && last_trace_id_ != last_applied_trace_id_)
  {
    if (metrics_enabled_) { applied_count_.fetch_add(1, std::memory_order_relaxed); }
    if (capture_enabled_.load(std::memory_order_relaxed)) {
      const auto write = event_write_.load(std::memory_order_relaxed);
      const auto next = (write + 1) % applied_events_.size();
      if (next == event_read_.load(std::memory_order_acquire)) {
        dropped_events_.fetch_add(1, std::memory_order_relaxed);
      } else {
        applied_events_[write] = {last_trace_id_, get_node()->get_clock()->now().nanoseconds()};
        event_write_.store(next, std::memory_order_release);
      }
    }
    last_applied_trace_id_ = last_trace_id_;
  }

  return controller_interface::return_type::OK;
}

void ForwardControllersBase::publish_trace_event(uint64_t trace_id, const std::string & stage)
{
  if (!trace_event_publisher_ || trace_id == 0)
  {
    return;
  }

  TraceEventType msg;
  const auto now_ns = get_node()->get_clock()->now().nanoseconds();
  msg.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
  msg.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
  msg.trace_id = trace_id;
  msg.stage = stage;
  msg.source = kTraceSource;
  trace_event_publisher_->publish(msg);
}

void ForwardControllersBase::export_diagnostics()
{
  const auto now = std::chrono::steady_clock::now();
  if (capture_enabled_.load() && now >= capture_until_) {
    capture_enabled_.store(false);
    tracing_mode_ = "metrics";
  }
  auto read = event_read_.load(std::memory_order_relaxed);
  const auto write = event_write_.load(std::memory_order_acquire);
  while (read != write) {
    const auto record = applied_events_[read];
    TraceEventType event;
    event.trace_id = record.id;
    event.stamp.sec = static_cast<int32_t>(record.stamp_ns / 1000000000LL);
    event.stamp.nanosec = static_cast<uint32_t>(record.stamp_ns % 1000000000LL);
    event.stage = "command_applied";
    event.source = kTraceSource;
    trace_event_publisher_->publish(event);
    read = (read + 1) % applied_events_.size();
    event_read_.store(read, std::memory_order_release);
  }
  const auto seconds = std::chrono::duration<double>(now - last_metrics_).count();
  if (seconds < 1.0) { return; }
  last_metrics_ = now;
  std::ostringstream json;
  json << "{\"schema_version\":\"runtime-timing.v1\",\"source\":\"joint_controller\","
       << "\"mode\":\"" << tracing_mode_ << "\",\"window_s\":" << seconds
       << ",\"received_hz\":" << static_cast<double>(received_count_.exchange(0)) / seconds
       << ",\"applied_hz\":" << static_cast<double>(applied_count_.exchange(0)) / seconds
       << ",\"update_hz\":" << static_cast<double>(update_count_.exchange(0)) / seconds
       << ",\"late_updates\":" << late_updates_.exchange(0)
       << ",\"command_age_ms\":" << static_cast<double>(command_age_ns_.load()) / 1e6
       << ",\"dropped_trace_events\":" << dropped_events_.exchange(0) << "}";
  std_msgs::msg::String message;
  message.data = json.str();
  metrics_publisher_->publish(message);
}
}  // namespace forward_command_controller_traced
