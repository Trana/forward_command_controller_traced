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

#include "forward_command_controller/forward_controllers_base.hpp"

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
constexpr const char * kTraceSource = "forward_command_controller";

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

namespace forward_command_controller
{
ForwardControllersBase::ForwardControllersBase()
: controller_interface::ControllerInterface(),
  rt_command_ptr_(nullptr),
  rt_trace_id_(0),
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
      rt_command_ptr_.writeFromNonRT(msg);
      rt_trace_id_.writeFromNonRT(0);
    });

  traced_joints_command_subscriber_ = get_node()->create_subscription<TraceCmdType>(
    "~/commands_traced", rclcpp::SystemDefaultsQoS(),
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

      auto cmd_msg = std::make_shared<CmdType>();
      cmd_msg->data = cmd_values;
      rt_command_ptr_.writeFromNonRT(cmd_msg);
      rt_trace_id_.writeFromNonRT(msg->trace_id);
      publish_trace_event(msg->trace_id, "command_received");
    });

  trace_event_publisher_ = get_node()->create_publisher<TraceEventType>(
    kTraceEventsTopic, rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(
    get_node()->get_logger(),
    "configure successful (subscribers: ~/commands and optional ~/commands_traced)");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ForwardControllersBase::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names = command_interface_types_;

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
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Reset buffers in case a command arrived while inactive.
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  rt_trace_id_.writeFromNonRT(0);
  last_trace_id_ = 0;
  last_applied_trace_id_ = 0;

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardControllersBase::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
  rt_trace_id_.writeFromNonRT(0);
  last_trace_id_ = 0;
  last_applied_trace_id_ = 0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ForwardControllersBase::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto trace_id_ptr = rt_trace_id_.readFromRT();
  if (trace_id_ptr)
  {
    last_trace_id_ = *trace_id_ptr;
  }

  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands))
  {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  {
    if (!set_interface_value_compat(command_interfaces_[index], (*joint_commands)->data[index]))
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Unable to set the command interface value %s: value = %f",
        command_interfaces_[index].get_name().c_str(), (*joint_commands)->data[index]);
      return controller_interface::return_type::OK;
    }
  }

  if (last_trace_id_ != 0 && last_trace_id_ != last_applied_trace_id_)
  {
    publish_trace_event(last_trace_id_, "command_applied");
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
}  // namespace forward_command_controller
