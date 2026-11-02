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

#ifndef FORWARD_COMMAND_CONTROLLER_TRACED__FORWARD_CONTROLLERS_BASE_HPP_
#define FORWARD_COMMAND_CONTROLLER_TRACED__FORWARD_CONTROLLERS_BASE_HPP_

#include <cstdint>
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "marvin_trace_msgs/msg/trace_event.hpp"
#include "marvin_trace_msgs/msg/traced_float64_multi_array.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#if __has_include(<realtime_tools/realtime_buffer.hpp>)
#include "realtime_tools/realtime_buffer.hpp"
#elif __has_include(<realtime_tools/realtime_buffer.h>)
#include "realtime_tools/realtime_buffer.h"
#else
#error "Missing realtime_tools realtime_buffer header"
#endif
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace forward_command_controller_traced
{
using CmdType = std_msgs::msg::Float64MultiArray;
using TraceCmdType = marvin_trace_msgs::msg::TracedFloat64MultiArray;
using TraceEventType = marvin_trace_msgs::msg::TraceEvent;

/**
 * \brief Forward command controller for a set of joints and interfaces.
 *
 * This class forwards the command signal down to a set of joints or interfaces.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 * - \b commands_traced (marvin_trace_msgs::msg::TracedFloat64MultiArray) :
 *   Optional command stream carrying trace_id metadata.
 */
class ForwardControllersBase : public controller_interface::ControllerInterface
{
public:
  ForwardControllersBase();

  ~ForwardControllersBase() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /**
   * Derived controllers have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method of this class.
   */
  virtual void declare_parameters() = 0;

  /**
   * Derived controllers have to read parameters in this method and set `command_interface_types_`
   * variable. The variable is then used to propagate the command interface configuration to
   * controller manager. The method is called from `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and
   * their values are allowed, controller_interface::CallbackReturn::ERROR otherwise.
   */
  virtual controller_interface::CallbackReturn read_parameters() = 0;

  std::vector<std::string> joint_names_;
  std::string interface_name_;

  std::vector<std::string> command_interface_types_;

  // realtime containers used to exchange command data with the control loop
  // Values and correlation metadata must cross into update() atomically.
  struct ReceivedCommand {
    std::shared_ptr<TraceCmdType> message;
    double receipt_monotonic_s;
  };
  void receive_command(const std::shared_ptr<TraceCmdType>& message);
  realtime_tools::RealtimeBuffer<std::shared_ptr<ReceivedCommand>> rt_command_ptr_;
  bool export_command_receipt_ = false;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_commands_;
  std::vector<std::string> claimed_command_names() const;
  uint64_t last_trace_id_;
  uint64_t last_applied_trace_id_;

  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
  rclcpp::Subscription<TraceCmdType>::SharedPtr traced_joints_command_subscriber_;
  rclcpp::Publisher<TraceEventType>::SharedPtr trace_event_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  struct AppliedEvent { uint64_t id; int64_t stamp_ns; };
  std::array<AppliedEvent, 256> applied_events_{};
  std::atomic<size_t> event_write_{0}, event_read_{0};
  std::atomic<uint64_t> dropped_events_{0}, received_count_{0}, applied_count_{0};
  std::atomic<uint64_t> update_count_{0}, late_updates_{0};
  std::atomic<int64_t> command_age_ns_{0};
  std::atomic<bool> capture_enabled_{false};
  bool metrics_enabled_{false};
  int64_t expected_update_ns_{5000000};
  std::string tracing_mode_;
  std::chrono::steady_clock::time_point capture_until_, last_metrics_;

  void export_diagnostics();

  void publish_trace_event(uint64_t trace_id, const std::string & stage);
};

}  // namespace forward_command_controller_traced

#endif  // FORWARD_COMMAND_CONTROLLER_TRACED__FORWARD_CONTROLLERS_BASE_HPP_
