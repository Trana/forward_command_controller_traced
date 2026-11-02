#include <gtest/gtest.h>
#include "forward_command_controller_traced/forward_command_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "forward_command_controller_traced/experimental_joint_hold_controller.hpp"
#include <limits>

using namespace forward_command_controller_traced;

class ProbeController : public ForwardCommandController
{
public:
  using ForwardControllersBase::applied_count_;
  using ForwardControllersBase::capture_enabled_;
  using ForwardControllersBase::capture_until_;
  using ForwardControllersBase::dropped_events_;
  using ForwardControllersBase::event_read_;
  using ForwardControllersBase::event_write_;
  using ForwardControllersBase::export_diagnostics;
  using ForwardControllersBase::joints_command_subscriber_;
  using ForwardControllersBase::last_applied_trace_id_;
  using ForwardControllersBase::metrics_publisher_;
  using ForwardControllersBase::trace_event_publisher_;
  void command(uint64_t id, double value) {
    auto msg = std::make_shared<TraceCmdType>();
    msg->trace_id = id;
    msg->stamp = get_node()->now();
    msg->data = {value};
    receive_command(msg);
  }
};

class TracingModes : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(TracingModes, ExperimentalHoldRetainsOwnershipAndLatchesBadTracking)
{
  std::set<std::string> instances;
  for (double measured : {.2, .8, std::numeric_limits<double>::quiet_NaN()}) {
    ExperimentalJointHoldController controller;
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
      rclcpp::Parameter("max_handover_error", std::vector<double>{.05}),
      rclcpp::Parameter("max_feedback_age_s", .5)});
    ASSERT_EQ(controller.init("hold_test", "", 200, "", options), controller_interface::return_type::OK);
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
    double target = .21;
    hardware_interface::CommandInterface command("joint", "position", &target);
    hardware_interface::StateInterface state("joint", "position", &measured);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    commands.emplace_back(command);
    states.emplace_back(state);
    controller.assign_interfaces(std::move(commands), std::move(states));
    const auto result = controller.on_activate(rclcpp_lifecycle::State());
    ASSERT_EQ(result, controller_interface::CallbackReturn::SUCCESS);
    JointHoldSample sample;
    EXPECT_FALSE(controller.read_sample(sample)); // No successful update yet.
    EXPECT_EQ(controller.tracking_fault_latched(), measured != .2);
    target = .9;  // Check that update repeats the captured command, not measurement.
    EXPECT_EQ(controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)),
      controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(target, .21);
    ASSERT_TRUE(controller.read_sample(sample));
    EXPECT_EQ(sample.count,1u);
    EXPECT_GT(sample.activation,0u);
    EXPECT_GT(sample.sampled_at,0.);
    EXPECT_DOUBLE_EQ(sample.commands[0],.21);
    if (std::isfinite(measured)) {EXPECT_DOUBLE_EQ(sample.positions[0],measured);}
    else {EXPECT_TRUE(std::isnan(sample.positions[0]));}
    EXPECT_TRUE(std::isnan(sample.receipts[0]));
    const auto revision=sample.status_word;
    JointHoldSample repeated;
    ASSERT_TRUE(controller.read_sample(repeated));
    EXPECT_EQ(repeated.status_word,revision); // Reads do not renew receipt/revision.
    EXPECT_EQ(repeated.sampled_at,sample.sampled_at);
    JointTestSnapshot plan_state;
    ASSERT_TRUE(controller.read_plan_snapshot(plan_state));
    EXPECT_TRUE(canonical_uuid(plan_state.instance));
    EXPECT_TRUE(instances.insert(plan_state.instance).second);
    EXPECT_EQ(plan_state.joints,std::vector<std::string>{"joint"});
    EXPECT_EQ(plan_state.commands,std::vector<double>{.21});
    EXPECT_EQ(plan_state.revision,sample.status_word>>8);
    EXPECT_EQ(plan_state.activation,sample.activation);
    EXPECT_EQ(plan_state.sampled_at,sample.sampled_at);
    EXPECT_TRUE(plan_state.hold_applied);
    EXPECT_TRUE(plan_state.feedback_fault); // No receipt interface configured.
    measured = .8;  // A fault after activation is also latched.
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
    ASSERT_TRUE(controller.tracking_fault_latched());
    measured = .21;
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
    EXPECT_TRUE(controller.tracking_fault_latched());
    EXPECT_DOUBLE_EQ(target, .21);
    controller.on_deactivate(rclcpp_lifecycle::State());
    EXPECT_FALSE(controller.read_sample(sample));
    EXPECT_FALSE(controller.read_plan_snapshot(plan_state));
    target = .4;
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
    EXPECT_DOUBLE_EQ(target, .4);
    target = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::ERROR);
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
    EXPECT_TRUE(std::isnan(target));  // No guessed replacement for invalid command memory.
    controller.release_interfaces();
  }
}

TEST_F(TracingModes, ReceiptFreshnessFaultRetainsHoldAndDoesNotAutoClear)
{
  for (double age : {.01, 2., -1., std::numeric_limits<double>::quiet_NaN()}) {
    ExperimentalJointHoldController controller;
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
      rclcpp::Parameter("max_handover_error", std::vector<double>{.05}),
      rclcpp::Parameter("feedback_receipt_interface", "encoder_receipt_monotonic_s"),
      rclcpp::Parameter("max_feedback_age_s", 1.)}); // Synthetic test bound, not a robot profile.
    ASSERT_EQ(controller.init("receipt_test", "", 200, "", options), controller_interface::return_type::OK);
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
    auto now = []() {return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();};
    double target = .21, measured = .2, receipt = now() - age;
    hardware_interface::CommandInterface command("joint", "position", &target);
    hardware_interface::StateInterface position("joint", "position", &measured);
    hardware_interface::StateInterface stamp("joint", "encoder_receipt_monotonic_s", &receipt);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    commands.emplace_back(command);
    states.emplace_back(position);
    states.emplace_back(stamp);
    controller.assign_interfaces(std::move(commands), std::move(states));
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(controller.feedback_fault_latched(), age != .01);
    receipt = now() - 2.;
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
    EXPECT_TRUE(controller.feedback_fault_latched());
    receipt = now();
    controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01));
    EXPECT_TRUE(controller.feedback_fault_latched());
    EXPECT_DOUBLE_EQ(target, .21);
    controller.on_deactivate(rclcpp_lifecycle::State());
    controller.release_interfaces();
  }
}

TEST_F(TracingModes, CommandsApplyInEveryModeAndLegacyIsOptIn)
{
  for (const auto & mode : {"off", "metrics", "capture"}) {
    ProbeController controller;
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
      rclcpp::Parameter("interface_name", "position"), rclcpp::Parameter("tracing_mode", mode)});
    ASSERT_EQ(controller.init("trace_test", "", 200, "", options), controller_interface::return_type::OK);
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(controller.joints_command_subscriber_, nullptr);
    EXPECT_EQ(controller.trace_event_publisher_ != nullptr, std::string(mode) == "capture");
    EXPECT_EQ(controller.metrics_publisher_ != nullptr, std::string(mode) != "off");
    double position = 0;
    hardware_interface::CommandInterface handle("joint", "position", &position);
    std::vector<hardware_interface::LoanedCommandInterface> interfaces;
    interfaces.emplace_back(handle);
    controller.assign_interfaces(std::move(interfaces), {});
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
    controller.command(42, 0.25);
    EXPECT_EQ(controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(.005)),
      controller_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(position, .25);
    EXPECT_EQ(controller.last_applied_trace_id_, 42u);
    EXPECT_EQ(controller.event_write_.load(), std::string(mode) == "capture" ? 1u : 0u);
    controller.on_deactivate(rclcpp_lifecycle::State());
    controller.release_interfaces();
  }
}

TEST_F(TracingModes, BoundedQueueDropsDiagnosticsAndExpiryKeepsCommandsWorking)
{
  ProbeController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
    rclcpp::Parameter("interface_name", "position"), rclcpp::Parameter("tracing_mode", "capture")});
  ASSERT_EQ(controller.init("queue_test", "", 200, "", options), controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  double position = 0;
  hardware_interface::CommandInterface handle("joint", "position", &position);
  std::vector<hardware_interface::LoanedCommandInterface> interfaces;
  interfaces.emplace_back(handle);
  controller.assign_interfaces(std::move(interfaces), {});
  controller.on_activate(rclcpp_lifecycle::State());
  for (uint64_t id = 1; id <= 300; ++id) {
    controller.command(id, static_cast<double>(id));
    controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(.005));
  }
  EXPECT_DOUBLE_EQ(position, 300);
  EXPECT_EQ(controller.dropped_events_.load(), 45u);
  controller.capture_until_ = std::chrono::steady_clock::now();
  controller.export_diagnostics();
  EXPECT_FALSE(controller.capture_enabled_.load());
  EXPECT_EQ(controller.event_read_.load(), controller.event_write_.load());
  controller.command(301, .5);
  controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(position, .5);
  EXPECT_EQ(controller.last_applied_trace_id_, 301u);
  controller.on_deactivate(rclcpp_lifecycle::State());
  controller.release_interfaces();
}

TEST_F(TracingModes, ReactivationClearsBufferedCommandsButNotFutureArrivals)
{
  // Fake command memory only. This tests lifecycle callbacks, not an actual
  // controller-manager switch, middleware queue drainage or hardware support.
  ProbeController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
    rclcpp::Parameter("interface_name", "position"), rclcpp::Parameter("tracing_mode", "off")});
  ASSERT_EQ(controller.init("handover_test", "", 200, "", options), controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  double position = .2;
  hardware_interface::CommandInterface handle("joint", "position", &position);
  std::vector<hardware_interface::LoanedCommandInterface> interfaces;
  interfaces.emplace_back(handle);
  controller.assign_interfaces(std::move(interfaces), {});
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  controller.command(1, .25);
  controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(.005));
  ASSERT_DOUBLE_EQ(position, .25);
  ASSERT_EQ(controller.on_deactivate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  controller.command(2, .9);  // Already delivered to the buffer while inactive.
  position = .3;  // Stand-in for the final target of another controller.
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(position, .3);
  EXPECT_EQ(controller.last_applied_trace_id_, 0u);

  // An old source that keeps publishing can still replace the handback target.
  // Trace IDs are diagnostics, not ownership/freshness fences.
  controller.command(1, .9);
  controller.update(controller.get_node()->now(), rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(position, .9);
  controller.on_deactivate(rclcpp_lifecycle::State());
  controller.release_interfaces();
}

TEST_F(TracingModes, ReceiptBelongsToCallbackAndSurvivesRepeatedUpdates)
{
  ProbeController controller;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("joints", std::vector<std::string>{"joint"}),
    rclcpp::Parameter("interface_name", "position"),
    rclcpp::Parameter("export_command_receipt", true)});
  ASSERT_EQ(controller.init("receipt_test", "", 200, "", options), controller_interface::return_type::OK);
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.command_interface_configuration().names,
    (std::vector<std::string>{"joint/position", "joint/command_receipt_monotonic_s"}));
  double target = 0, receipt = 0;
  hardware_interface::CommandInterface position("joint", "position", &target);
  hardware_interface::CommandInterface timestamp("joint", "command_receipt_monotonic_s", &receipt);
  std::vector<hardware_interface::LoanedCommandInterface> commands;
  // Deliberately reverse loan order: position and receipt must remain paired.
  commands.emplace_back(timestamp);
  commands.emplace_back(position);
  controller.assign_interfaces(std::move(commands), {});
  ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
  controller.command(1, .4);
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(target, .4);
  EXPECT_GT(receipt, 0);
  const auto first = receipt;
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.005));
  EXPECT_DOUBLE_EQ(receipt, first);
  controller.command(2, .5);
  controller.update(rclcpp::Time(0), rclcpp::Duration::from_seconds(.005));
  EXPECT_GT(receipt, first);
  EXPECT_DOUBLE_EQ(target, .5);
}
