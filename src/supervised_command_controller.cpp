#include "forward_command_controller_traced/forward_command_controller.hpp"
#include "forward_command_controller_traced/diagnostic_execution.hpp"
#include "forward_command_controller_traced/bounded_telemetry_queue.hpp"
#include "runtime_interfaces/srv/native_execution.hpp"
#include "runtime_interfaces/msg/native_execution_command.hpp"
#include "runtime_interfaces/msg/native_execution_state.hpp"
#include "controller_interface/helpers.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <random>
#include <thread>
#include "rclcpp/executors/single_threaded_executor.hpp"
#include <optional>
#include <type_traits>
#include <sstream>
#include <iomanip>

namespace forward_command_controller_traced {
namespace d=diagnostic;
using Service=runtime_interfaces::srv::NativeExecution;
using Command=runtime_interfaces::msg::NativeExecutionCommand;
using State=runtime_interfaces::msg::NativeExecutionState;
static double monotonic() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
template<class Interface> bool assign(Interface& interface,double value) {
  if constexpr(std::is_same_v<decltype(interface.set_value(value)),bool>) return interface.set_value(value);
  else {interface.set_value(value);return true;}
}
// Replaces the forward controller. Both streamed and local targets use write().
class SupervisedCommandController : public ForwardCommandController {
  struct Request {uint8_t action{};d::Plan plan{};double receipt{};};
  struct Frame {uint64_t epoch{},generation{},sequence{},trace{};d::Vector q{};double receipt{};};
  struct Rejection {uint8_t kind{};size_t joint{};double target{},previous{},lower{},upper{},allowed{},dt{};};
  Rejection rejection_{};
  struct Status {uint64_t epoch{},generation{},operation{},revision{};uint8_t mode{};d::Snapshot execution{};d::Vector targets{};bool applied{};int64_t stamp_ns{};double applied_monotonic_s{};uint64_t trace{};Rejection rejection{};};
  BoundedTelemetryQueue<Request,8> requests_;
  BoundedTelemetryQueue<Status,64> statuses_;
  realtime_tools::RealtimeBuffer<Request> cancel_;
  realtime_tools::RealtimeBuffer<Frame> commands_;
  std::unique_ptr<d::Execution> execution_;
  d::Limits limits_;
  d::Vector held_{};
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> positions_,receipts_;
  std::string receipt_interface_;
  std::optional<Request> pending_;
  uint64_t epoch_{},generation_{},operation_{},sequence_{},revision_{},trace_{};
  uint8_t mode_{Service::Request::HOLD};
  double last_command_{},last_status_{},applied_at_{};
  uint64_t reported_generation_{}, reported_target_{};
  int64_t update_stamp_{};
  rclcpp::Publisher<TraceCmdType>::SharedPtr applied_pub_;
  bool seeded_{},applied_{},stream_fault_{};
  d::Fault stream_fault_reason_{d::Fault::none};
  std::atomic<uint64_t> advertised_epoch_{0},dropped_{0};
  uint64_t newest_cancel_{}, newest_request_{};
  std::array<Request,64> accepted_{};
  size_t accepted_index_{};
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Publisher<State>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::CallbackGroup::SharedPtr status_group_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> status_executor_;
  std::thread status_thread_;
  void stop_status_worker() {
    if(status_executor_)status_executor_->cancel();
    if(status_thread_.joinable())status_thread_.join();
    status_timer_.reset();status_executor_.reset();status_group_.reset();
  }
public:
  ~SupervisedCommandController() override {stop_status_worker();}
  controller_interface::CallbackReturn on_init() override {
    const auto result=ForwardCommandController::on_init();
    if(result!=controller_interface::CallbackReturn::SUCCESS)return result;
    auto_declare<std::vector<double>>("diagnostic_lower",{});
    auto_declare<std::vector<double>>("diagnostic_upper",{});
    auto_declare<std::vector<double>>("diagnostic_amplitude",{});
    auto_declare<std::string>("feedback_receipt_interface","");
    auto_declare<bool>("chirp_tracking_report_only",true);
    return result;
  }
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& old) override {
    stop_status_worker();
    const auto result=ForwardCommandController::on_configure(old);
    if(result!=controller_interface::CallbackReturn::SUCCESS)return result;
    // Unfenced legacy input must never bypass the owner protocol.
    joints_command_subscriber_.reset();traced_joints_command_subscriber_.reset();
    if(params_.joints.size()!=d::joints || params_.interface_name!="position")return controller_interface::CallbackReturn::ERROR;
    const auto lower=get_node()->get_parameter("diagnostic_lower").as_double_array();
    const auto upper=get_node()->get_parameter("diagnostic_upper").as_double_array();
    const auto amplitude=get_node()->get_parameter("diagnostic_amplitude").as_double_array();
    receipt_interface_=get_node()->get_parameter("feedback_receipt_interface").as_string();
    limits_.tracking_report_only=get_node()->get_parameter("chirp_tracking_report_only").as_bool();
    if(lower.size()!=12 || upper.size()!=12 || amplitude.size()!=12 || receipt_interface_.empty())return controller_interface::CallbackReturn::ERROR;
    for(size_t i=0;i<12;++i) {
      if(!d::finite(lower[i]) || !d::finite(upper[i]) || lower[i]>=upper[i] ||
         !d::finite(amplitude[i]) || amplitude[i]<0. || amplitude[i]>.4)return controller_interface::CallbackReturn::ERROR;
      limits_.lower[i]=lower[i];limits_.upper[i]=upper[i];limits_.amplitude[i]=amplitude[i];
    }
    service_=get_node()->create_service<Service>("~/execution",[this](Service::Request::SharedPtr req,Service::Response::SharedPtr res){
      if(req->epoch==0 || req->epoch!=advertised_epoch_.load() || req->generation==0 || req->operation==0) {res->reason="Controller epoch or identity mismatch";return;}
      Request command;command.action=req->action;command.receipt=monotonic();
      command.plan={{req->epoch,req->generation,req->operation},req->baseline,req->amplitude,req->duration_s,req->start_hz,req->end_hz};
      for(const auto& previous:accepted_) {
        if(previous.plan.key.epoch==req->epoch && previous.plan.key.generation==req->generation) {
          res->accepted=previous.action==command.action && previous.plan==command.plan;
          res->reason=res->accepted?"Duplicate request; observe applied status":"Generation reused with different content";
          return;
        }
      }
      if(req->generation<=newest_request_) {res->reason="Stale execution generation";return;}
      if(req->action==Service::Request::HOLD) {
        if(req->generation>newest_cancel_) {newest_cancel_=req->generation;cancel_.writeFromNonRT(command);}
        res->accepted=true;
      }
      else if(req->action==Service::Request::STREAM || req->action==Service::Request::POLICY_TARGET || req->action==Service::Request::CHIRP)res->accepted=requests_.push(command);
      if(res->accepted) {
        newest_request_=req->generation;accepted_[accepted_index_]=command;
        accepted_index_=(accepted_index_+1)%accepted_.size();
      }
      res->reason=res->accepted?"Queued; await applied status":"Invalid action or request queue full";
    });
    command_sub_=get_node()->create_subscription<Command>("~/execution_commands",rclcpp::QoS(1).best_effort(),[this](Command::SharedPtr msg){
      if(metrics_enabled_)received_count_.fetch_add(1,std::memory_order_relaxed);
      commands_.writeFromNonRT(Frame{msg->epoch,msg->generation,msg->sequence,msg->trace_id,msg->targets,monotonic()});
    });
    applied_pub_=get_node()->create_publisher<TraceCmdType>("~/commands_traced",rclcpp::QoS(128).best_effort());
    status_pub_=get_node()->create_publisher<State>("~/execution_state",rclcpp::QoS(16).best_effort());
    status_group_=get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    status_timer_=get_node()->create_wall_timer(std::chrono::milliseconds(20),[this]{
      Status s;
      while(statuses_.pop(s)) {
        if(s.execution.tracking_violations) {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),2000,
            "CHIRP_TRACKING_REPORT_ONLY operation=%llu joint=%s peak_error_rad=%.6f threshold_rad=%.3f joint_sample_overruns=%llu",
            static_cast<unsigned long long>(s.execution.key.operation),
            params_.joints[s.execution.peak_tracking_joint].c_str(),
            s.execution.peak_tracking_rad,limits_.tracking_rad,
            static_cast<unsigned long long>(s.execution.tracking_violations));
        }
        State msg;msg.stamp=rclcpp::Time(s.stamp_ns);msg.applied_monotonic_s=s.applied_monotonic_s;msg.epoch=s.epoch;msg.generation=s.generation;msg.operation=s.operation;msg.revision=s.revision;
        msg.mode=s.mode;msg.phase=static_cast<uint8_t>(s.execution.phase);msg.fault=static_cast<uint8_t>(s.execution.fault);
        if(s.rejection.kind) {
          const auto& r=s.rejection;
          std::ostringstream detail;detail<<std::fixed<<std::setprecision(6);
          detail<<"Native target rejected: joint="<<params_.joints[r.joint]
                <<" reason="<<(r.kind==1 ? "non-finite target" : r.kind==2 ? "absolute joint limit" : "target continuity")
                <<" target_rad="<<r.target<<" previous_rad="<<r.previous;
          if(r.kind==2)detail<<" lower_rad="<<r.lower<<" upper_rad="<<r.upper;
          if(r.kind==3)detail<<" delta_rad="<<std::abs(r.target-r.previous)
                            <<" allowed_delta_rad="<<r.allowed<<" dt_s="<<r.dt;
          msg.fault_detail=detail.str();
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),2000,"%s",msg.fault_detail.c_str());
        }
        msg.applied=s.applied;msg.targets=s.targets;msg.generated=s.execution.generated;msg.target_sequence=s.execution.applied;
        msg.elapsed_s=s.execution.elapsed_s;msg.dropped_status=dropped_.load();status_pub_->publish(msg);
        if(s.applied && s.generation) {
          TraceCmdType trace;trace.stamp=msg.stamp;trace.trace_id=s.trace;
          trace.data.assign(s.targets.begin(),s.targets.end());applied_pub_->publish(trace);
        }
      }
    },status_group_);
    status_executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    status_executor_->add_callback_group(status_group_,get_node()->get_node_base_interface());
    status_thread_=std::thread([this]{status_executor_->spin();});
    return result;
  }
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    controller_interface::InterfaceConfiguration cfg;cfg.type=controller_interface::interface_configuration_type::INDIVIDUAL;
    for(const auto& joint:params_.joints) {cfg.names.push_back(joint+"/position");cfg.names.push_back(joint+"/"+receipt_interface_);}
    return cfg;
  }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& old) override {
    auto result=ForwardCommandController::on_activate(old);
    if(result!=controller_interface::CallbackReturn::SUCCESS)return result;
    positions_.clear();receipts_.clear();
    if(!controller_interface::get_ordered_interfaces(state_interfaces_,params_.joints,"position",positions_) ||
       !controller_interface::get_ordered_interfaces(state_interfaces_,params_.joints,receipt_interface_,receipts_))return controller_interface::CallbackReturn::ERROR;
    std::random_device random;epoch_=(static_cast<uint64_t>(random())<<32)|random();if(!epoch_)epoch_=1;
    execution_=std::make_unique<d::Execution>(limits_,epoch_);
    generation_=operation_=sequence_=revision_=newest_cancel_=newest_request_=0;accepted_={};accepted_index_=0;mode_=Service::Request::HOLD;seeded_=applied_=stream_fault_=false;
    pending_.reset();last_status_=0.;advertised_epoch_.store(epoch_);
    return result;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& old) override {
    advertised_epoch_.store(0);return ForwardCommandController::on_deactivate(old);
  }
  controller_interface::return_type update(const rclcpp::Time& time,const rclcpp::Duration& period) override {
    const double now=monotonic();update_stamp_=time.nanoseconds();
    if(metrics_enabled_) {
      update_count_.fetch_add(1,std::memory_order_relaxed);
      if(period.nanoseconds()>expected_update_ns_*3/2)late_updates_.fetch_add(1,std::memory_order_relaxed);
    }
    d::Feedback feedback;
    for(size_t i=0;i<12;++i) {feedback.position[i]=positions_[i].get().get_value();feedback.receipt_s[i]=receipts_[i].get().get_value();}
    if(!seeded_) {
      bool valid=true;
      for(size_t i=0;i<12;++i)valid &= d::finite(feedback.position[i]) && feedback.position[i]>=limits_.lower[i] && feedback.position[i]<=limits_.upper[i] &&
        d::finite(feedback.receipt_s[i]) && feedback.receipt_s[i]>0. && now>=feedback.receipt_s[i] && now-feedback.receipt_s[i]<=.2;
      if(valid){held_=feedback.position;seeded_=true;}
    }
    // Cancellation has a separate mailbox and takes priority over queued starts.
    const auto cancel=*cancel_.readFromRT();
    if(cancel.plan.key.epoch==epoch_ && cancel.plan.key.generation>generation_ &&
       (!pending_ || cancel.plan.key.generation>=pending_->plan.key.generation))pending_=cancel;
    if(!pending_) {Request request;if(requests_.pop(request))pending_=request;}
    if(pending_ && (pending_->plan.key.epoch!=epoch_ || pending_->plan.key.generation<=generation_))pending_.reset();
    if(pending_ && execution_->owns_output()) {
      execution_->cancel(execution_->status().key);
      execution_->update(now,feedback,[this,now](const d::Vector& q){return write(q,now);});
      execution_->release(execution_->status().key);
      // The waveform is released; do not report its cleared counter as a new sample.
      mode_=Service::Request::HOLD;
      report(now);return controller_interface::return_type::OK;
    }
    if(pending_ && seeded_) {
      const auto request=*pending_;pending_.reset();generation_=request.plan.key.generation;operation_=request.plan.key.operation;
      stream_fault_reason_=d::Fault::none;rejection_={};mode_=request.action;trace_=uint64_t{3}<<56;sequence_=0;applied_=false;stream_fault_=false;last_command_=now;
      if(mode_==Service::Request::CHIRP) {
        if(now-request.receipt>.2 || !execution_->start(request.plan,now,held_,feedback)) {mode_=Service::Request::HOLD;stream_fault_=true;stream_fault_reason_=d::Fault::invalid_plan;}
      }
    }
    const auto frame=*commands_.readFromRT();
    if(frame.epoch==epoch_ && frame.generation==generation_ && frame.sequence>sequence_ && now-frame.receipt<=.2) {
      sequence_=frame.sequence;
      if(mode_==Service::Request::CHIRP)execution_->renew(execution_->status().key,sequence_,frame.receipt);
      else if((mode_==Service::Request::STREAM || mode_==Service::Request::POLICY_TARGET) && !stream_fault_) {
        bool valid=true;
        for(size_t i=0;i<12;++i) {
          const uint8_t kind=!d::finite(frame.q[i]) ? 1 :
            (frame.q[i]<limits_.lower[i] || frame.q[i]>limits_.upper[i]) ? 2 : 0;
          if(kind) {
            rejection_={kind,i,frame.q[i],held_[i],limits_.lower[i],limits_.upper[i],0.,0.};
            valid=false;break;
          }
        }
        if(valid) {
          held_=frame.q;last_command_=frame.receipt;trace_=frame.trace;
          if(metrics_enabled_)applied_count_.fetch_add(1,std::memory_order_relaxed);
        } else {stream_fault_=true;stream_fault_reason_=d::Fault::continuity;}
      }
    }
    if(execution_->owns_output())execution_->update(now,feedback,[this,now](const d::Vector& q){return write(q,now);});
    else if(seeded_ && generation_!=0) {
      if((mode_==Service::Request::STREAM || mode_==Service::Request::POLICY_TARGET) && now-last_command_>.2 && !stream_fault_) {stream_fault_=true;stream_fault_reason_=d::Fault::permission;}
      applied_=write(held_,now);
    }
    report(now);return controller_interface::return_type::OK;
  }
private:
  bool write(const d::Vector& q,double receipt) {
    for(size_t i=0;i<12;++i)if(!assign(ordered_commands_[i].get(),q[i])) {applied_=false;return false;}
    for(size_t i=12;i<ordered_commands_.size();++i)if(!assign(ordered_commands_[i].get(),receipt)) {applied_=false;return false;}
    held_=q;applied_=true;applied_at_=receipt;return true;
  }
  void report(double now) {
    const auto target=execution_->status().applied;
    const bool changed=generation_!=reported_generation_ || target!=reported_target_;
    if(!changed && now-last_status_<.019)return;
    reported_generation_=generation_;reported_target_=target;
    last_status_=now;Status status{epoch_,generation_,operation_,++revision_,mode_,execution_->status(),held_,applied_,update_stamp_,applied_at_,
      mode_==Service::Request::CHIRP ? (uint64_t{5}<<56)|execution_->status().applied : trace_};
    if(!execution_->owns_output()) {status.execution={};if(stream_fault_){status.execution.phase=d::Phase::faulted;status.execution.fault=stream_fault_reason_;}}
    if(stream_fault_ && stream_fault_reason_==d::Fault::continuity)status.rejection=rejection_;
    if(!statuses_.push(status))dropped_.fetch_add(1);
  }
};
}
PLUGINLIB_EXPORT_CLASS(forward_command_controller_traced::SupervisedCommandController,controller_interface::ControllerInterface)
