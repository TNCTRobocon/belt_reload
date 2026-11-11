#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sequence_interfaces/action/sequence.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

class BeltReloadNode final : public rclcpp::Node
{
public:
  using Sequence = sequence_interfaces::action::Sequence;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Sequence>;
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;

  BeltReloadNode()
  : Node("belt_reload_node")
  {
    declare_parameter("step_mm", 100.0);
    declare_parameter("position_tolerance_mm", 3.0);
    declare_parameter("stable_time_sec", 0.1);
    declare_parameter("feedback_timeout_ms", 500);
    declare_parameter("motion_timeout_sec", 15.0);
    declare_parameter("control_period_ms", 20);
    declare_parameter(
      "operation_state_topic", std::string("/shoot_belt/operation_state"));
    declare_parameter("ready_topic", std::string("/reload/ready"));

    operation_state_topic_ =
      get_parameter("operation_state_topic").as_string();
    ready_topic_ = get_parameter("ready_topic").as_string();
    if (operation_state_topic_.empty() || ready_topic_.empty()) {
      throw std::runtime_error("operation_state_topic and ready_topic must not be empty");
    }

    step_mm_ = get_parameter("step_mm").as_double();
    position_tolerance_mm_ =
      get_parameter("position_tolerance_mm").as_double();
    stable_time_sec_ = get_parameter("stable_time_sec").as_double();
    feedback_timeout_ms_ =
      get_parameter("feedback_timeout_ms").as_int();
    motion_timeout_sec_ =
      get_parameter("motion_timeout_sec").as_double();
    control_period_ms_ =
      get_parameter("control_period_ms").as_int();

    if (!std::isfinite(step_mm_) || step_mm_ <= 0.0 ||
        !std::isfinite(position_tolerance_mm_) ||
        position_tolerance_mm_ <= 0.0 ||
        !std::isfinite(stable_time_sec_) || stable_time_sec_ < 0.0 ||
        feedback_timeout_ms_ <= 0 ||
        !std::isfinite(motion_timeout_sec_) || motion_timeout_sec_ <= 0.0 ||
        control_period_ms_ <= 0) {
      throw std::runtime_error("Invalid belt_reload parameters");
    }

    const auto qos = rclcpp::QoS(10).reliable();

    can_rx_sub_ = create_subscription<can_msgs::msg::Frame>(
      "/can_rx", qos,
      std::bind(&BeltReloadNode::on_can_rx, this, std::placeholders::_1));

    can_tx_pub_ = create_publisher<can_msgs::msg::Frame>("/can_tx", qos);

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    reload_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      ready_topic_, state_qos);
    publish_reload_ready(false);

    shoot_operation_state_sub_ =
      create_subscription<std_msgs::msg::UInt8>(
      operation_state_topic_, state_qos,
      std::bind(
        &BeltReloadNode::on_shoot_operation_state, this,
        std::placeholders::_1));

    execute_server_ = rclcpp_action::create_server<Sequence>(
      this,
      "/reload/execute",
      std::bind(
        &BeltReloadNode::handle_execute_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &BeltReloadNode::handle_cancel, this, std::placeholders::_1),
      std::bind(
        &BeltReloadNode::handle_execute_accepted, this,
        std::placeholders::_1));

    reset_origin_server_ = rclcpp_action::create_server<Sequence>(
      this,
      "/reload/reset_origin",
      std::bind(
        &BeltReloadNode::handle_reset_origin_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &BeltReloadNode::handle_cancel, this, std::placeholders::_1),
      std::bind(
        &BeltReloadNode::handle_reset_origin_accepted, this,
        std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&BeltReloadNode::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "belt_reload started: /reload/execute and /reload/reset_origin; "
      "shot readiness from %s, published on %s; waiting for CAN position "
      "feedback (ID 0x180)",
      operation_state_topic_.c_str(), ready_topic_.c_str());
  }

private:
  static constexpr uint32_t kPositionFeedbackCanId = 0x180;
  static constexpr uint32_t kPositionTargetCanId = 0x280;
  static constexpr uint32_t kResetOriginCanId = 0x281;

  static constexpr uint8_t kPhaseIdle = 0;
  static constexpr uint8_t kPhaseCommanding = 1;
  static constexpr uint8_t kPhaseVerifying = 2;
  static constexpr uint8_t kPhaseComplete = 3;
  static constexpr uint8_t kPhaseError = 4;
  static constexpr uint8_t kPhaseCanceled = 5;

  enum class ActionKind
  {
    NONE,
    EXECUTE,
    RESET_ORIGIN
  };

  rclcpp_action::GoalResponse handle_execute_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const Sequence::Goal> goal)
  {
    if (active_goal_ || goal_reserved_) {
      RCLCPP_WARN(get_logger(), "Rejecting execute: another action is active or reserved");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->operation != Sequence::Goal::OPERATION_PROCEED) {
      RCLCPP_WARN(
        get_logger(), "Rejecting execute: expected OPERATION_PROCEED (2)");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!shoot_operation_state_ || *shoot_operation_state_ != kShootReadyState) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting execute: shoot_belt is not READY (operation state=%s)",
        shoot_operation_state_ ? std::to_string(*shoot_operation_state_).c_str() : "unknown");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!reload_ready_) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting execute: no successful shot has completed since the last reload/reset");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!has_fresh_feedback()) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting execute: no recent position feedback on /can_rx (ID 0x180)");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Reserve and consume the one-shot permission in the goal callback so
    // duplicate goals cannot race before the accepted callback runs.
    goal_reserved_ = true;
    shot_running_seen_ = false;
    publish_reload_ready(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse handle_reset_origin_goal(
    const rclcpp_action::GoalUUID &,
    const std::shared_ptr<const Sequence::Goal> goal)
  {
    if (active_goal_ || goal_reserved_) {
      RCLCPP_WARN(
        get_logger(), "Rejecting reset_origin: another action is active or reserved");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->operation != Sequence::Goal::OPERATION_HOME) {
      RCLCPP_WARN(
        get_logger(), "Rejecting reset_origin: expected OPERATION_HOME (4)");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (shoot_operation_state_ &&
        *shoot_operation_state_ == kShootRunningState) {
      RCLCPP_WARN(
        get_logger(), "Rejecting reset_origin: shoot_belt is RUNNING");
      return rclcpp_action::GoalResponse::REJECT;
    }

    goal_reserved_ = true;
    shot_running_seen_ = false;
    publish_reload_ready(false);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void on_shoot_operation_state(
    const std_msgs::msg::UInt8::SharedPtr message)
  {
    if (!message) {
      return;
    }

    shoot_operation_state_ = message->data;
    if (message->data == kShootRunningState) {
      shot_running_seen_ = true;
      publish_reload_ready(false);
    } else if (message->data == kShootReadyState) {
      const bool shot_completed = shot_running_seen_;
      shot_running_seen_ = false;
      if (shot_completed && !active_goal_ && !goal_reserved_) {
        publish_reload_ready(true);
        RCLCPP_INFO(
          get_logger(),
          "RELOAD enabled: shoot_belt completed successfully (RUNNING -> READY)");
      }
    } else {
      // ERROR, CANCELLED, RESETTING, RESET_FAILED and unknown states all
      // invalidate a pending reload permission.
      shot_running_seen_ = false;
      publish_reload_ready(false);
    }
  }

  void publish_reload_ready(const bool ready)
  {
    reload_ready_ = ready;
    if (!reload_ready_pub_) {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = ready;
    reload_ready_pub_->publish(message);
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle> handle)
  {
    if (!active_goal_ || !handle ||
        handle->get_goal_id() != active_goal_->get_goal_id()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_execute_accepted(const std::shared_ptr<GoalHandle> handle)
  {
    goal_reserved_ = false;
    start_action(handle, ActionKind::EXECUTE);
  }

  void handle_reset_origin_accepted(const std::shared_ptr<GoalHandle> handle)
  {
    goal_reserved_ = false;
    start_action(handle, ActionKind::RESET_ORIGIN);
  }

  void start_action(
    const std::shared_ptr<GoalHandle> & handle,
    const ActionKind kind)
  {
    active_goal_ = handle;
    active_action_ = kind;
    action_started_at_ = SteadyClock::now();
    next_command_at_ = action_started_at_;
    feedback_generation_at_start_ = feedback_generation_;
    in_tolerance_since_.reset();
    stable_arrival_confirmed_ = false;
    phase_ = kPhaseCommanding;

    if (kind == ActionKind::EXECUTE) {
      start_position_mm_ = current_position_mm_;
      target_position_mm_ = current_position_mm_ + step_mm_;

      if (!std::isfinite(target_position_mm_)) {
        finish_aborted("calculated target position is not finite");
        return;
      }

      RCLCPP_INFO(
        get_logger(), "Execute started: current=%.3f mm, target=%.3f mm",
        start_position_mm_, target_position_mm_);
    } else {
      target_position_mm_ = 0.0;
      RCLCPP_INFO(
        get_logger(),
        "Origin reset started: repeatedly sending CAN ID 0x281, data=0x01");
    }

    // Send immediately, then repeat at control_period_ms until stable arrival.
    send_active_command();
    next_command_at_ =
      SteadyClock::now() + std::chrono::milliseconds(control_period_ms_);
  }

  void on_can_rx(const can_msgs::msg::Frame::SharedPtr message)
  {
    if (!message ||
        message->id != kPositionFeedbackCanId ||
        message->is_extended ||
        message->is_rtr ||
        message->is_error ||
        message->dlc != 4) {
      return;
    }

    float position = 0.0F;
    if (!unpack_float32_le(message->data.data(), position) ||
        !std::isfinite(position)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignored invalid float32 position feedback on CAN ID 0x180");
      return;
    }

    current_position_mm_ = static_cast<double>(position);
    last_feedback_received_at_ = SteadyClock::now();
    feedback_valid_ = true;
    ++feedback_generation_;

    if (!startup_position_valid_) {
      startup_position_mm_ = current_position_mm_;
      startup_position_valid_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Startup reference position captured: %.3f mm",
        startup_position_mm_);
    }

    if (!active_goal_ ||
        feedback_generation_ <= feedback_generation_at_start_) {
      return;
    }

    const bool within_tolerance =
      std::abs(current_position_mm_ - target_position_mm_) <=
      position_tolerance_mm_;

    if (within_tolerance) {
      if (!in_tolerance_since_) {
        in_tolerance_since_ = last_feedback_received_at_;
        stable_arrival_confirmed_ = stable_time_sec_ <= 0.0;
      } else {
        const auto stable_elapsed = std::chrono::duration<double>(
          last_feedback_received_at_ - *in_tolerance_since_).count();
        if (stable_elapsed >= stable_time_sec_) {
          stable_arrival_confirmed_ = true;
        }
      }
      phase_ = kPhaseVerifying;
    } else {
      in_tolerance_since_.reset();
      stable_arrival_confirmed_ = false;
      phase_ = kPhaseCommanding;
    }
  }

  void tick()
  {
    if (!active_goal_) {
      return;
    }

    if (active_goal_->is_canceling()) {
      finish_canceled();
      return;
    }

    const auto now = SteadyClock::now();

    const auto feedback_reference =
      feedback_generation_ > feedback_generation_at_start_
      ? last_feedback_received_at_
      : action_started_at_;

    const auto feedback_age =
      std::chrono::duration<double>(now - feedback_reference).count();

    if (feedback_age >
        static_cast<double>(feedback_timeout_ms_) / 1000.0) {
      finish_aborted("position feedback timeout");
      return;
    }

    const auto motion_elapsed =
      std::chrono::duration<double>(now - action_started_at_).count();

    if (motion_elapsed > motion_timeout_sec_) {
      finish_aborted("motion timeout");
      return;
    }

    if (stable_arrival_confirmed_) {
      finish_succeeded();
      return;
    }

    if (now >= next_command_at_) {
      send_active_command();
      next_command_at_ =
        now + std::chrono::milliseconds(control_period_ms_);
    }

    publish_action_feedback();
  }

  void send_active_command()
  {
    if (!active_goal_) {
      return;
    }

    if (active_action_ == ActionKind::EXECUTE) {
      publish_position_target(target_position_mm_);
    } else if (active_action_ == ActionKind::RESET_ORIGIN) {
      publish_reset_origin();
    }
  }

  void publish_position_target(const double position_mm)
  {
    if (!std::isfinite(position_mm)) {
      return;
    }

    can_msgs::msg::Frame frame;
    frame.header.stamp = now();
    frame.id = kPositionTargetCanId;
    frame.is_extended = false;
    frame.is_rtr = false;
    frame.is_error = false;
    frame.dlc = 4;
    frame.data.fill(0);
    pack_float32_le(static_cast<float>(position_mm), frame.data.data());
    can_tx_pub_->publish(frame);
  }

  void publish_reset_origin()
  {
    can_msgs::msg::Frame frame;
    frame.header.stamp = now();
    frame.id = kResetOriginCanId;
    frame.is_extended = false;
    frame.is_rtr = false;
    frame.is_error = false;
    frame.dlc = 1;
    frame.data.fill(0);
    frame.data[0] = 1;
    can_tx_pub_->publish(frame);
  }

  static bool unpack_float32_le(
    const uint8_t * data,
    float & value)
  {
    if (data == nullptr) {
      return false;
    }

    const uint32_t bits =
      static_cast<uint32_t>(data[0]) |
      (static_cast<uint32_t>(data[1]) << 8U) |
      (static_cast<uint32_t>(data[2]) << 16U) |
      (static_cast<uint32_t>(data[3]) << 24U);

    static_assert(sizeof(value) == sizeof(bits), "float must be 32-bit");
    std::memcpy(&value, &bits, sizeof(value));
    return std::isfinite(value);
  }

  static void pack_float32_le(const float value, uint8_t * data)
  {
    static_assert(sizeof(value) == sizeof(uint32_t), "float must be 32-bit");

    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));

    data[0] = static_cast<uint8_t>((bits >> 0U) & 0xFFU);
    data[1] = static_cast<uint8_t>((bits >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((bits >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((bits >> 24U) & 0xFFU);
  }

  bool has_fresh_feedback() const
  {
    if (!feedback_valid_) {
      return false;
    }

    const auto age =
      std::chrono::duration<double>(
        SteadyClock::now() - last_feedback_received_at_).count();

    return age <= static_cast<double>(feedback_timeout_ms_) / 1000.0;
  }

  void publish_action_feedback()
  {
    if (!active_goal_) {
      return;
    }

    auto feedback = std::make_shared<Sequence::Feedback>();
    feedback->phase = phase_;

    if (active_action_ == ActionKind::EXECUTE) {
      const double travel = target_position_mm_ - start_position_mm_;
      const double moved = current_position_mm_ - start_position_mm_;
      const double ratio = travel > 0.0 ? moved / travel : 0.0;
      feedback->progress = static_cast<float>(
        std::clamp(ratio, 0.0, 1.0));
      feedback->active_actuator = "reload";

      std::ostringstream detail;
      detail << std::fixed << std::setprecision(2)
             << "current=" << current_position_mm_
             << " mm, target=" << target_position_mm_ << " mm";
      feedback->detail = detail.str();
    } else {
      const bool near_origin =
        feedback_generation_ > feedback_generation_at_start_ &&
        std::abs(current_position_mm_) <= position_tolerance_mm_;
      feedback->progress = near_origin ? 0.9F : 0.0F;
      feedback->active_actuator = "reload";

      std::ostringstream detail;
      detail << std::fixed << std::setprecision(2)
             << "waiting for position near 0 mm; current="
             << (feedback_valid_ ? current_position_mm_ : 0.0)
             << " mm";
      feedback->detail = detail.str();
    }

    active_goal_->publish_feedback(feedback);
  }

  std::shared_ptr<Sequence::Result> make_result(
    const bool success,
    const uint8_t final_state,
    const uint16_t error_code,
    const std::string & reason) const
  {
    auto result = std::make_shared<Sequence::Result>();
    result->success = success;
    result->final_state = final_state;
    result->error_code = error_code;
    result->failed_actuator = success ? "" : "reload";
    result->reason = reason;
    return result;
  }

  void finish_succeeded()
  {
    phase_ = kPhaseComplete;
    const auto result = make_result(
      true, kPhaseComplete, 0U,
      active_action_ == ActionKind::EXECUTE
        ? "reload movement complete"
        : "origin reset complete");

    auto goal = active_goal_;
    clear_active_action();
    goal->succeed(result);

    RCLCPP_INFO(
      get_logger(), "Action completed successfully; final position=%.3f mm",
      current_position_mm_);
  }

  void finish_aborted(const std::string & reason)
  {
    phase_ = kPhaseError;
    const auto result = make_result(false, kPhaseError, 1U, reason);

    auto goal = active_goal_;
    clear_active_action();
    if (goal) {
      goal->abort(result);
    }

    RCLCPP_WARN(get_logger(), "Action aborted: %s", reason.c_str());
  }

  void finish_canceled()
  {
    const bool fresh = has_fresh_feedback();
    const bool post_reset_feedback =
      active_action_ != ActionKind::RESET_ORIGIN ||
      feedback_generation_ > feedback_generation_at_start_;

    bool hold_command_sent = false;
    if (fresh && post_reset_feedback) {
      // The cancellation hold command is sent once; repeated target commands
      // are used only during normal execute/reset_origin operation.
      publish_position_target(current_position_mm_);
      hold_command_sent = true;
    }

    const std::string reason = hold_command_sent
      ? "canceled; current position sent as hold target"
      : "canceled; feedback unavailable or stale, hold target not sent";

    phase_ = kPhaseCanceled;
    const auto result = make_result(false, kPhaseCanceled, 1U, reason);

    auto goal = active_goal_;
    clear_active_action();
    if (goal) {
      goal->canceled(result);
    }

    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
  }

  void clear_active_action()
  {
    active_goal_.reset();
    active_action_ = ActionKind::NONE;
    goal_reserved_ = false;
    // A shot that started before this reload/reset finished must not unlock
    // another reload. A new RUNNING -> READY transition is required.
    shot_running_seen_ = false;
    publish_reload_ready(false);
    in_tolerance_since_.reset();
    stable_arrival_confirmed_ = false;
    phase_ = kPhaseIdle;
  }

  static constexpr uint8_t kShootRunningState = 1U;
  static constexpr uint8_t kShootReadyState = 2U;

  std::string operation_state_topic_;
  std::string ready_topic_;
  std::optional<uint8_t> shoot_operation_state_;
  bool shot_running_seen_{false};
  bool reload_ready_{false};
  bool goal_reserved_{false};

  double step_mm_{100.0};
  double position_tolerance_mm_{3.0};
  double stable_time_sec_{0.1};
  int feedback_timeout_ms_{500};
  double motion_timeout_sec_{15.0};
  int control_period_ms_{20};

  bool feedback_valid_{false};
  bool startup_position_valid_{false};
  double current_position_mm_{0.0};
  double startup_position_mm_{0.0};
  double start_position_mm_{0.0};
  double target_position_mm_{0.0};
  uint64_t feedback_generation_{0};
  uint64_t feedback_generation_at_start_{0};

  SteadyTime last_feedback_received_at_{};
  SteadyTime action_started_at_{};
  SteadyTime next_command_at_{};
  std::optional<SteadyTime> in_tolerance_since_;
  bool stable_arrival_confirmed_{false};

  ActionKind active_action_{ActionKind::NONE};
  uint8_t phase_{kPhaseIdle};
  std::shared_ptr<GoalHandle> active_goal_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr shoot_operation_state_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reload_ready_pub_;
  rclcpp_action::Server<Sequence>::SharedPtr execute_server_;
  rclcpp_action::Server<Sequence>::SharedPtr reset_origin_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<BeltReloadNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("belt_reload"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
