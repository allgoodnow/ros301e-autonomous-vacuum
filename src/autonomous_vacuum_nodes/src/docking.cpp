#include "autonomous_vacuum_nodes/docking.hpp"

#include <cmath>
#include <utility>

using namespace std::chrono_literals;

Docking::Docking(rclcpp::Node & node, rclcpp::Publisher<Command>::SharedPtr publisher,
  std::function<bool()> cleaning_active)
: node_(node), commands_(std::move(publisher))
{
  const std::string action_name = "/vacuum/return_to_dock";
  server_ = rclcpp_action::create_server<Action>(&node_, action_name,
    [this, cleaning_active](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
      const bool fresh_pose = odom_received_ && Clock::now() - *odom_received_ < 1s;
      const bool requested_return = automatic_pending_ && goal->parent_session_id == pending_parent_;
      if (goal_ || ((automatic_pending_ || cleaning_active()) && !requested_return) || !floor_ || !fresh_pose ||
        (!goal->floor_id.empty() && goal->floor_id != floor_->floor_id))
      {
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](std::shared_ptr<Handle> goal) {
      return goal == goal_ && !stopping_ ? rclcpp_action::CancelResponse::ACCEPT :
        rclcpp_action::CancelResponse::REJECT;
    },
    [this](std::shared_ptr<Handle> goal) { start(std::move(goal)); });
  client_ = rclcpp_action::create_client<Action>(&node_, action_name);
  timer_ = node_.create_wall_timer(50ms, [this]() { tick(); });
}

bool Docking::busy() const
{
  return automatic_pending_ || goal_ != nullptr;
}

void Docking::request_return(const std::string & floor_id, const std::string & parent_session,
  ReturnCallback on_finished)
{
  Action::Goal request;
  request.floor_id = floor_id;
  request.parent_session_id = parent_session;
  pending_parent_ = parent_session;
  automatic_pending_ = true;
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.goal_response_callback = [this, on_finished](const rclcpp_action::ClientGoalHandle<Action>::SharedPtr & handle) {
    automatic_pending_ = false;
    if (!handle) {
      RCLCPP_WARN(node_.get_logger(), "Automatic docking request was rejected");
      if (on_finished) { on_finished(false); }
    }
  };
  options.result_callback = [on_finished](const rclcpp_action::ClientGoalHandle<Action>::WrappedResult & result) {
    if (on_finished) {
      on_finished(result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->docked);
    }
  };
  client_->async_send_goal(request, options);
}

void Docking::set_floor(const Floor & floor)
{
  if (goal_) {
    stop("Floor changed during docking");
  }
  floor_ = floor;
  odom_received_.reset();
  at_dock_ = false;
}

void Docking::observe(const Odom & odom)
{
  if (!floor_ || rclcpp::Time(odom.header.stamp) < rclcpp::Time(floor_->grid.header.stamp)) {
    return;
  }
  odom_received_ = Clock::now();
  const auto & pose = odom.pose.pose;
  const auto & dock = floor_->dock_pose;
  distance_ = std::hypot(pose.position.x - dock.position.x, pose.position.y - dock.position.y);
  const double yaw = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
  const double wanted = 2.0 * std::atan2(dock.orientation.z, dock.orientation.w);
  const double angle = std::remainder(wanted - yaw, 2.0 * std::acos(-1.0));
  at_dock_ = distance_ <= 0.008 && std::abs(angle) <= 0.02;
  const auto & velocity = odom.twist.twist;
  const bool moving = std::abs(velocity.linear.x) > 0.001 || std::abs(velocity.angular.z) > 0.001;
  if (moving) {
    last_motion_ = Clock::now();
  }
  if (stopping_ && rclcpp::Time(odom.header.stamp) > stop_stamp_) {
    stationary_ = !moving;
  }
}

void Docking::start(std::shared_ptr<Handle> goal)
{
  goal_ = std::move(goal);
  started_ = last_motion_ = next_feedback_ = Clock::now();
  stopping_ = stationary_ = false;
  failure_.clear();
  command_.floor_id = floor_->floor_id;
  command_.session_id = rclcpp_action::to_string(goal_->get_goal_id());
  command_.mode = "RETURN_TO_DOCK";
  command_.power_mode = "ECO";
  command_.docked = false;
  command_.running = true;
  commands_->publish(command_);
  RCLCPP_INFO(node_.get_logger(), "Returning to dock on %s", command_.floor_id.c_str());
}

void Docking::stop(const std::string & failure)
{
  if (stopping_) {
    return;
  }
  stopping_ = true;
  failure_ = failure;
  stop_requested_ = Clock::now();
  stop_stamp_ = node_.now();
  stationary_ = false;
  command_.running = false;
  commands_->publish(command_);
}

void Docking::tick()
{
  if (!goal_) {
    return;
  }
  const auto now = Clock::now();
  if (stopping_) {
    if (stationary_) {
      finish();
    } else if (now - stop_requested_ > 3s) {
      failure_ = "Could not confirm motion stop";
      finish();
    }
    return;
  }
  if (goal_->is_canceling() || at_dock_) {
    stop("");
  } else if (!odom_received_ || now - *odom_received_ > 2s) {
    stop("Lost floor odometry");
  } else if (now - last_motion_ > 5s) {
    stop("No progress along the dock route");
  } else if (now >= next_feedback_) {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->distance_to_dock_m = distance_;
    goal_->publish_feedback(feedback);
    next_feedback_ = now + 1s;
  }
}

void Docking::finish()
{
  auto result = std::make_shared<Action::Result>();
  result->elapsed_seconds = std::chrono::duration<double>(Clock::now() - started_).count();
  if (goal_->is_canceling()) {
    result->message = failure_.empty() ? "Docking canceled; robot stopped" : failure_;
    goal_->canceled(result);
  } else if (failure_.empty() && at_dock_ && stationary_) {
    command_.docked = true;
    commands_->publish(command_);
    result->docked = true;
    result->message = "At dock, aligned and stationary";
    goal_->succeed(result);
  } else {
    result->message = failure_.empty() ? "Robot left the docking tolerance" : failure_;
    goal_->abort(result);
  }
  RCLCPP_INFO(node_.get_logger(), "%s", result->message.c_str());
  goal_.reset();
}
