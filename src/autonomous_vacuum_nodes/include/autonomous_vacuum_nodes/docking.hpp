#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "autonomous_vacuum_interfaces/action/return_to_dock.hpp"
#include "autonomous_vacuum_interfaces/msg/floor_config.hpp"
#include "autonomous_vacuum_interfaces/msg/session_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Runs on task_manager's single-threaded executor, alongside the cleaning action.
class Docking
{
public:
  using Action = autonomous_vacuum_interfaces::action::ReturnToDock;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using Floor = autonomous_vacuum_interfaces::msg::FloorConfig;
  using Command = autonomous_vacuum_interfaces::msg::SessionCommand;
  using Odom = nav_msgs::msg::Odometry;
  using Clock = std::chrono::steady_clock;

  Docking(rclcpp::Node & node, rclcpp::Publisher<Command>::SharedPtr publisher,
    std::function<bool()> cleaning_active);
  bool busy() const;
  void set_floor(const Floor & floor);
  void observe(const Odom & odom);
  using ReturnCallback = std::function<void(bool docked)>;
  void request_return(const std::string & floor_id, const std::string & parent_session = {},
    ReturnCallback on_finished = {});

private:
  void start(std::shared_ptr<Handle> goal);
  void tick();
  void stop(const std::string & failure);
  void finish();

  rclcpp::Node & node_;
  rclcpp::Publisher<Command>::SharedPtr commands_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Handle> goal_;
  std::optional<Floor> floor_;
  std::optional<Clock::time_point> odom_received_;
  Command command_;
  std::string pending_parent_;
  bool automatic_pending_ = false;
  bool stopping_ = false;
  bool stationary_ = false;
  bool at_dock_ = false;
  double distance_ = 0;
  std::string failure_;
  Clock::time_point started_, last_motion_, next_feedback_, stop_requested_;
  rclcpp::Time stop_stamp_{0, 0, RCL_ROS_TIME};
};
