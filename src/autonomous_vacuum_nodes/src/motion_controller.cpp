#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "autonomous_vacuum_interfaces/msg/floor_config.hpp"
#include "autonomous_vacuum_interfaces/msg/session_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using FloorConfig = autonomous_vacuum_interfaces::msg::FloorConfig;
using SessionCommand = autonomous_vacuum_interfaces::msg::SessionCommand;
using Twist = geometry_msgs::msg::Twist;
using Odom = nav_msgs::msg::Odometry;
using Path = nav_msgs::msg::Path;

class MotionController : public rclcpp::Node
{
public:
  MotionController() : Node("motion_controller")
  {
    velocity_pub_ = create_publisher<Twist>("/vacuum/navigation/cmd_vel", 1);
    floor_sub_ = create_subscription<FloorConfig>(
      "/vacuum/floor/config", rclcpp::QoS(1).transient_local(), [this](const FloorConfig & floor) {
        stop();
        max_speed_ = 0.0;
        floor_id_ = floor.floor_id;
        cell_size_ = floor.grid.info.resolution;
      });
    session_sub_ = create_subscription<SessionCommand>(
      "/vacuum/session_state", 10, [this](const SessionCommand & cmd) {
        if (!cmd.running) {
          max_speed_ = 0.0;
          stop();
        } else if (cmd.floor_id == floor_id_) {
          docking_ = cmd.mode == "RETURN_TO_DOCK";
          max_speed_ = cell_size_ * (cmd.power_mode == "TURBO" ? 3.0 : 2.0);
        }
      });
    path_sub_ = create_subscription<Path>(
      "/vacuum/navigation/path", 1, [this](const Path & path) {
        if (path.poses.empty()) {
          stop();
        } else {
          path_ = path;
          waypoint_ = 0;
        }
      });
    odom_sub_ = create_subscription<Odom>(
      "/vacuum/sensors/odom", 1, [this](const Odom & msg) { follow_path(msg.pose.pose); });
  }

private:
  void stop()
  {
    if (!path_.poses.empty()) {
      velocity_pub_->publish(Twist{});
    }
    path_.poses.clear();
    waypoint_ = 0;
  }

  void follow_path(const geometry_msgs::msg::Pose & pose)
  {
    if (path_.poses.empty() || max_speed_ <= 0.0) {
      return;
    }
    // Reach each cell center before turning into a one-cell-wide corridor.
    const auto & goal = path_.poses[waypoint_].pose.position;
    const double dx = goal.x - pose.position.x;
    const double dy = goal.y - pose.position.y;
    const double distance = std::hypot(dx, dy);
    if (distance <= 0.008) {
      if (docking_ && waypoint_ + 1 == path_.poses.size()) {
        const auto & orientation = path_.poses.back().pose.orientation;
        const double wanted = 2.0 * std::atan2(orientation.z, orientation.w);
        const double yaw = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
        const double error = std::remainder(wanted - yaw, 2.0 * std::acos(-1.0));
        if (std::abs(error) > 0.02) {
          Twist turn;
          turn.angular.z = std::clamp(6.0 * error, -3.0, 3.0);
          velocity_pub_->publish(turn);
          return;
        }
      }
      ++waypoint_;
      velocity_pub_->publish(Twist{});
      if (waypoint_ == path_.poses.size()) {
        path_.poses.clear();
      }
      return;
    }

    const double yaw = 2.0 * std::atan2(pose.orientation.z, pose.orientation.w);
    const double heading_error = std::remainder(std::atan2(dy, dx) - yaw, 2.0 * std::acos(-1.0));
    Twist velocity;
    velocity.angular.z = std::clamp(6.0 * heading_error, -3.0, 3.0);
    if (std::abs(heading_error) < 0.02) {
      velocity.linear.x = std::min(max_speed_, 10.0 * distance);
    }
    velocity_pub_->publish(velocity);
  }

  std::string floor_id_;
  bool docking_ = false;
  double cell_size_ = 0.0;
  double max_speed_ = 0.0;
  Path path_;
  std::size_t waypoint_ = 0;
  rclcpp::Publisher<Twist>::SharedPtr velocity_pub_;
  rclcpp::Subscription<FloorConfig>::SharedPtr floor_sub_;
  rclcpp::Subscription<SessionCommand>::SharedPtr session_sub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionController>());
  rclcpp::shutdown();
  return 0;
}
