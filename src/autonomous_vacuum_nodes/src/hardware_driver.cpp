#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autonomous_vacuum_interfaces/msg/floor_config.hpp"
#include "autonomous_vacuum_interfaces/msg/vacuum_status.hpp"
#include "autonomous_vacuum_nodes/grid_world.hpp"
#include "autonomous_vacuum_nodes/energy_model.hpp"
#include "autonomous_vacuum_interfaces/msg/coverage.hpp"
#include "autonomous_vacuum_interfaces/msg/session_command.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using FloorConfig = autonomous_vacuum_interfaces::msg::FloorConfig;
using Coverage = autonomous_vacuum_interfaces::msg::Coverage;
using Command = autonomous_vacuum_interfaces::msg::SessionCommand;
using VacuumStatus = autonomous_vacuum_interfaces::msg::VacuumStatus;
using Twist = geometry_msgs::msg::Twist;
using Odometry = nav_msgs::msg::Odometry;
using LaserScan = sensor_msgs::msg::LaserScan;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

class HardwareDriver : public rclcpp::Node
{
public:
  HardwareDriver() : Node("hardware_driver"), transform_broadcaster_(*this)
  {
    battery_ = declare_parameter<double>("initial_battery_percent", 100.0);
    const auto capacity = declare_parameter<int64_t>("dustbin_capacity_cells", 35);
    if (!std::isfinite(battery_) || battery_ < 0 || battery_ > 100 ||
      capacity < 1 || capacity > std::numeric_limits<uint32_t>::max())
    {
      throw std::invalid_argument("Battery must be in [0,100] and bin capacity a positive uint32");
    }
    bin_capacity_ = static_cast<uint32_t>(capacity);
    status_publisher_ = create_publisher<VacuumStatus>("/vacuum/status", 10);
    odom_publisher_ = create_publisher<Odometry>("/vacuum/sensors/odom", 10);
    scan_publisher_ = create_publisher<LaserScan>("/vacuum/sensors/scan", 10);

    rclcpp::QoS floor_qos(rclcpp::KeepLast(1));
    floor_qos.transient_local();
    floor_subscription_ = create_subscription<FloorConfig>(
      "/vacuum/floor/config", floor_qos,
      [this](const FloorConfig & config) { select_world(config); });
    velocity_subscription_ = create_subscription<Twist>(
      "/vacuum/navigation/cmd_vel", 1, [this](const Twist & command) {
        if (world_ && std::isfinite(command.linear.x) && std::isfinite(command.angular.z)) {
          command_ = command;
          last_command_ = Clock::now();
        }
      });

    session_sub_ = create_subscription<Command>("/vacuum/session_state", 10,
      [this](const Command & command) { on_session(command); });
    coverage_sub_ = create_subscription<Coverage>("/vacuum/cleaning/coverage", 10,
      [this](const Coverage & coverage) {
        if (coverage.session_id == cleaning_id_ && coverage.cleaned_cells > credited_cells_) {
          const auto added = coverage.cleaned_cells - credited_cells_;
          bin_cells_ += std::min(added, bin_capacity_ - bin_cells_);
          credited_cells_ = coverage.cleaned_cells;
        }
      });
    motion_timer_ = create_wall_timer(50ms, [this]() { update_motion(); });
    status_timer_ = create_wall_timer(1s, [this]() { publish_status(); });
  }

private:
  void on_session(const Command & command)
  {
    work_rate_ = 0;
    if (command.running) {
      at_charger_ = false;
      work_rate_ = energy::work_rate(command.mode, command.power_mode);
      if (command.mode != "RETURN_TO_DOCK" && command.mode != "MAP_ONLY") {
        cleaning_id_ = command.session_id;
        credited_cells_ = command.cleaned_cells_at_start;
      }
    } else if (command.docked) {
      cleaning_id_.clear();
      bin_cells_ = 0;
      at_charger_ = true;
      publish_status();
    }
  }

  void select_world(const FloorConfig & config)
  {
    command_ = Twist{};
    world_.reset();
    at_charger_ = false;
    work_rate_ = 0;
    try {
      std::string world_file;
      if (config.floor_id == "Floor_1") {
        world_file = "/config/floors/Floor_1.yaml";  // The first floor is fully known.
      } else if (config.floor_id == "Floor_2") {
        world_file = "/config/worlds/Floor_2.yaml";  // Hidden from mapping and planning.
      } else {
        throw std::runtime_error("Unknown floor: " + config.floor_id);
      }
      const auto share = ament_index_cpp::get_package_share_directory("autonomous_vacuum_nodes");
      const auto & info = config.grid.info;
      GridWorld selected_world(share + world_file, info.resolution, info.width, info.height);
      const auto & start = config.initial_pose;
      if (!selected_world.can_place_robot(start.position.x, start.position.y)) {
        throw std::runtime_error("Initial robot footprint intersects a wall");
      }
      world_ = std::move(selected_world);
      x_ = start.position.x;
      y_ = start.position.y;
      yaw_ = 2.0 * std::atan2(start.orientation.z, start.orientation.w);
      bumper_pressed_ = false;
      previous_step_ = Clock::now();
      RCLCPP_INFO(get_logger(), "Simulator ready on %s", config.floor_id.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Simulator could not start: %s", error.what());
    }
  }

  void update_motion()
  {
    if (!world_) {
      return;
    }
    const auto tick = Clock::now();
    // A delayed tick must not let a command jump across an obstacle.
    const double elapsed = std::chrono::duration<double>(tick - previous_step_).count();
    const double dt = std::min(elapsed, 0.05);
    previous_step_ = tick;
    if (tick - last_command_ > 500ms || battery_ <= 0.0) {
      command_ = Twist{};
    }

    const double speed = std::clamp(command_.linear.x, -1.5, 1.5);
    const double turn_rate = std::clamp(command_.angular.z, -3.0, 3.0);
    const double mid_heading = yaw_ + turn_rate * dt / 2.0;
    const double next_x = x_ + speed * std::cos(mid_heading) * dt;
    const double next_y = y_ + speed * std::sin(mid_heading) * dt;
    yaw_ += turn_rate * dt;

    bumper_pressed_ = !world_->can_place_robot(next_x, next_y);
    if (!bumper_pressed_) {
      x_ = next_x;
      y_ = next_y;
    }
    const double actual_speed = bumper_pressed_ ? 0.0 : speed;
    if (actual_speed != 0 || turn_rate != 0) { at_charger_ = false; }
    if (at_charger_) {
      constexpr double charge_per_second = 20.0;
      battery_ = std::min(100.0, battery_ + charge_per_second * elapsed);
    } else {
      const double used = energy::motion_cost(actual_speed * dt, turn_rate * dt) + work_rate_ * dt;
      battery_ = std::max(0.0, battery_ - used);
    }
    const auto stamp = now();
    publish_odometry(stamp, actual_speed, turn_rate);
    publish_scan(stamp);
  }

  void publish_odometry(const rclcpp::Time & stamp, double speed, double turn_rate)
  {
    Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    auto & pose = odom.pose.pose;
    pose.position.x = x_;
    pose.position.y = y_;
    pose.orientation.z = std::sin(yaw_ / 2.0);
    pose.orientation.w = std::cos(yaw_ / 2.0);
    odom.twist.twist.linear.x = speed;
    odom.twist.twist.angular.z = turn_rate;
    odom_publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation = pose.orientation;
    transform_broadcaster_.sendTransform(transform);
  }

  void publish_scan(const rclcpp::Time & stamp)
  {
    constexpr int beam_count = 72;
    const double full_turn = 2.0 * std::acos(-1.0);
    LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = "base_link";
    scan.angle_min = 0.0;
    scan.angle_increment = full_turn / beam_count;
    scan.angle_max = (beam_count - 1) * scan.angle_increment;
    scan.range_min = 0.0;
    scan.range_max = 8.0;
    scan.scan_time = 0.05;
    scan.ranges.reserve(beam_count);
    for (int beam = 0; beam < beam_count; ++beam) {
      const double angle = yaw_ + beam * scan.angle_increment;
      scan.ranges.push_back(world_->range_to_wall(x_, y_, angle, scan.range_max));
    }
    scan_publisher_->publish(scan);
  }

  void publish_status()
  {
    VacuumStatus status;
    status.battery_percent = battery_;
    status.dustbin_cells = bin_cells_;
    status.dustbin_capacity_cells = bin_capacity_;
    status.dustbin_status = bin_cells_ == bin_capacity_ ? "FULL" : "OK";
    status.charging = at_charger_ && battery_ < 100.0;
    status.cliff_detected = false;
    status.bumper_pressed = bumper_pressed_;
    status_publisher_->publish(status);
  }

  std::string cleaning_id_;
  uint32_t credited_cells_ = 0;
  uint32_t bin_cells_ = 0;
  uint32_t bin_capacity_ = 0;
  double battery_ = 100;
  double work_rate_ = 0;
  bool at_charger_ = false;
  rclcpp::Subscription<Command>::SharedPtr session_sub_;
  rclcpp::Subscription<Coverage>::SharedPtr coverage_sub_;
  std::optional<GridWorld> world_;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  bool bumper_pressed_ = false;
  Twist command_;
  Clock::time_point previous_step_{};
  Clock::time_point last_command_{};

  tf2_ros::TransformBroadcaster transform_broadcaster_;
  rclcpp::Publisher<VacuumStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Subscription<FloorConfig>::SharedPtr floor_subscription_;
  rclcpp::Subscription<Twist>::SharedPtr velocity_subscription_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr motion_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareDriver>());
  rclcpp::shutdown();
  return 0;
}
