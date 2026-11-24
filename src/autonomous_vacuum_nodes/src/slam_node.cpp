#include <algorithm>
#include <array>
#include <string>
#include <cmath>
#include <memory>
#include <optional>

#include "autonomous_vacuum_interfaces/msg/floor_config.hpp"
#include "autonomous_vacuum_interfaces/msg/session_command.hpp"
#include "autonomous_vacuum_interfaces/msg/map_snapshot.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using Floor = autonomous_vacuum_interfaces::msg::FloorConfig;
using MapSnapshot = autonomous_vacuum_interfaces::msg::MapSnapshot;
using Command = autonomous_vacuum_interfaces::msg::SessionCommand;
using Grid = nav_msgs::msg::OccupancyGrid;
using Scan = sensor_msgs::msg::LaserScan;
using Odom = nav_msgs::msg::Odometry;
using Pose = geometry_msgs::msg::PoseStamped;

class SlamNode : public rclcpp::Node
{
public:
  SlamNode() : Node("slam_node"), transforms_(*this)
  {
    const auto retained = rclcpp::QoS(1).transient_local();
    snapshot_pub_ = create_publisher<MapSnapshot>("/vacuum/mapping/snapshot", 5);
    map_pub_ = create_publisher<Grid>("/vacuum/mapping/map", retained);
    pose_pub_ = create_publisher<Pose>("/vacuum/localization/pose", 5);
    floor_sub_ = create_subscription<Floor>("/vacuum/floor/config", retained, [this](const Floor & f) {
      floor_id_ = f.floor_id;
      map_ = f.grid;
      dx_ = dy_ = 0;
      mapping_ = false;
      mapping_session_.clear();
      scan_.reset();
      odom_.reset();
      map_pub_->publish(*map_);
    });
    command_sub_ = create_subscription<Command>("/vacuum/session_state", 10, [this](const Command & c) {
      if (!map_ || c.floor_id != floor_id_) { return; }
      if (c.running && (c.mode == "MAP_ONLY" || c.mode == "SLAM_AND_CLEAN")) {
        mapping_ = true;
        mapping_session_ = c.session_id;
      } else if (!c.running && mapping_ && c.session_id == mapping_session_) {
        mapping_ = false;
        MapSnapshot snapshot;
        snapshot.session_id = mapping_session_;
        snapshot.grid = *map_;
        snapshot_pub_->publish(snapshot);
        mapping_session_.clear();
      }
    });
    odom_sub_ = create_subscription<Odom>("/vacuum/sensors/odom", 5, [this](Odom::ConstSharedPtr odom) {
      odom_ = odom;
      process_pair();
    });
    scan_sub_ = create_subscription<Scan>("/vacuum/sensors/scan", 5, [this](Scan::ConstSharedPtr scan) {
      scan_ = scan;
      process_pair();
    });
  }

private:
  // Project the nearest laser beam onto a cardinal grid axis.
  double axial_range(double yaw, int direction) const
  {
    const double turn = 2.0 * std::acos(-1.0);
    const double desired = direction * turn / 4 - yaw - scan_->angle_min;
    const int count = static_cast<int>(scan_->ranges.size());
    const int nearest = std::lround(desired / scan_->angle_increment);
    const int beam = (nearest % count + count) % count;
    return scan_->ranges[beam] * std::cos(desired - beam * scan_->angle_increment);
  }

  void process_pair()
  {
    if (!map_ || !odom_ || !scan_ || odom_->header.stamp != scan_->header.stamp ||
      rclcpp::Time(scan_->header.stamp) < rclcpp::Time(map_->info.map_load_time) ||
      scan_->ranges.empty() || scan_->angle_increment <= 0)
    {
      return;
    }
    Pose estimate;
    estimate.header = scan_->header;
    estimate.header.frame_id = "map";
    estimate.pose = odom_->pose.pose;
    estimate.pose.position.x += dx_;
    estimate.pose.position.y += dy_;
    const double size = map_->info.resolution;
    const auto & p = estimate.pose.position;
    const int col = std::floor(p.x / size);
    const int row = std::floor(p.y / size);
    // From near a cell center the four beam projections stay inside their grid lanes.
    if (std::abs(p.x - (col + 0.5) * size) < size * 0.18 &&
      std::abs(p.y - (row + 0.5) * size) < size * 0.18)
    {
      observe_lanes(estimate, col, row);
    }
    estimate.pose.position.x = odom_->pose.pose.position.x + dx_;
    estimate.pose.position.y = odom_->pose.pose.position.y + dy_;
    pose_pub_->publish(estimate);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = estimate.header;
    transform.child_frame_id = "odom";
    transform.transform.translation.x = dx_;
    transform.transform.translation.y = dy_;
    transform.transform.rotation.w = 1.0;
    transforms_.sendTransform(transform);
    scan_.reset();
  }

  void observe_lanes(const Pose & estimate, int col, int row)
  {
    constexpr std::array<int, 4> column_step{1, 0, -1, 0};
    constexpr std::array<int, 4> row_step{0, 1, 0, -1};
    const double size = map_->info.resolution;
    const auto & p = estimate.pose.position;
    const auto & q = estimate.pose.orientation;
    const double yaw = 2.0 * std::atan2(q.z, q.w);
    double correction_x = 0, correction_y = 0;
    int matches_x = 0, matches_y = 0;
    bool changed = false;
    for (int d = 0; d < 4; ++d) {
      const double range = axial_range(yaw, d);
      if (!std::isfinite(range) || range < scan_->range_min) {
        continue;
      }
      for (int step = 1; step <= 3; ++step) {
        const int x = col + step * column_step[d];
        const int y = row + step * row_step[d];
        if (x < 0 || y < 0 || x >= static_cast<int>(map_->info.width) ||
          y >= static_cast<int>(map_->info.height)) { break; }
        const double near_face =
          column_step[d] * ((x + 0.5) * size - p.x) +
          row_step[d] * ((y + 0.5) * size - p.y) - size / 2;
        const double residual = near_face - range;
        const bool hit = std::abs(residual) < size * 0.22;
        if (residual >= size * 0.22) { break; }
        auto & cell = map_->data[y * map_->info.width + x];
        // Only a previously known wall may correct the odometry prediction.
        if (cell == 100 && hit) {
          correction_x += column_step[d] * residual;
          correction_y += row_step[d] * residual;
          matches_x += column_step[d] != 0;
          matches_y += row_step[d] != 0;
        }
        if (mapping_ && cell == -1) {
          cell = hit ? 100 : 0;
          changed = true;
        }
        if (hit || cell == 100) { break; }
      }
    }
    if (matches_x) { dx_ += 0.5 * correction_x / matches_x; }
    if (matches_y) { dy_ += 0.5 * correction_y / matches_y; }
    if (changed) {
      map_->header.stamp = estimate.header.stamp;
      map_pub_->publish(*map_);
    }
  }

  std::string floor_id_;
  std::optional<Grid> map_;
  Odom::ConstSharedPtr odom_;
  Scan::ConstSharedPtr scan_;
  double dx_ = 0, dy_ = 0;
  std::string mapping_session_;
  bool mapping_ = false;
  rclcpp::Publisher<MapSnapshot>::SharedPtr snapshot_pub_;
  tf2_ros::TransformBroadcaster transforms_;
  rclcpp::Publisher<Grid>::SharedPtr map_pub_;
  rclcpp::Publisher<Pose>::SharedPtr pose_pub_;
  rclcpp::Subscription<Floor>::SharedPtr floor_sub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Subscription<Scan>::SharedPtr scan_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamNode>());
  rclcpp::shutdown();
  return 0;
}
