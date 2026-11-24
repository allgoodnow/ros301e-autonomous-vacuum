#include "autonomous_vacuum_nodes/floor_markers.hpp"
#include "autonomous_vacuum_nodes/energy_model.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <queue>
#include <vector>

#include "autonomous_vacuum_interfaces/msg/coverage.hpp"
#include "autonomous_vacuum_interfaces/msg/floor_config.hpp"
#include "autonomous_vacuum_interfaces/msg/session_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

using Coverage = autonomous_vacuum_interfaces::msg::Coverage;
using FloorConfig = autonomous_vacuum_interfaces::msg::FloorConfig;
using SessionCommand = autonomous_vacuum_interfaces::msg::SessionCommand;
using Pose = geometry_msgs::msg::Pose;
using Path = nav_msgs::msg::Path;
using Odom = nav_msgs::msg::Odometry;

// Counterclockwise cardinal directions; each pair is (column step, row step).
constexpr std::array<std::array<int, 2>, 4> directions{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}};

class CoveragePlanner : public rclcpp::Node
{
public:
  CoveragePlanner() : Node("coverage_planner")
  {
    const auto retained = rclcpp::QoS(1).transient_local();
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/vacuum/visualization/floor", retained);
    // RViz may receive the retained scene before its TF buffer has both robot transforms.
    // Refresh the full scene so an idle robot and late subscribers recover automatically.
    scene_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      if (floor_ && localized_pose_) {
        marker_pub_->publish(make_floor_markers(*floor_, cleaned_));
      }
    });
    path_pub_ = create_publisher<Path>("/vacuum/navigation/path", retained);
    progress_pub_ = create_publisher<Coverage>("/vacuum/cleaning/coverage", 10);
    floor_sub_ = create_subscription<FloorConfig>(
      "/vacuum/floor/config", rclcpp::QoS(1).transient_local(),
      [this](const FloorConfig & config) {
        stop();
        floor_ = config;
        cleaned_.clear();
        cleaning_task_id_.clear();
        marker_pub_->publish(make_floor_markers(*floor_, cleaned_));
        pose_.reset();
        localized_pose_.reset();
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/vacuum/mapping/map", rclcpp::QoS(1).transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid & map) {
        if (floor_ && map.info.map_load_time == floor_->grid.info.map_load_time) {
          floor_->grid = map;
          marker_pub_->publish(make_floor_markers(*floor_, cleaned_));
        }
      });
    localized_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/vacuum/localization/pose", 5, [this](const geometry_msgs::msg::PoseStamped & estimate) {
        if (floor_ && rclcpp::Time(estimate.header.stamp) >= rclcpp::Time(floor_->grid.info.map_load_time)) {
          localized_pose_ = estimate.pose;
        }
      });
    session_sub_ = create_subscription<SessionCommand>(
      "/vacuum/session_state", 10, [this](const SessionCommand & command) { on_session(command); });
    odom_sub_ = create_subscription<Odom>(
      "/vacuum/sensors/odom", 1, [this](const Odom & msg) {
        if (!floor_ || rclcpp::Time(msg.header.stamp) < rclcpp::Time(floor_->grid.info.map_load_time)) {
          return;
        }
        pose_ = msg.pose.pose;
        if (running_) {
          update_coverage();
        }
      });
  }

private:
  // -1 means outside the map, occupied, or unknown.
  int free_cell(int col, int row) const
  {
    const auto & grid = floor_->grid;
    const int width = static_cast<int>(grid.info.width);
    if (row < 0 || row >= static_cast<int>(grid.info.height) || col < 0 || col >= width) {
      return -1;
    }
    const int index = row * width + col;
    return grid.data[index] == 0 ? index : -1;
  }

  int current_cell() const
  {
    const double size = floor_->grid.info.resolution;
    const Pose & estimate = localized_pose_ ? *localized_pose_ : *pose_;
    return free_cell(std::floor(estimate.position.x / size), std::floor(estimate.position.y / size));
  }

  Pose cell_center(int index) const
  {
    const int width = floor_->grid.info.width;
    const double size = floor_->grid.info.resolution;
    Pose result;
    result.position.x = (index % width + 0.5) * size;
    result.position.y = (index / width + 0.5) * size;
    result.orientation.w = 1.0;
    return result;
  }

  struct GridSearch
  {
    std::vector<int> parent;
    std::vector<int> visit_order;  // Nearest reachable cells appear first.
  };

  GridSearch search(int start) const
  {
    const int width = floor_->grid.info.width;
    GridSearch result;
    result.parent.assign(floor_->grid.data.size(), -1);
    std::queue<int> frontier;
    result.parent[start] = start;
    frontier.push(start);
    while (!frontier.empty()) {
      const int cell = frontier.front();
      frontier.pop();
      result.visit_order.push_back(cell);
      for (const auto & step : directions) {
        const int next = free_cell(cell % width + step[0], cell / width + step[1]);
        if (next >= 0 && result.parent[next] < 0) {
          result.parent[next] = cell;
          frontier.push(next);
        }
      }
    }
    return result;
  }

  void on_session(const SessionCommand & cmd)
  {
    if (!cmd.running) {
      if (cmd.session_id == active_id_) {
        if (mapping_ && floor_ && floor_->map_state != "complete") {
          floor_->map_state = last_progress_.complete ? "complete" : "partial";
        }
        stop();
        if (cmd.mode != "RETURN_TO_DOCK") {
          last_progress_.stopped = true;
          progress_pub_->publish(last_progress_);
        }
      }
      return;
    }
    if (cmd.mode == "RETURN_TO_DOCK") {
      if (!running_ && floor_ && pose_ && floor_->floor_id == cmd.floor_id) {
        active_id_ = cmd.session_id;
        plan_dock_route();
      }
      return;
    }
    const bool valid_power = cmd.power_mode == "ECO" || cmd.power_mode == "STANDARD" ||
      cmd.power_mode == "TURBO";
    const bool mapping = cmd.mode == "MAP_ONLY" || cmd.mode == "SLAM_AND_CLEAN";
    if (running_ || !floor_ || !pose_ || floor_->floor_id != cmd.floor_id ||
      (!mapping && (floor_->map_state == "unexplored" || cmd.mode != "CLEAN_EXISTING_MAP")) || !valid_power)
    {
      RCLCPP_WARN(get_logger(), "Cleaning requires an idle planner and a selected, known map");
      return;
    }
    const int start = current_cell();
    if (start < 0) {
      RCLCPP_WARN(get_logger(), "Robot is outside the known free area");
      return;
    }
    active_id_ = cmd.session_id;
    last_progress_ = Coverage{};
    last_progress_.session_id = cmd.session_id;
    mapping_ = mapping;
    cleaning_ = cmd.mode != "MAP_ONLY";
    wide_footprint_ = cmd.power_mode != "ECO";
    const auto & task_id = cmd.task_id.empty() ? cmd.session_id : cmd.task_id;
    if (task_id != cleaning_task_id_ || cleaned_.empty()) {
      cleaning_task_id_ = task_id;
      cleaned_.assign(floor_->grid.data.size(), false);
      cleaned_count_ = 0;
    }
    // Bin occupancy starts over at the dock. Coverage stays cumulative for the task.
    cells_at_departure_ = cleaned_count_;
    available_bin_cells_ = cmd.available_bin_cells;
    marker_pub_->publish(make_floor_markers(*floor_, cleaned_));
    reachable_count_ = search(start).visit_order.size();
    running_ = true;
    target_ = -1;
    update_coverage();
  }

  void mark_clean(int cell)
  {
    if (cell >= 0 && !cleaned_[cell] && cleaned_count_ - cells_at_departure_ < available_bin_cells_) {
      cleaned_[cell] = true;
      ++cleaned_count_;
    }
  }

  void apply_footprint(int cell)
  {
    mark_clean(cell);
    if (!wide_footprint_) {
      return;
    }
    const int width = floor_->grid.info.width;
    const int col = cell % width;
    const int row = cell / width;
    const auto & rotation = pose_->orientation;
    const double yaw = 2.0 * std::atan2(rotation.z, rotation.w);
    const int quarter_turn = std::lround(yaw / (std::acos(-1.0) / 2.0));
    const int heading = (quarter_turn % 4 + 4) % 4;
    const auto & forward = directions[heading];
    const auto & left = directions[(heading + 1) % 4];
    const int front_cell = free_cell(col + forward[0], row + forward[1]);
    const int left_cell = free_cell(col + left[0], row + left[1]);
    mark_clean(front_cell);
    mark_clean(left_cell);
    // The diagonal must not be cleaned through the corner of a wall.
    if (front_cell >= 0 && left_cell >= 0) {
      mark_clean(free_cell(col + forward[0] + left[0], row + forward[1] + left[1]));
    }
  }

  void update_coverage()
  {
    const int cell = current_cell();
    if (cell < 0) {
      stop();
      return;
    }
    if (cleaning_) {
      const auto previous_count = cleaned_count_;
      apply_footprint(cell);
      if (cleaned_count_ != previous_count) {
        marker_pub_->publish(make_floor_markers(*floor_, cleaned_));
      }
    }
    const auto reachable = search(cell);
    reachable_count_ = reachable.visit_order.size();
    int frontier = -1;
    int uncleaned = -1;
    for (int next : reachable.visit_order) {
      if (mapping_ && frontier < 0 && beside_unknown(next)) { frontier = next; }
      if (cleaning_ && uncleaned < 0 && !cleaned_[next]) { uncleaned = next; }
    }
    const int next_target = frontier >= 0 ? frontier : uncleaned;
    const float reserve = dock_reserve(cell, reachable.parent);
    publish_progress(next_target < 0, reserve);
    if (next_target < 0 || reserve < 0 || (cleaning_ && cleaned_count_ - cells_at_departure_ >= available_bin_cells_)) {
      stop();
      return;
    }
    if (target_ >= 0) {
      const auto goal = cell_center(target_);
      if (std::hypot(goal.position.x - pose_->position.x, goal.position.y - pose_->position.y) > 0.009) {
        return;
      }
    }
    target_ = next_target;
    publish_route(cell, target_, reachable.parent, cell_center(target_));
  }

  bool beside_unknown(int cell) const
  {
    const auto & grid = floor_->grid;
    const int width = grid.info.width;
    for (const auto & step : directions) {
      const int x = cell % width + step[0];
      const int y = cell / width + step[1];
      if (x >= 0 && x < width && y >= 0 && y < static_cast<int>(grid.info.height) &&
        grid.data[y * width + x] == -1) { return true; }
    }
    return false;
  }

  void publish_progress(bool complete, float reserve)
  {
    const auto & grid = floor_->grid;
    const double clean_percent = 100.0 * cleaned_count_ / reachable_count_;
    const auto unknown = std::count(grid.data.begin(), grid.data.end(), -1);
    const double map_percent = 100.0 * (grid.data.size() - unknown) / grid.data.size();
    double percent = clean_percent;
    if (mapping_) {
      percent = cleaning_ ? (map_percent + clean_percent) / 2 : map_percent;
    }
    const float reported = complete ? 100.0 : std::min(percent, 99.0);
    if (last_progress_.floor_id == floor_->floor_id && last_progress_.completion_percent == reported &&
      last_progress_.cleaned_cells == static_cast<unsigned>(cleaned_count_) &&
      last_progress_.reachable_cells == static_cast<unsigned>(reachable_count_) &&
      last_progress_.return_reserve_percent == reserve && last_progress_.complete == complete) { return; }
    last_progress_.floor_id = floor_->floor_id;
    last_progress_.reachable_cells = reachable_count_;
    last_progress_.cleaned_cells = cleaned_count_;
    last_progress_.cleaned_area_m2 = cleaned_count_ * static_cast<double>(grid.info.resolution) * grid.info.resolution;
    last_progress_.completion_percent = reported;
    last_progress_.complete = complete;
    last_progress_.return_reserve_percent = reserve;
    progress_pub_->publish(last_progress_);
  }

  int dock_cell() const
  {
    const double size = floor_->grid.info.resolution;
    const auto & dock = floor_->dock_pose;
    return free_cell(std::floor(dock.position.x / size), std::floor(dock.position.y / size));
  }

  float dock_reserve(int start, const std::vector<int> & parents) const
  {
    const int dock = dock_cell();
    if (dock < 0 || parents[dock] < 0) { return -1.0F; }
    int steps = 0;
    for (int cell = dock; cell != start; cell = parents[cell]) { ++steps; }
    return energy::return_reserve(steps, floor_->grid.info.resolution);
  }

  void plan_dock_route()
  {
    const int start = current_cell();
    const int target = dock_cell();
    if (start < 0 || target < 0) {
      return;
    }
    // Only geometry matters for docking. The cleaning mask is never updated.
    const auto reachable = search(start);
    if (reachable.parent[target] >= 0) {
      publish_route(start, target, reachable.parent, floor_->dock_pose);
    }
  }

  void publish_route(int cell, int target, const std::vector<int> & parents, const Pose & final_pose)
  {
    std::vector<int> cells;
    for (int step = target; step != cell; step = parents[step]) {
      cells.push_back(step);
    }
    cells.push_back(cell);
    Path path;
    path.header.frame_id = "odom";  // In this simulator the grid and ideal odometry share their origin.
    path.header.stamp = now();
    for (auto it = cells.rbegin(); it != cells.rend(); ++it) {
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header = path.header;
      waypoint.pose = cell_center(*it);
      path.poses.push_back(waypoint);
    }
    path.poses.back().pose = final_pose;
    path_pub_->publish(path);
  }

  void stop()
  {
    running_ = false;
    target_ = -1;
    Path empty_path;
    empty_path.header.frame_id = "map";  // Clearing the display needs no odometry transform.
    empty_path.header.stamp = now();
    path_pub_->publish(empty_path);
  }

  std::string active_id_;
  bool running_ = false;
  std::optional<Pose> localized_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr localized_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  bool mapping_ = false;
  bool cleaning_ = false;
  bool wide_footprint_ = false;
  int target_ = -1;
  uint32_t cleaned_count_ = 0;
  std::string cleaning_task_id_;
  uint32_t cells_at_departure_ = 0;
  uint32_t available_bin_cells_ = 0;
  int reachable_count_ = 0;
  Coverage last_progress_;
  std::vector<bool> cleaned_;
  std::optional<FloorConfig> floor_;
  std::optional<Pose> pose_;
  rclcpp::Subscription<FloorConfig>::SharedPtr floor_sub_;
  rclcpp::Subscription<SessionCommand>::SharedPtr session_sub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr scene_timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;
  rclcpp::Publisher<Coverage>::SharedPtr progress_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoveragePlanner>());
  rclcpp::shutdown();
  return 0;
}
