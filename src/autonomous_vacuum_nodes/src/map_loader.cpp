#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "autonomous_vacuum_interfaces/msg/floor_config.hpp"
#include "autonomous_vacuum_interfaces/srv/set_floor_level.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

#include "autonomous_vacuum_interfaces/srv/save_map.hpp"

using SaveMap = autonomous_vacuum_interfaces::srv::SaveMap;
using FloorConfig = autonomous_vacuum_interfaces::msg::FloorConfig;
using SetFloorLevel = autonomous_vacuum_interfaces::srv::SetFloorLevel;

namespace
{
struct GridPose
{
  geometry_msgs::msg::Pose pose;
  double yaw_radians;
};

GridPose read_pose(const YAML::Node & entry, const nav_msgs::msg::OccupancyGrid & grid)
{
  if (!entry.IsSequence() || entry.size() != 3) {
    throw std::runtime_error("Expected pose [column, row, heading_degrees]");
  }
  const int col = entry[0].as<int>();
  const int row = entry[1].as<int>();
  const bool inside_grid = col >= 0 && row >= 0 &&
    static_cast<unsigned>(col) < grid.info.width &&
    static_cast<unsigned>(row) < grid.info.height;
  if (!inside_grid) {
    throw std::runtime_error("Pose lies outside the grid");
  }

  const auto cell_index = static_cast<std::size_t>(row) * grid.info.width + col;
  if (grid.data[cell_index] != 0) {
    throw std::runtime_error("Pose must lie on a known free cell");
  }

  GridPose result;
  // The physical pose is at the center of the cell, in metres.
  result.pose.position.x = (col + 0.5) * grid.info.resolution;
  result.pose.position.y = (row + 0.5) * grid.info.resolution;
  const double heading_degrees = entry[2].as<double>();
  constexpr double radians_per_degree = 0.017453292519943295;  // pi / 180
  result.yaw_radians = heading_degrees * radians_per_degree;
  if (!std::isfinite(result.yaw_radians)) {
    throw std::runtime_error("Heading must be finite");
  }
  // A rotation about the vertical axis has only z and w quaternion components.
  result.pose.orientation.z = std::sin(result.yaw_radians / 2.0);
  result.pose.orientation.w = std::cos(result.yaw_radians / 2.0);
  return result;
}

std::int8_t occupancy_value(char cell)
{
  switch (cell) {
    case '.': return 0;    // Free
    case '#': return 100;  // Occupied
    case '?': return -1;   // Unknown
    default: throw std::runtime_error("Unrecognized grid cell");
  }
}

nav_msgs::msg::OccupancyGrid read_grid(const YAML::Node & file)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.origin.orientation.w = 1.0;
  grid.info.resolution = file["resolution"].as<float>();
  const auto rows = file["grid"].as<std::vector<std::string>>();
  if (!std::isfinite(grid.info.resolution) || grid.info.resolution <= 0 ||
    rows.empty() || rows.front().empty())
  {
    throw std::runtime_error("Expected a nonempty grid and a positive resolution");
  }
  grid.info.height = rows.size();
  grid.info.width = rows.front().size();
  for (const auto & row : rows) {
    if (row.size() != grid.info.width) {
      throw std::runtime_error("Grid rows must have the same width");
    }
    for (char cell : row) {
      grid.data.push_back(occupancy_value(cell));
    }
  }
  return grid;
}
}  // namespace

class MapLoader : public rclcpp::Node
{
public:
  MapLoader() : Node("map_loader")
  {
    floor_dir_ = ament_index_cpp::get_package_share_directory("autonomous_vacuum_nodes") +
      "/config/floors/";
    const char * ros_home = std::getenv("ROS_HOME");
    const auto state_dir = ros_home ? std::filesystem::path(ros_home) :
      std::filesystem::path(std::getenv("HOME")) / ".ros";
    saved_dir_ = declare_parameter("saved_map_directory", (state_dir / "vacuum_maps").string());
    save_service_ = create_service<SaveMap>("/vacuum/floor/save_map",
      [this](SaveMap::Request::SharedPtr req, SaveMap::Response::SharedPtr res) { save_map(*req, *res); });
    declare_parameter("initial_x", 0.0);
    declare_parameter("initial_y", 0.0);
    declare_parameter("initial_yaw", 0.0);

    // A planner starting after floor selection still needs the current configuration.
    rclcpp::QoS floor_config_qos(rclcpp::KeepLast(1));
    floor_config_qos.transient_local();
    floor_config_publisher_ = create_publisher<FloorConfig>("/vacuum/floor/config", floor_config_qos);

    set_floor_service_ = create_service<SetFloorLevel>(
      "/vacuum/set_floor_level",
      [this](const SetFloorLevel::Request::SharedPtr request,
        SetFloorLevel::Response::SharedPtr response) {
        select_floor(request->floor_id, *response);
      });
  }

private:
  void select_floor(const std::string & id, SetFloorLevel::Response & response)
  {
    if (id != "Floor_1" && id != "Floor_2") {
      response.message = "Unknown floor '" + id + "'. Choose Floor_1 or Floor_2.";
      response.success = false;
      return;
    }
    try {
      const auto file = YAML::LoadFile(floor_dir_ + id + ".yaml");
      FloorConfig config;
      config.floor_id = id;
      config.map_state = file["map_state"].as<std::string>();
      config.grid = read_grid(file);
      const auto saved = saved_dir_ / (id + ".yaml");
      if (std::filesystem::exists(saved)) {
        const auto snapshot = YAML::LoadFile(saved.string());
        auto grid = read_grid(snapshot);
        if (grid.info.width != config.grid.info.width || grid.info.height != config.grid.info.height ||
          grid.info.resolution != config.grid.info.resolution)
        {
          throw std::runtime_error("Saved map does not match floor dimensions");
        }
        config.map_state = snapshot["map_state"].as<std::string>();
        config.grid = std::move(grid);
      }
      const auto initial = read_pose(file["initial_pose"], config.grid);
      config.initial_pose = initial.pose;
      config.dock_pose = read_pose(file["dock_pose"], config.grid).pose;
      config.grid.header.stamp = now();
      config.grid.info.map_load_time = config.grid.header.stamp;

      const auto update = set_parameters_atomically({
        rclcpp::Parameter("initial_x", initial.pose.position.x),
        rclcpp::Parameter("initial_y", initial.pose.position.y),
        rclcpp::Parameter("initial_yaw", initial.yaw_radians)});
      if (!update.successful) {
        throw std::runtime_error(update.reason);
      }
      selected_ = config;
      floor_config_publisher_->publish(config);
      response.success = true;
      response.message = "Loaded " + id + ", " + std::to_string(config.grid.info.width) +
        "x" + std::to_string(config.grid.info.height) + " grid (" + config.map_state +
        "), initial x=" + std::to_string(initial.pose.position.x) +
        " m, y=" + std::to_string(initial.pose.position.y) +
        " m, yaw=" + std::to_string(initial.yaw_radians) + " rad.";
      RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
    } catch (const std::exception & error) {
      response.success = false;
      response.message = "Could not load " + id + ": " + error.what();
      RCLCPP_WARN(get_logger(), "%s", response.message.c_str());
    }
  }

  void save_map(const SaveMap::Request & request, SaveMap::Response & response)
  {
    const auto & grid = request.grid;
    if (!selected_ || selected_->floor_id != request.floor_id ||
      selected_->grid.info.map_load_time != grid.info.map_load_time ||
      selected_->grid.info.width != grid.info.width || selected_->grid.info.height != grid.info.height ||
      selected_->grid.info.resolution != grid.info.resolution || selected_->grid.data.size() != grid.data.size() ||
      (request.map_state != "partial" && request.map_state != "complete"))
    {
      response.message = "Snapshot does not match the selected floor";
      return;
    }
    try {
      YAML::Node snapshot;
      snapshot["map_state"] = request.map_state;
      snapshot["resolution"] = grid.info.resolution;
      for (unsigned y = 0; y < grid.info.height; ++y) {
        std::string row;
        for (unsigned x = 0; x < grid.info.width; ++x) {
          switch (grid.data[y * grid.info.width + x]) {
            case -1: row += '?'; break;
            case 0: row += '.'; break;
            case 100: row += '#'; break;
            default: throw std::runtime_error("Invalid occupancy value");
          }
        }
        snapshot["grid"].push_back(row);
      }
      std::filesystem::create_directories(saved_dir_);
      const auto destination = saved_dir_ / (request.floor_id + ".yaml");
      const auto temporary = destination.string() + ".tmp";
      std::ofstream output;
      output.exceptions(std::ios::badbit | std::ios::failbit);
      output.open(temporary);
      output << snapshot << '\n';
      output.close();
      std::filesystem::rename(temporary, destination);
      selected_->grid = grid;
      selected_->map_state = request.map_state;
      response.success = true;
      response.message = "Saved " + request.map_state + " map for " + request.floor_id;
    } catch (const std::exception & error) {
      response.message = error.what();
    }
  }

  std::optional<FloorConfig> selected_;
  std::filesystem::path saved_dir_;
  rclcpp::Service<SaveMap>::SharedPtr save_service_;
  std::string floor_dir_;
  rclcpp::Publisher<FloorConfig>::SharedPtr floor_config_publisher_;
  rclcpp::Service<SetFloorLevel>::SharedPtr set_floor_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapLoader>());
  rclcpp::shutdown();
  return 0;
}
