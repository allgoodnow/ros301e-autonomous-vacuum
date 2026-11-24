#pragma once

#include <vector>

#include "autonomous_vacuum_interfaces/msg/floor_config.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// Build a complete snapshot so an RViz window opened mid-session shows every cleaned cell.
visualization_msgs::msg::MarkerArray make_floor_markers(
  const autonomous_vacuum_interfaces::msg::FloorConfig & floor,
  const std::vector<bool> & cleaned);
