#include "autonomous_vacuum_nodes/floor_markers.hpp"

#include <cstddef>

using Marker = visualization_msgs::msg::Marker;

namespace
{
geometry_msgs::msg::Point point(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}
}  // namespace

visualization_msgs::msg::MarkerArray make_floor_markers(
  const autonomous_vacuum_interfaces::msg::FloorConfig & floor,
  const std::vector<bool> & cleaned)
{
  visualization_msgs::msg::MarkerArray scene;
  const double cell_size = floor.grid.info.resolution;

  // Use flat colored cells for the map. This avoids the indexed-texture shader
  // which fails on the graphics driver in the course Distrobox.
  Marker map;
  map.header.frame_id = "map";
  map.ns = "occupancy_grid";
  map.type = Marker::CUBE_LIST;
  map.pose.orientation.w = 1.0;
  map.scale.x = map.scale.y = cell_size;
  map.scale.z = 0.004;
  map.color.a = 1.0F;
  const auto columns = floor.grid.info.width;
  for (std::size_t i = 0; i < floor.grid.data.size(); ++i) {
    map.points.push_back(point((i % columns + 0.5) * cell_size, (i / columns + 0.5) * cell_size, -0.005));
    const auto occupancy = floor.grid.data[i];
    std_msgs::msg::ColorRGBA color;
    float shade = 0.49F;  // Unknown
    if (occupancy == 0) { shade = 0.94F; }  // Free
    if (occupancy == 100) { shade = 0.08F; }  // Wall
    color.r = color.g = color.b = shade;
    color.a = 1.0F;
    map.colors.push_back(color);
  }
  scene.markers.push_back(map);

  Marker tiles;
  tiles.header.frame_id = "map";
  tiles.ns = "cleaned_cells";
  tiles.type = Marker::CUBE_LIST;
  tiles.pose.orientation.w = 1.0;
  tiles.scale.x = tiles.scale.y = 0.94 * cell_size;
  tiles.scale.z = 0.008;
  tiles.color.r = 0.13F;
  tiles.color.g = 0.76F;
  tiles.color.b = 0.42F;
  tiles.color.a = 0.78F;
  for (std::size_t i = 0; i < cleaned.size(); ++i) {
    if (cleaned[i]) {
      tiles.points.push_back(point((i % columns + 0.5) * cell_size, (i / columns + 0.5) * cell_size, 0.015));
    }
  }
  // Remove the previous session's overlay when the new mask is empty.
  tiles.action = tiles.points.empty() ? Marker::DELETE : Marker::ADD;
  scene.markers.push_back(tiles);

  Marker dock;
  dock.header.frame_id = "map";
  dock.ns = "dock";
  dock.type = Marker::LINE_STRIP;
  dock.pose = floor.dock_pose;
  dock.pose.position.z = 0.045;
  dock.scale.x = 0.035;
  dock.color.r = 1.0F;
  dock.color.g = 0.55F;
  dock.color.b = 0.08F;
  dock.color.a = 1.0F;
  const double half = 0.46 * cell_size;
  dock.points = {point(half, half), point(-half, half), point(-half, -half), point(half, -half)};
  scene.markers.push_back(dock);

  // RViz follows base_link through TF between marker publications.
  Marker body;
  body.header.frame_id = "base_link";
  body.frame_locked = true;
  body.ns = "robot_body";
  body.type = Marker::CYLINDER;
  body.pose.orientation.w = 1.0;
  body.pose.position.z = 0.08;
  body.scale.x = body.scale.y = 0.72 * cell_size;
  body.scale.z = 0.06;
  body.color.r = 0.12F;
  body.color.g = 0.22F;
  body.color.b = 0.55F;
  body.color.a = 1.0F;
  scene.markers.push_back(body);

  Marker heading;
  heading.header = body.header;
  heading.frame_locked = true;
  heading.ns = "robot_heading";
  heading.type = Marker::ARROW;
  heading.pose.orientation.w = 1.0;
  heading.points = {point(-0.19 * cell_size, 0, 0.13), point(0.28 * cell_size, 0, 0.13)};
  heading.scale.x = 0.035;
  heading.scale.y = 0.10;
  heading.scale.z = 0.09;
  heading.color.r = heading.color.g = heading.color.b = heading.color.a = 1.0F;
  scene.markers.push_back(heading);
  return scene;
}
