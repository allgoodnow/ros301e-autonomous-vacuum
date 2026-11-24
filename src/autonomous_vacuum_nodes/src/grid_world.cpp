#include "autonomous_vacuum_nodes/grid_world.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

GridWorld::GridWorld(const std::string & filename, double resolution, unsigned width, unsigned height)
: rows_(YAML::LoadFile(filename)["grid"].as<std::vector<std::string>>()), cell_size_(resolution)
{
  if (rows_.empty() || rows_.size() != height || width == 0 ||
    !std::isfinite(cell_size_) || cell_size_ <= 0.0)
  {
    throw std::runtime_error("Physical world dimensions do not match the selected floor");
  }
  for (const auto & row : rows_) {
    if (row.size() != width || row.find_first_not_of(".#") != std::string::npos) {
      throw std::runtime_error("Physical world must be rectangular and contain only . and #");
    }
  }
}

bool GridWorld::is_wall(int column, int row) const
{
  if (row < 0 || column < 0 || static_cast<unsigned>(row) >= rows_.size() ||
    static_cast<unsigned>(column) >= rows_.front().size())
  {
    return true;
  }
  return rows_[row][column] == '#';
}

bool GridWorld::can_place_robot(double x, double y) const
{
  // The square collision footprint fits inside one cell in a narrow corridor.
  const double half_width = 0.45 * cell_size_;
  const int left = std::floor((x - half_width) / cell_size_);
  const int right = std::floor((x + half_width) / cell_size_);
  const int bottom = std::floor((y - half_width) / cell_size_);
  const int top = std::floor((y + half_width) / cell_size_);
  for (int row = bottom; row <= top; ++row) {
    for (int column = left; column <= right; ++column) {
      if (is_wall(column, row)) {
        return false;
      }
    }
  }
  return true;
}

double GridWorld::range_to_wall(double x, double y, double angle, double max_range) const
{
  const double direction_x = std::cos(angle);
  const double direction_y = std::sin(angle);
  const double sample_spacing = cell_size_ / 64.0;
  for (double distance = 0.0; distance <= max_range; distance += sample_spacing) {
    const int column = std::floor((x + distance * direction_x) / cell_size_);
    const int row = std::floor((y + distance * direction_y) / cell_size_);
    if (is_wall(column, row)) {
      return distance;
    }
  }
  return std::numeric_limits<double>::infinity();
}
