#pragma once

#include <string>
#include <vector>

// Physical world geometry. Only the hardware simulator reads this grid.
class GridWorld
{
public:
  GridWorld(const std::string & filename, double resolution, unsigned width, unsigned height);

  bool can_place_robot(double x, double y) const;
  double range_to_wall(double x, double y, double angle, double max_range) const;

private:
  bool is_wall(int column, int row) const;

  std::vector<std::string> rows_;
  double cell_size_;
};
