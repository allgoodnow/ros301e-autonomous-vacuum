#pragma once

#include <cmath>
#include <string>

namespace energy
{
// Simulation units are battery percentage points, metres, radians, and seconds.
constexpr double drive_per_metre = 0.80;
constexpr double turn_per_radian = 0.12;

inline double motion_cost(double metres, double radians)
{
  return drive_per_metre * std::abs(metres) + turn_per_radian * std::abs(radians);
}

inline double work_rate(const std::string & mode, const std::string & power)
{
  if (mode == "RETURN_TO_DOCK") { return 0.0; }
  // Mapping and task electronics run with the brush off in MAP_ONLY.
  if (mode == "MAP_ONLY") { return 0.04; }
  if (power == "TURBO") { return 0.60; }
  if (power == "STANDARD") { return 0.38; }
  return 0.22;
}

inline double return_reserve(int steps, double cell_size)
{
  // Allow for re-centering, one half-turn per waypoint and final dock alignment.
  return motion_cost((steps + 1) * cell_size, (steps + 2) * std::acos(-1.0));
}

inline double next_step_budget(double cell_size, double work_per_second)
{
  // A step away also lengthens the return. Allow two seconds for telemetry and braking.
  return 2.0 * motion_cost(cell_size, std::acos(-1.0)) + 2.0 * work_per_second + 2.0;
}
}  // namespace energy
