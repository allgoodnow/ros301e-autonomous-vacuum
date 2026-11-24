# Autonomous Home Vacuum

A C++17 ROS 2 Jazzy workspace for the ROS301E HW I vacuum simulation. The robot moves in a 2D grid, publishes live telemetry, maps unknown space, cleans reachable cells, and returns to its dock. Each grid cell is 0.5 m wide and has an area of 0.25 m².

## Build and launch

Use Ubuntu 24.04 with ROS 2 Jazzy Desktop, colcon, and the dependencies declared in the two package manifests. On the course machine, run the commands inside the ubuntu-24-04 Distrobox. From the workspace root (the directory containing src), run:

```bash
source /opt/ros/jazzy/setup.bash
# On a fresh setup with rosdep initialized:
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to autonomous_vacuum_nodes
source install/setup.bash
ros2 launch autonomous_vacuum_nodes vacuum.launch.xml rviz:=true
```

In each additional terminal, source Jazzy and the workspace again. Omit rviz:=true for a headless run. The RViz view uses the learned map. Gray cells are unknown, black cells are walls, and light cells are free. Green tiles are cleaned cells. The blue robot has a white heading arrow, the amber U marks the dock, and a blue line shows the current planned path.

## Reproduce the homework demonstrations

1. **V1 — System graph.** Start the launch file, run `rqt_graph`, and use the Nodes/Topics view. Inspect services with `ros2 service list -t` and actions with `ros2 action list -t`.
2. **V2 — Telemetry.** Run `rqt`, open Plugins → Topics → Topic Monitor, and expand `/vacuum/status`. In another terminal, run `ros2 topic hz /vacuum/status`. Telemetry is published at 1 Hz, including while idle. Battery percentage, bin occupancy (used cells and capacity), charging and bumper state come from the simulator. The 2D floors have no cliffs.
3. **V3 — Floor selection.** Select a floor before sending a goal. The response includes the loaded dimensions, map state, and initial pose. Floor_2 starts with known boundaries and an unknown interior on its first selection.

   ```bash
   ros2 service call /vacuum/set_floor_level autonomous_vacuum_interfaces/srv/SetFloorLevel "{floor_id: Floor_2}"
   ```

4. **V4 — Action feedback and cancellation.** For a repeatable cleaning run, select the fully known Floor_1. Let one goal complete. When docking finishes, repeat the goal and cancel while progress is between 0% and 100%.

   ```bash
   ros2 service call /vacuum/set_floor_level autonomous_vacuum_interfaces/srv/SetFloorLevel "{floor_id: Floor_1}"
   ros2 action send_goal /vacuum/run_task autonomous_vacuum_interfaces/action/VacuumTask "{mode: CLEAN_EXISTING_MAP, power_mode: STANDARD}" --feedback
   ```

   From a separate terminal during the second run:

   ```bash
   ros2 service call /vacuum/run_task/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
   ```

   Inspect the original goal terminal for the final status and partial statistics. Only one cleaning task is accepted at a time. In the reference run, Floor_1 finished at 22 cleaned cells (5.50 m²). Cancellation returns the area reached at the time of the stop.

## Mapping, cleaning, and docking

| Mode | Behavior |
|---|---|
| MAP_ONLY | Explore the frontier and update the occupancy grid. Cleaned area stays at 0. |
| CLEAN_EXISTING_MAP | Clean reachable known free cells on a complete or partial map. An unexplored floor is rejected. |
| SLAM_AND_CLEAN | Explore and clean, then finish any reachable cells left to clean. |

| Power mode | Cleaning footprint | Maximum speed |
|---|---|---|
| ECO | 1×1 | 2 cells/s |
| STANDARD | 2×2 | 2 cells/s |
| TURBO | 2×2 | 3 cells/s |

For a mapping run, select Floor_2 and send:

```bash
ros2 action send_goal /vacuum/run_task autonomous_vacuum_interfaces/action/VacuumTask "{mode: SLAM_AND_CLEAN, power_mode: TURBO}" --feedback
```

The simulator supplies ideal odometry and 72 range beams. The grid mapper pairs scans with odometry by timestamp, projects four directions near cell centers, and uses known walls to correct translation. Heading comes from odometry. A full Floor_2 mapping and cleaning run covers 35 free cells (8.75 m²). Completion and cancellation save the map before the result is returned. By default the next launch reads saved floor maps from `~/.ros/vacuum_maps/` when that floor is selected. To demonstrate a fresh floor while preserving existing saves, launch the system with a fresh ROS_HOME:

```bash
ROS_HOME="$(mktemp -d)" ros2 launch autonomous_vacuum_nodes vacuum.launch.xml rviz:=true
```

When work finishes or the user cancels during cleaning or mapping, the manager sends a separate ReturnToDock goal with the brush off. Canceling the cleaning goal during a return lets that return finish. Canceling it while charging leaves the robot at the dock. Canceling the docking action itself stops the return.

## Battery and dustbin

The simulation starts at 100% battery with 35 dustbin slots. Each cell uses one slot the first time the robot vacuums it during a task, including cells reached by the wider brush. Revisiting a cell, mapping with the brush off, and driving back to the dock add no dust. The last slot can fill partway through a 2×2 footprint. When the bin fills before coverage is complete, the task pauses and reports `RETURNING_DUSTBIN_FULL` in action feedback.

Energy is measured in battery percentage points. Driving costs 0.80 points per metre and steering costs 0.12 points per radian. Active cleaning adds 0.22 points/s in ECO, 0.38 in STANDARD, and 0.60 in TURBO. Mapping alone adds 0.04 points/s. Returning to the dock uses only the motion cost. The planner counts the free grid steps on the shortest known route home. The manager reserves energy for that route, turning and alignment, and checks whether another work step would leave enough margin. Low reserve triggers a service visit with feedback phase `RETURNING_LOW_BATTERY`. The original action stays active during the return and charging. At the dock, the bin empties and the battery charges at 20 percentage points/s up to 100%. Once hardware confirms a full battery and empty bin, the same task resumes with its original power mode, learned map and cleaned-cell mask. With `--feedback`, the phase changes to `CHARGING` at the dock and back to `WORKING` when the task resumes. Multiple visits are supported. Final area counts unique cells across the whole task. Elapsed time includes dock visits. Canceling the original action during a visit ends the task. A return already underway continues to the dock.

For a short demo, launch with **one** of the following commands, select Floor_1, and run the cleaning action shown above. Monitor `/vacuum/status` for physical measurements and `/vacuum/cleaning/coverage` for the estimated return reserve.

```bash
ros2 launch autonomous_vacuum_nodes vacuum.launch.xml dustbin_capacity_cells:=7
ros2 launch autonomous_vacuum_nodes vacuum.launch.xml initial_battery_percent:=14.0
```

## Source layout

`autonomous_vacuum_interfaces` contains the custom messages, services, and actions. `autonomous_vacuum_nodes` contains six executables:

| Node | Responsibility |
|---|---|
| hardware_driver | Simulates motion and collision, publishes odometry, laser scans, and vacuum telemetry. |
| map_loader | Loads floor configuration and persists complete or partial maps. |
| slam_node | Publishes the estimated pose and learned occupancy grid. |
| coverage_planner | Finds reachable frontiers and cleaning targets using BFS, tracks cleaned cells, and plans dock routes. |
| motion_controller | Follows waypoints and aligns the robot with the final dock pose. |
| task_manager | Handles goal validation, periodic feedback, cancellation, result statistics, and automatic docking. |
