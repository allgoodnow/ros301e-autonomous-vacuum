cmake_minimum_required(VERSION 3.8)
project(autonomous_vacuum_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(autonomous_vacuum_interfaces REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(hardware_driver src/hardware_driver.cpp src/grid_world.cpp)
target_compile_features(hardware_driver PRIVATE cxx_std_17)
target_include_directories(hardware_driver PRIVATE include)
ament_target_dependencies(hardware_driver
  rclcpp autonomous_vacuum_interfaces ament_index_cpp geometry_msgs nav_msgs sensor_msgs tf2_ros)
target_link_libraries(hardware_driver yaml-cpp::yaml-cpp)

add_executable(map_loader src/map_loader.cpp)
target_compile_features(map_loader PRIVATE cxx_std_17)
ament_target_dependencies(map_loader rclcpp autonomous_vacuum_interfaces ament_index_cpp geometry_msgs nav_msgs)
target_link_libraries(map_loader yaml-cpp::yaml-cpp)

foreach(node coverage_planner motion_controller)
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PRIVATE cxx_std_17)
  ament_target_dependencies(${node} rclcpp autonomous_vacuum_interfaces geometry_msgs nav_msgs)
endforeach()
target_sources(coverage_planner PRIVATE src/floor_markers.cpp)
target_include_directories(coverage_planner PRIVATE include)
ament_target_dependencies(coverage_planner visualization_msgs)

add_executable(slam_node src/slam_node.cpp)
target_compile_features(slam_node PRIVATE cxx_std_17)
ament_target_dependencies(slam_node rclcpp autonomous_vacuum_interfaces geometry_msgs nav_msgs sensor_msgs tf2_ros)

add_executable(task_manager src/task_manager.cpp src/docking.cpp)
target_include_directories(task_manager PRIVATE include)
target_compile_features(task_manager PRIVATE cxx_std_17)
ament_target_dependencies(task_manager rclcpp rclcpp_action autonomous_vacuum_interfaces nav_msgs)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(TARGETS slam_node task_manager hardware_driver map_loader coverage_planner motion_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
