cmake_minimum_required(VERSION 3.5)
project(navigation)

# Default to C99
set(CMAKE_C_STANDARD 99)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories for your package
include_directories(
  include
)

# Add executable target
add_executable(straight_line_planner_node src/straight_line_planner.cpp)
add_executable(astar_planner_node src/astar_planner.cpp)
add_executable(occupancy_grid src/occupancy_grid.cpp)
add_executable(waypoint_follower src/waypoint_follower.cpp)

# Link dependencies to the target
ament_target_dependencies(straight_line_planner_node
  rclcpp
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
)
ament_target_dependencies(astar_planner_node rclcpp nav2_costmap_2d geometry_msgs nav_msgs)
ament_target_dependencies(occupancy_grid rclcpp nav_msgs)
ament_target_dependencies(waypoint_follower rclcpp geometry_msgs nav_msgs)

# Install the executable
install(TARGETS straight_line_planner_node astar_planner_node occupancy_grid waypoint_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Include the plugin XML if you are using plugins (remove if not using)
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
