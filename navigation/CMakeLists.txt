cmake_minimum_required(VERSION 3.8)
project(navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

# Add include directories
include_directories(include)

# Add library for the planner
add_library(StraightLinePlanner SHARED src/straight_line_planner.cpp)

# Link dependencies to the library
ament_target_dependencies(StraightLinePlanner
  rclcpp
  nav_msgs
  nav2_core
  pluginlib
  tf2_ros
  rclcpp_action
  rclcpp_lifecycle
  geometry_msgs
  builtin_interfaces
  nav2_costmap_2d
)

# # Export the plugin
# pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# Install targets
install(TARGETS StraightLinePlanner
  DESTINATION lib/${PROJECT_NAME}
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
