cmake_minimum_required(VERSION 3.8)
project(shredder_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/track_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME track_controller
)
add_executable(shredder_controller_node src/shredder_controller.cpp)

ament_target_dependencies(shredder_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_executable(shredder_traversability src/traversability_grid_map.cpp)

ament_target_dependencies(shredder_traversability
  rclcpp
  grid_map_ros
  grid_map_core
  grid_map_msgs
  builtin_interfaces
  rcutils
)

install(
  DIRECTORY params launch worlds urdf rviz models
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  shredder_controller_node
  shredder_traversability
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()