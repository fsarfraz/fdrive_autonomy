cmake_minimum_required(VERSION 3.8)
project(sdrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(comms_bridge)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDBUS REQUIRED sdbus-c++)

add_executable(sdrive_odometry src/sdrive_odometry.cpp)
ament_target_dependencies(sdrive_odometry rclcpp std_msgs comms_bridge rclcpp_lifecycle lifecycle_msgs nav_msgs tf2 tf2_ros)
target_include_directories(sdrive_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sdrive_odometry PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(sdrive_controller src/sdrive_controller.cpp)
ament_target_dependencies(sdrive_controller rclcpp std_msgs comms_bridge rclcpp_lifecycle lifecycle_msgs nav_msgs tf2 tf2_ros)
target_include_directories(sdrive_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sdrive_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(vesc_socketcan_node src/vesc_socketcan_node.cpp)
target_include_directories(vesc_socketcan_node PUBLIC
  $<INSTALL_INTERFACE:include>
  ${SDBUS_INCLUDE_DIRS}
)
ament_target_dependencies(vesc_socketcan_node rclcpp std_msgs comms_bridge)
target_link_libraries(vesc_socketcan_node ${SDBUS_LIBRARIES})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

install(TARGETS sdrive_odometry sdrive_controller vesc_socketcan_node
  DESTINATION lib/${PROJECT_NAME})

# --- Path Planning Nodes ---
add_executable(gridmap_to_costmap src/gridmap_to_costmap_node.cpp)
ament_target_dependencies(gridmap_to_costmap rclcpp grid_map_ros grid_map_msgs nav_msgs)
target_compile_features(gridmap_to_costmap PUBLIC cxx_std_17)

add_executable(global_costmap src/global_costmap_node.cpp)
ament_target_dependencies(global_costmap rclcpp nav_msgs tf2_ros tf2_geometry_msgs geometry_msgs)
target_compile_features(global_costmap PUBLIC cxx_std_17)

add_executable(simple_planner src/simple_planner_node.cpp)
ament_target_dependencies(simple_planner rclcpp nav_msgs geometry_msgs tf2_ros)
target_compile_features(simple_planner PUBLIC cxx_std_17)

add_executable(path_follower src/path_follower_node.cpp)
ament_target_dependencies(path_follower rclcpp nav_msgs geometry_msgs tf2_ros tf2)
target_compile_features(path_follower PUBLIC cxx_std_17)

install(TARGETS gridmap_to_costmap global_costmap simple_planner path_follower
  DESTINATION lib/${PROJECT_NAME})

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
