cmake_minimum_required(VERSION 3.8)
project(comms_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bandwidth.msg"
  "msg/System.msg"
  "msg/Status.msg"
  "msg/Teleop.msg"
  "msg/Gimbal.msg"
  "msg/ThrottleBoard.msg"
  "msg/CurrentRel.msg"
  "msg/RpmRel.msg"
  "msg/EmergencyStop.msg"
  "msg/CanStatus1Msg.msg"
  "msg/CanStatus2Msg.msg"
  "msg/CanStatus3Msg.msg"
  "msg/CanStatus4Msg.msg"
  "msg/CanStatus5Msg.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
