cmake_minimum_required(VERSION 3.5)
project(offboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(offboard_control_node src/offboard_control_node.cpp)
add_executable(sensor_combined_listener_node src/sensor_combined_listener.cpp)

ament_target_dependencies(offboard_control_node
  rclcpp
  px4_msgs
)

ament_target_dependencies(sensor_combined_listener_node
  rclcpp
  px4_msgs
)

include_directories(
  include
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


install(PROGRAMS
  scripts/start_px4.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  offboard_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  sensor_combined_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
