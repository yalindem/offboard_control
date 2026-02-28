cmake_minimum_required(VERSION 3.5)
project(offboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(height_estimator SHARED src/height_estimator.cpp)
add_library(FastMedianFilter INTERFACE)

include_directories(
  include
)

target_include_directories(FastMedianFilter INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(offboard_control_node src/offboard_control_node.cpp)
add_executable(target_localizer_node src/target_localizer_node.cpp)
add_executable(sensor_combined_listener_node src/sensor_combined_listener.cpp)
add_executable(advertiser_node src/advertiser.cpp)
add_executable(takeoff_node src/takeoff_node.cpp)

target_link_libraries(offboard_control_node
  FastMedianFilter
  height_estimator
)

ament_target_dependencies(offboard_control_node
  rclcpp
  px4_msgs
)

ament_target_dependencies(takeoff_node
  rclcpp
  px4_msgs
)

ament_target_dependencies(target_localizer_node
  rclcpp
  tf2_geometry_msgs
  tf2
  tf2_ros
  px4_msgs
)

ament_target_dependencies(sensor_combined_listener_node
  rclcpp
  px4_msgs
)

ament_target_dependencies(advertiser_node
  rclcpp
  px4_msgs
)



install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
)


install(PROGRAMS
  scripts/start_px4.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  target_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  offboard_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  takeoff_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  sensor_combined_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  advertiser_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
