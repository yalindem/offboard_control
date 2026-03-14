cmake_minimum_required(VERSION 3.5)
project(offboard_control)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

############################################################
# Header-only libraries
############################################################

add_library(KalmanFilter INTERFACE)

target_include_directories(KalmanFilter INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(KalmanFilter INTERFACE Eigen3::Eigen)


add_library(FastMedianFilter INTERFACE)

target_include_directories(FastMedianFilter INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

############################################################
# height_estimator library
############################################################

add_library(height_estimator SHARED
  src/height_estimator.cpp
)

target_include_directories(height_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(height_estimator
  KalmanFilter
)

############################################################
# Executables
############################################################

add_executable(offboard_control_node
  src/offboard_control_node.cpp
)

target_link_libraries(offboard_control_node
  height_estimator
  FastMedianFilter
)

ament_target_dependencies(offboard_control_node
  rclcpp
  px4_msgs
)

add_executable(takeoff_node
  src/takeoff_node.cpp
)

ament_target_dependencies(takeoff_node
  rclcpp
  px4_msgs
)

############################################################
# Install
############################################################

install(TARGETS
  offboard_control_node
  takeoff_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/start_px4.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()