cmake_minimum_required(VERSION 3.5)
project(offboard_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)


include_directories(
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)


add_executable(offboard_control_node src/offboard_control_node.cpp)
add_executable(sensor_combined_listener_node src/sensor_combined_listener.cpp)
add_executable(edge_detector_node src/edge_detector_node.cpp)
add_executable(range_detector_node src/range_detector_node.cpp)


install(TARGETS
  range_detector_node
  DESTINATION lib/${PROJECT_NAME})


ament_target_dependencies(edge_detector_node rclcpp sensor_msgs OpenCV cv_bridge image_transport)

ament_target_dependencies(offboard_control_node
  rclcpp
  px4_msgs
)

ament_target_dependencies(sensor_combined_listener_node
  rclcpp
  px4_msgs
)

ament_target_dependencies(range_detector_node 
  rclcpp 
  sensor_msgs
  pcl_conversions
)

target_link_libraries(range_detector_node ${PCL_LIBRARIES})


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
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

install(TARGETS
  edge_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  range_detector_node 
  DESTINATION lib/${PROJECT_NAME})

ament_package()
