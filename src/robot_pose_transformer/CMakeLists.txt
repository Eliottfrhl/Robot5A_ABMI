cmake_minimum_required(VERSION 3.5)
project(robot_pose_transformer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  # Ajout de tf2_geometry_msgs
find_package(Eigen3 REQUIRED)
find_package(interfaces REQUIRED)         # Ajout de la dépendance interfaces
find_package(yaml-cpp REQUIRED)

add_executable(pose_transformer src/pose_transformer.cpp)

ament_target_dependencies(
  pose_transformer
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs  # Ajout de tf2_geometry_msgs
  Eigen3
  interfaces         # Ajout de la dépendance interfaces
  yaml-cpp
)

target_link_libraries(pose_transformer Eigen3::Eigen yaml-cpp)

install(TARGETS
  pose_transformer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
