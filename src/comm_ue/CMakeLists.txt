cmake_minimum_required(VERSION 3.8)
project(comm_ue)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Specify the OpenCV version directory
set(OpenCV_DIR /opt/ros/jazzy/share/OpenCV-4.0.6)  # Adjust this path according to your OpenCV installation

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interfaces REQUIRED) 
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
  )

# Add executables
add_executable(joints_command src/ros_ue_connector_joints_command.cpp)
add_executable(image src/ros_ue_connector_image.cpp)
add_executable(calibration src/ue_cam_calibration.cpp)
add_executable(aruco_detector src/ue_aruco_detector.cpp)

# Link dependencies using ament_target_dependencies
ament_target_dependencies(joints_command rclcpp std_msgs)
ament_target_dependencies(image rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(calibration rclcpp std_msgs sensor_msgs cv_bridge OpenCV)
ament_target_dependencies(aruco_detector rclcpp sensor_msgs cv_bridge OpenCV tf2_ros geometry_msgs interfaces 
Eigen3 yaml-cpp)

# Link libraries
target_link_libraries(joints_command rt ${OpenCV_LIBS}) # Real-time library (required for shm)
target_link_libraries(image rt ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}) # Real-time library (required for shm)
target_link_libraries(calibration rt ${OpenCV_LIBS} ${cv_bridge_LIBRARIES}) # Real-time library (required for calibration)
target_link_libraries(aruco_detector rt ${OpenCV_LIBS} ${cv_bridge_LIBRARIES} ${tf2_ros_LIBRARIES} ${YAML_CPP_LIBRARIES}) # Real-time library (required for aruco detection)

# Explicitly specify the OpenCV modules to link
target_link_libraries(aruco_detector ${OpenCV_LIBS} opencv_aruco)

# Install targets
install(TARGETS 
  joints_command
  image
  calibration
  aruco_detector
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter which checks for copyrights
  set(ament_cmake_cpplint_FOUND TRUE)
  # Skip cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
