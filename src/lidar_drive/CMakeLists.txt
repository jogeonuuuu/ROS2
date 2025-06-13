cmake_minimum_required(VERSION 3.16)
project(lidar_drive)
 
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
 
include_directories(include)
 
# build
add_executable(lidar_drive src/lidar_drive.cpp src/main.cpp)
ament_target_dependencies(lidar_drive rclcpp std_msgs sensor_msgs OpenCV)
 
# Install
install(TARGETS
  lidar_drive
  DESTINATION lib/${PROJECT_NAME})
 
ament_package()
