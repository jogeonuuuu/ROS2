cmake_minimum_required(VERSION 3.16)
project(dxl)

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
find_package(geometry_msgs REQUIRED)

include_directories(include)
include_directories(/usr/local/include/dynamixel_sdk)

# Build
add_executable(pub_manual src/pub_manual.cpp src/dxl.cpp)
ament_target_dependencies(pub_manual rclcpp geometry_msgs)
target_link_libraries(pub_manual dxl_x64_cpp)

add_executable(sub_manual src/sub_manual.cpp src/dxl.cpp)
ament_target_dependencies(sub_manual rclcpp geometry_msgs)
target_link_libraries(sub_manual dxl_x64_cpp)

add_executable(sub_auto src/sub_auto.cpp src/dxl.cpp)
ament_target_dependencies(sub_auto rclcpp std_msgs)
target_link_libraries(sub_auto dxl_x64_cpp)

install(TARGETS
  pub_manual
  sub_manual
  sub_auto
  DESTINATION lib/${PROJECT_NAME})

ament_package()
