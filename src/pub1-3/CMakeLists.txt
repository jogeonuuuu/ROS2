cmake_minimum_required(VERSION 3.16)
project(pub1-3) # 프로젝트 이름 (== 패키지명)
 
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14) #변수 설정 (변수 참조할 때는 '${변수명}' 형식으로 사용)
endif()
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
 
# find dependencies
#rclcpp을 사용하는 경우 ament_cmake와 rclcpp는 기본으로 포함시켜야하며, 
#geometry_msgs과 같은 인터페이스 패키지나 의존성 패키지를 기술
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
 
#build
add_executable(pub1-3 src/pub1-3.cpp) # 소스파일(pub1-3.cpp)로부터 만들어지는 실행파일(pub1-3)
ament_target_dependencies(pub1-3 rclcpp geometry_msgs)
# 타겟에 링크
target_link_libraries(pub1-3
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
)
# 변수를 사용하여 타겟에 링크
target_link_libraries(pub1-3
  ${rclcpp_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)
 
 
# Install  (<target>파일을 <dir> 경로에 설치)
install(TARGETS pub1-3             # 설치할 실행파일
  DESTINATION lib/${PROJECT_NAME}) # 파일을 설치할 경로 (${PROJECT_NAME} 변수는 이 프로젝트 이름을 참조)
 
ament_package()