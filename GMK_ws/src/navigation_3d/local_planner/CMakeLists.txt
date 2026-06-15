cmake_minimum_required(VERSION 3.5)
project(local_planner)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Release 编译优化
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")

# 编译选项
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# local_planner_node 可执行文件
add_executable(localPlanner src/localPlanner.cpp)
ament_target_dependencies(localPlanner 
  rclcpp std_srvs std_msgs sensor_msgs nav_msgs geometry_msgs 
  tf2 tf2_ros tf2_geometry_msgs message_filters pcl_ros pcl_conversions)

# nav_node 可执行文件
add_executable(nav_node
  src/nav.cpp
  src/planning.cpp
  src/control.cpp
)
ament_target_dependencies(nav_node 
  rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs 
  tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions)
target_link_libraries(nav_node ${PCL_LIBRARIES})

install(TARGETS
  localPlanner
  nav_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  paths
  config
  include
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
