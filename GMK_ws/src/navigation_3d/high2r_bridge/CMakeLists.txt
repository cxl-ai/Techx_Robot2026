cmake_minimum_required(VERSION 3.8)
project(high2r_bridge)

# Compile as C++17 for ROS2
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add subdirectory for DDS communication
add_subdirectory("dds_comm")

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  dds_comm/include
  dds_comm/ltr_m1/include
  dds_comm/thirdparty/include
  dds_comm/thirdparty/include/ddscxx
)

# Build executable
add_executable(${PROJECT_NAME}_node
  src/hig2robot_node.cpp
)

# Specify dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node 
  ltr_sdk
)

# Install executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Export package dependencies
ament_package()