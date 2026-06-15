cmake_minimum_required(VERSION 3.8)
project(waypoint_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalPoint.msg"
  "srv/RecordPoints.srv"
  "srv/SelectGoal.srv"
  "srv/ListFloorIndex.srv"
  "srv/SelectPoint.srv"
  "srv/SetAxisControl.srv"
  "srv/SetFloorState.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(waypoint_manager_node src/waypoint_manager_node.cpp)
target_compile_features(waypoint_manager_node PUBLIC cxx_std_17)

ament_target_dependencies(
  waypoint_manager_node
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_dependencies(waypoint_manager_node ${PROJECT_NAME}__rosidl_generator_cpp)
target_include_directories(waypoint_manager_node PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(waypoint_manager_node "${cpp_typesupport_target}")

# waypoint_search_node: 模块化设计
# - 核心逻辑拆分到 include/waypoint_search/*.hpp
# - csv_loader.hpp: CSV 文件读取
# - path_planner.hpp: 路径规划算法
# - waypoint_tracker.hpp: 航点追踪器
# - visualization_helper.hpp: 可视化工具
# - waypoint_search_types.hpp: 基础数据结构
add_executable(waypoint_search_node
  src/waypoint_search_node.cpp
  src/goal_sequence_runner.cpp
)
target_compile_features(waypoint_search_node PUBLIC cxx_std_17)

ament_target_dependencies(
  waypoint_search_node
  rclcpp
  std_srvs
  nav_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_dependencies(waypoint_search_node ${PROJECT_NAME}__rosidl_generator_cpp)
target_include_directories(waypoint_search_node PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(waypoint_search_node "${cpp_typesupport_target}")
target_link_libraries(waypoint_search_node ${OpenCV_LIBS})

add_executable(waypoint_editor_node src/waypoint_editor_node.cpp)
target_compile_features(waypoint_editor_node PUBLIC cxx_std_17)

ament_target_dependencies(
  waypoint_editor_node
  rclcpp
  nav_msgs
  visualization_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

add_dependencies(waypoint_editor_node ${PROJECT_NAME}__rosidl_generator_cpp)
target_include_directories(waypoint_editor_node PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp"
)
target_link_libraries(waypoint_editor_node "${cpp_typesupport_target}")

install(TARGETS waypoint_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS waypoint_search_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS waypoint_editor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

