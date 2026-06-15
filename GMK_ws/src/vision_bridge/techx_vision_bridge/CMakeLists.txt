cmake_minimum_required(VERSION 3.8)
project(techx_vision_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target3D.msg"
  "msg/VisionTarget.msg"
  "msg/VisionObject.msg"
  "msg/VisionFrame.msg"
  "msg/VisionRequest.msg"
  "msg/VisionSelection.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(vision_bridge_node
  src/vision_frame_bridge_node.cpp
)

target_include_directories(vision_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(vision_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
)

rosidl_target_interfaces(vision_bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

find_package(Threads REQUIRED)
target_link_libraries(vision_bridge_node Threads::Threads)

install(TARGETS
  vision_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  tools/mock_jetson_sender.py
  tools/check_vision_bridge_config.py
  tools/estimate_extrinsic_from_points.py
  tools/vision_request_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
