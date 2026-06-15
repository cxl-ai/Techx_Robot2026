cmake_minimum_required(VERSION 3.8)
project(pcd2pgm)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS2 依赖 ────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# ── PCL（点云加载与预处理滤波）───────────────────────────────────────────────
find_package(PCL REQUIRED COMPONENTS common io filters)

# ── octomap（三维体素化与二维占用投影）───────────────────────────────────────
find_package(octomap REQUIRED)

# ── 可执行文件 ────────────────────────────────────────────────────────────────
add_executable(pcd2pgm_node src/pcd2pgm_node.cpp)

ament_target_dependencies(pcd2pgm_node rclcpp)

target_link_libraries(pcd2pgm_node
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

target_include_directories(pcd2pgm_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_compile_definitions(pcd2pgm_node PRIVATE
  ${PCL_DEFINITIONS}
)

# ── 安装 ─────────────────────────────────────────────────────────────────────
install(TARGETS pcd2pgm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
