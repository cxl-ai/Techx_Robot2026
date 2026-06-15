cmake_minimum_required(VERSION 3.8)
project(waypoint_editor_rviz_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)
find_package(waypoint_manager REQUIRED)

set(CMAKE_AUTOMOC ON)

include_directories(include)

set(HDR_FILES
  include/waypoint_editor_panel.hpp
)

foreach(header "${HDR_FILES}")
  qt5_wrap_cpp(rviz_plugins_moc_files "${header}")
endforeach()

set(SRC_FILES
  src/waypoint_editor_panel.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${HDR_FILES}
  ${rviz_plugins_moc_files}
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  rclcpp
  rviz_common
  waypoint_manager
  pluginlib
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${pluginlib}
  Qt5::Widgets
  Qt5::Core
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_dependencies(
  rclcpp
  rviz_common
  waypoint_manager
  pluginlib
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()

