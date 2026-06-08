cmake_minimum_required(VERSION 3.8)
project(supervisor_remake)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(executor_interfaces REQUIRED)
find_package(hri_tools REQUIRED)
find_package(talker_interfaces REQUIRED)
find_package(known_locations_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(supervisor_node
  src/SupervisorNode.cpp
  src/supervisor_main.cpp
)

target_include_directories(supervisor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(supervisor_node
  rclcpp
  rclcpp_action
  executor_interfaces
  hri_tools
  talker_interfaces
  known_locations_interfaces
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  yaml_cpp_vendor
)

install(TARGETS supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch plans
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
