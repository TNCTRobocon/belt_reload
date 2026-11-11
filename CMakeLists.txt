cmake_minimum_required(VERSION 3.8)
project(belt_reload)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sequence_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(belt_reload_node
  src/belt_reload_node.cpp
)

target_compile_features(belt_reload_node PUBLIC cxx_std_17)
ament_target_dependencies(belt_reload_node
  rclcpp
  rclcpp_action
  can_msgs
  sequence_interfaces
  std_msgs
)

install(
  TARGETS belt_reload_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
