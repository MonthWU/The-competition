cmake_minimum_required(VERSION 3.8)
project(puzzle_coordinator_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)

add_executable(puzzle_coordinator_node
  src/main.cpp
  src/puzzle_coordinator_node.cpp
)
target_include_directories(puzzle_coordinator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(puzzle_coordinator_node rclcpp std_msgs vision_interfaces)

install(TARGETS puzzle_coordinator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
