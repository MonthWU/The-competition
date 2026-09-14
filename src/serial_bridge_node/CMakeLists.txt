cmake_minimum_required(VERSION 3.8)
project(serial_bridge_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)

add_executable(serial_bridge_node
  src/serial_bridge_node.cpp
)

target_include_directories(serial_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(serial_bridge_node
  rclcpp
  std_msgs
  vision_interfaces
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_vision_frame_formatter test/test_vision_frame_formatter.cpp)
  target_include_directories(test_vision_frame_formatter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(NAME vision_frame_formatter COMMAND test_vision_frame_formatter)
endif()

install(
  TARGETS serial_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
