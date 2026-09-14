cmake_minimum_required(VERSION 3.8)
project(kalman_filter_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core video)

add_executable(kalman_filter_node src/kalman_filter_node.cpp)
target_include_directories(kalman_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(kalman_filter_node rclcpp std_msgs vision_interfaces)
target_link_libraries(kalman_filter_node ${OpenCV_LIBS})

install(TARGETS kalman_filter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
