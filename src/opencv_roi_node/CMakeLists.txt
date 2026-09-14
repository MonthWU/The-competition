cmake_minimum_required(VERSION 3.8)
project(opencv_roi_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_executable(opencv_roi_node src/opencv_roi_node.cpp)
target_include_directories(opencv_roi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(opencv_roi_node rclcpp sensor_msgs std_msgs vision_interfaces)
target_link_libraries(opencv_roi_node ${OpenCV_LIBS})

install(TARGETS opencv_roi_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
