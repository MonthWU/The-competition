cmake_minimum_required(VERSION 3.8)
project(puzzle_perception_node)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(puzzle_geometry REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio calib3d)

add_executable(puzzle_perception_node
  src/main.cpp
  src/puzzle_perception_node.cpp
)
target_include_directories(puzzle_perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(puzzle_perception_node
  rclcpp std_msgs sensor_msgs geometry_msgs vision_interfaces puzzle_geometry
)
target_link_libraries(puzzle_perception_node ${OpenCV_LIBS})

if(BUILD_TESTING)
  enable_testing()
  function(add_perception_test test_name source_file)
    set(target_name "test_${test_name}")
    add_executable(${target_name} ${source_file})
    target_include_directories(${target_name} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(${target_name} ${OpenCV_LIBS})
    add_test(NAME ${test_name} COMMAND ${target_name})
  endfunction()

  add_perception_test(green_a4_detector test/test_green_a4_detector.cpp)
  add_perception_test(hsv_object_mask test/test_hsv_object_mask.cpp)
  add_perception_test(hmi_exposure test/test_hmi_exposure.cpp)
  add_perception_test(piece_foreground_mask test/test_piece_foreground_mask.cpp)
  add_perception_test(lab_background_mask test/test_lab_background_mask.cpp)
  add_perception_test(polygon_refinement test/test_polygon_refinement.cpp)
endif()

install(TARGETS puzzle_perception_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()
