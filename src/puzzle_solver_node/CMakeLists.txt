cmake_minimum_required(VERSION 3.8)
project(puzzle_solver_node)

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
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Threads REQUIRED)

add_library(puzzle_solver_core src/puzzle_solver_core.cpp)
target_include_directories(puzzle_solver_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(puzzle_solver_core PRIVATE -O3)
endif()
target_link_libraries(puzzle_solver_core ${OpenCV_LIBS} Threads::Threads)

add_executable(puzzle_solver_node
  src/main.cpp
  src/puzzle_solver_node.cpp
)
target_include_directories(puzzle_solver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(puzzle_solver_node
  rclcpp std_msgs sensor_msgs geometry_msgs vision_interfaces puzzle_geometry
)
target_link_libraries(puzzle_solver_node puzzle_solver_core ${OpenCV_LIBS})

install(TARGETS puzzle_solver_core puzzle_solver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_puzzle_solver_core test/test_puzzle_solver_core.cpp)
  set_tests_properties(test_puzzle_solver_core PROPERTIES TIMEOUT 180)
  target_link_libraries(test_puzzle_solver_core puzzle_solver_core ${OpenCV_LIBS})
  target_include_directories(test_puzzle_solver_core PRIVATE include)
endif()

ament_package()
