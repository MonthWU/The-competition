cmake_minimum_required(VERSION 3.8)
project(puzzle_geometry)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_planar_geometry test/test_planar_geometry.cpp)
  target_include_directories(test_planar_geometry PRIVATE include)
  add_test(NAME planar_geometry COMMAND test_planar_geometry)
endif()

ament_package()
