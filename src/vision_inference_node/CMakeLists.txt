cmake_minimum_required(VERSION 3.18)

# JetPack installs nvcc outside the non-interactive ROS build PATH. Select the
# stable CUDA toolkit path before project() enables the CUDA language.
if(EXISTS "/usr/local/cuda/bin/nvcc" AND NOT DEFINED CMAKE_CUDA_COMPILER)
  set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc" CACHE FILEPATH "CUDA compiler")
endif()

project(vision_inference_node LANGUAGES CXX CUDA)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(CUDAToolkit REQUIRED)
find_package(Threads REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS /usr/include/aarch64-linux-gnu /usr/include
  REQUIRED
)
find_library(TENSORRT_LIBRARY nvinfer
  HINTS /usr/lib/aarch64-linux-gnu /usr/lib
  REQUIRED
)

add_executable(vision_inference_node
  src/main.cpp
  src/vision_inference_node.cpp
  src/camera_source.cpp
  src/inference_metrics.cpp
  src/tensorrt_detector.cpp
  src/yolo_postprocess.cpp
  src/cuda_preprocess.cu
)

target_include_directories(vision_inference_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TENSORRT_INCLUDE_DIR}
)

ament_target_dependencies(vision_inference_node
  rclcpp
  sensor_msgs
  std_msgs
  vision_interfaces
)

target_link_libraries(vision_inference_node
  ${OpenCV_LIBS}
  CUDA::cudart
  Threads::Threads
  ${TENSORRT_LIBRARY}
)

set_target_properties(vision_inference_node PROPERTIES
  CUDA_ARCHITECTURES 87
  CUDA_STANDARD 17
  CUDA_STANDARD_REQUIRED ON
  CUDA_SEPARABLE_COMPILATION ON
)

install(
  TARGETS vision_inference_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_yolo_postprocess
    test/test_yolo_postprocess.cpp
    src/yolo_postprocess.cpp
  )
  target_include_directories(test_yolo_postprocess PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_yolo_postprocess CUDA::cudart)
endif()

ament_package()
