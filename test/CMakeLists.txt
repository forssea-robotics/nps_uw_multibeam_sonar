cmake_minimum_required(VERSION 3.23)
project(nps_uw_multibeam_sonar LANGUAGES CUDA CXX)

## Make sure you have installed CUDA and include in paths
## LD_LIBRARY_PATH=LD_LIBRARY_PATH:/usr/local/cuda-11.1/lib64
## PATH=$PATH:/usr/local/cuda-11.1/bin
#set(CMAKE_CUDA_COMPILER "nvcc")
enable_language(CUDA)

include(CheckLanguage)
check_language(CUDA)

set(CMAKE_BUILD_TYPE Debug)

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
  add_compile_options(-std=c++17)
endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS}")

## Plugins
add_executable(${PROJECT_NAME} sonar_calculation_cuda.cu main.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
#set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_ARCHITECTURES all-major)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CUDA_LIBRARIES} ${CUDA_CUFFT_LIBRARIES})
