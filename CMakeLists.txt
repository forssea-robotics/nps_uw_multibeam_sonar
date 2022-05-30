cmake_minimum_required(VERSION 3.23)
project(nps_uw_multibeam_sonar LANGUAGES CUDA CXX)

## Make sure you have installed CUDA and include in paths
## LD_LIBRARY_PATH=LD_LIBRARY_PATH:/usr/local/cuda-11.1/lib64
## PATH=$PATH:/usr/local/cuda-11.1/bin
# set(CMAKE_CUDA_COMPILER "nvcc")
enable_language(CUDA)

include(CheckLanguage)
check_language(CUDA)

if(NOT DEFINED CMAKE_CUDA_STANDARD)
  set(CMAKE_CUDA_STANDARD 14)
  set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
  add_compile_options(-std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(velodyne_gazebo_plugins REQUIRED)
find_package(acoustic_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS}")

find_package(ament_lint_auto REQUIRED)

include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(SENSOR_ROS_PLUGINS_LIST "")

## Plugins
add_library(nps_multibeam_sonar_ros_plugin
            SHARED
            src/gazebo_multibeam_sonar_raster_based.cpp
            src/sonar_calculation_cuda.cu
            src/MaterialSwitcher.cc  # From gazebo/rendering/selection_buffer
            src/SelectionBuffer.cc  # From gazebo/rendering/selection_buffer
            src/SelectionRenderListener.cc  # From gazebo/rendering/selection_buffer
  )
target_include_directories(
  nps_multibeam_sonar_ros_plugin
    PUBLIC
    include
)
set_target_properties(nps_multibeam_sonar_ros_plugin
                      PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET nps_multibeam_sonar_ros_plugin PROPERTY CUDA_ARCHITECTURES all-major)
target_link_libraries(nps_multibeam_sonar_ros_plugin
                      ${OGRE_LIBRARIES} ${ament_LIBRARIES}
                      ${CUDA_LIBRARIES}
                      ${CUDA_CUFFT_LIBRARIES})
ament_target_dependencies(nps_multibeam_sonar_ros_plugin
  tf2
  cv_bridge
  gazebo_plugins
  acoustic_msgs
  xacro
  gazebo
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  PCL
  CUDA
)
list(APPEND SENSOR_ROS_PLUGINS_LIST nps_multibeam_sonar_ros_plugin)

add_library(nps_multibeam_sonar_ray_ros_plugin
            SHARED
            src/gazebo_multibeam_sonar_ray_based.cpp
            src/sonar_calculation_cuda.cu
            src/MaterialSwitcher.cc  # From gazebo/rendering/selection_buffer
            src/SelectionBuffer.cc  # From gazebo/rendering/selection_buffer
            src/SelectionRenderListener.cc  # From gazebo/rendering/selection_buffer
  )
target_include_directories(
  nps_multibeam_sonar_ray_ros_plugin
    PUBLIC
    include
)
set_target_properties(nps_multibeam_sonar_ray_ros_plugin
                      PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET nps_multibeam_sonar_ray_ros_plugin PROPERTY CUDA_ARCHITECTURES all-major)
target_link_libraries(nps_multibeam_sonar_ray_ros_plugin
                      ${OGRE_LIBRARIES} ${ament_LIBRARIES}
                      ${CUDA_LIBRARIES} ${PCL_LIBRARIES}
                      ${CUDA_CUFFT_LIBRARIES})
ament_target_dependencies(nps_multibeam_sonar_ray_ros_plugin
  tf2
  cv_bridge
  gazebo_plugins
  acoustic_msgs
  xacro
  gazebo
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  PCL
  CUDA
)
list(APPEND SENSOR_ROS_PLUGINS_LIST nps_multibeam_sonar_ray_ros_plugin)

ament_lint_auto_find_test_dependencies()

# Install plugins
install(
        DIRECTORY include/
        DESTINATION include
)
install(
  TARGETS ${SENSOR_ROS_PLUGINS_LIST}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# for launch
install(DIRECTORY launch config worlds urdf models
        DESTINATION share/${PROJECT_NAME}
        PATTERN "*~" EXCLUDE)

# EXPORTS
ament_export_include_directories(include)
ament_export_libraries(
    nps_multibeam_sonar_ros_plugin
    nps_multibeam_sonar_ray_ros_plugin
)
ament_export_dependencies(
  acoustic_msgs
)
ament_package()
