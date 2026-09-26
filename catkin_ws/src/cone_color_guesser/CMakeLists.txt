cmake_minimum_required(VERSION 3.0.2)
project(cone_color_guesser)

# ---- C++ 标准 ----
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---- Apple Silicon 特别处理 ----
# 显式指定 arm64 架构，避免混用 x86 的库导致链接错误
if(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
  set(CMAKE_OSX_ARCHITECTURES "arm64")
endif()

# ---- 找依赖 ----
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  tf2_ros
  tf2_sensor_msgs
)

find_package(PCL REQUIRED COMPONENTS common filters segmentation search io)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions
                 visualization_msgs tf2_ros
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

# ---- 编译可执行文件 ----
add_executable(cone_color_guesser_node
  src/cone_color_guesser_node.cpp
)

target_link_libraries(cone_color_guesser_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)