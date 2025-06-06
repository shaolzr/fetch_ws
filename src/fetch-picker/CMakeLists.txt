cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs visualization_msgs pcl_ros pcl_conversions
)

add_library(perception_segmentation
  src/segmentation.cpp
)

target_link_libraries(perception_segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(point_cloud_demo src/point_cloud_demo.cpp)
target_link_libraries(point_cloud_demo
  perception_segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(point_cloud_demo1 src/point_cloud_demo1.cpp)
target_link_libraries(point_cloud_demo1
  perception_segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS point_cloud_demo point_cloud_demo1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 