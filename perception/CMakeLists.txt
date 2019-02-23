cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  rosbag
  sensor_msgs
  tf
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    perception_crop
    perception_downsample
    perception_segmentation
#  LIBRARIES robot_api
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(perception_save_cloud_main src/save_cloud_main.cpp)
set_target_properties(perception_save_cloud_main PROPERTIES OUTPUT_NAME save_cloud PREFIX "")
add_dependencies(perception_save_cloud_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_save_cloud_main
   ${catkin_LIBRARIES}
)

add_library(perception_crop src/crop.cpp)
add_dependencies(perception_crop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_crop ${catkin_LIBRARIES})

add_library(perception_downsample src/downsample.cpp)
add_dependencies(perception_downsample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_downsample ${catkin_LIBRARIES})

add_library(perception_segmentation src/segmentation.cpp)
add_dependencies(perception_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_segmentation ${catkin_LIBRARIES})

add_executable(perception_point_cloud_demo src/point_cloud_demo.cpp)
set_target_properties(perception_point_cloud_demo PROPERTIES OUTPUT_NAME point_cloud_demo PREFIX "")
add_dependencies(perception_point_cloud_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_point_cloud_demo
  perception_crop
  perception_downsample
  perception_segmentation
  ${catkin_LIBRARIES}
)
