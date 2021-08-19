cmake_minimum_required(VERSION 3.5)
project(tree_detection_ros)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  tree_detection
  ground_plane_removal
  pcl_conversions
  pcl_ros
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  roslib
  jsk_recognition_msgs
  message_generation
)

set (SRC_FILES
  src/TreeDetectorRos.cpp
  src/helpers.cpp
  src/creators.cpp
)


find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

add_service_files(
  DIRECTORY
    srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    yaml-cpp
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_runtime
  DEPENDS
    PCL
)


###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

link_directories( #only needed for pcl
  ${PCL_LIBRARY_DIRS}
)

add_definitions( #only needed for pcl
  ${PCL_DEFINITIONS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
)

add_executable(tree_detection_node
  src/tree_detection_node.cpp
)

target_link_libraries(tree_detection_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



#add_executable(process_pcd_node
#  src/process_pcd_node.cpp
#)
#
#target_link_libraries(process_pcd_node
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}
#)

#############
## Install ##
#############
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE
#)
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node point_cloud_stitcher_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY
#  launch
#  param
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
