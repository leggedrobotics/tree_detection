cmake_minimum_required(VERSION 3.5)
project(tree_detection)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



set(SRC_FILES
  src/TreeDetector.cpp
  src/Parameters.cpp
#  src/common.cpp
)

find_package(catkin REQUIRED
)


find_package(PCL 1.10 REQUIRED)

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
