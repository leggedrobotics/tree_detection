cmake_minimum_required(VERSION 3.5)
project(point_cloud_preprocessing)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



set(SRC_FILES
  src/PointCloudPreprocessor.cpp
  src/Parameters.cpp
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_REQUIRED_COMPONENTS}
)


find_package(PCL 1.10 REQUIRED)

find_package(OpenMP QUIET)
if (OpenMP_FOUND)
  add_compile_options("${OpenMP_CXX_FLAGS}")
  add_definitions(-DGROUND_PLANE_REMOVAL_OPENMP_FOUND=${OpenMP_FOUND})
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    yaml-cpp
    ${OpenMP_CXX_LIBRARIES}
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
    ${OpenMP_CXX_INCLUDE_DIRS}
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
  ${OpenMP_CXX_LIBRARIES}
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
