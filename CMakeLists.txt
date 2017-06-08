cmake_minimum_required(VERSION 2.8.3)
project(pcl_tests)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  sensor_msgs
  tf
)

#find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)



###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_tests
  CATKIN_DEPENDS
	geometry_msgs
	roscpp
	sensor_msgs
	eigen_conversions
  DEPENDS
	system_lib
#	Eigen 
	PCL 
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


#add_library(camtfcal src/camtfcal.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/pcl_tests_node.cpp)


#target_link_libraries(camtfcal ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
#  camtfcal
)

