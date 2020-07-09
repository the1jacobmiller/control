cmake_minimum_required(VERSION 2.8.3)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  message_generation
  pcl_ros
  dbw_mkz_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc
  CATKIN_DEPENDS roscpp rospy std_msgs dbw_mkz_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mpc.cpp
# )

add_executable(mpc_node src/main.cpp src/MPC.cpp src/visualization.cpp src/path.cpp src/helper_functions.cpp)
target_link_libraries(mpc_node ${catkin_LIBRARIES} ipopt)
add_dependencies(mpc_node ${catkin_EXPORTED_TARGETS})
