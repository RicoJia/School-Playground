cmake_minimum_required(VERSION 3.0.2)
project(costmap_plugins)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES costmap_plugins
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

