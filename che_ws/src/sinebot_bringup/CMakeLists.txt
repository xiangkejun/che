cmake_minimum_required(VERSION 2.8.3)
project(sinebot_bringup)

##################
## Dependencies ##
##################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries 
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    rospy 
    tf2
		tf2_ros
    nav_msgs 
    geometry_msgs 
    sensor_msgs 
    std_msgs
#    libmodbus
)

include(FindPkgConfig) 
pkg_check_modules(LIBMODBUS   libmodbus)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sinebot_bringup
  CATKIN_DEPENDS roscpp rospy tf2 tf2_ros nav_msgs geometry_msgs sensor_msgs std_msgs 
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
file(GLOB sinebot_SOURCES src/*.cpp)
add_library(sinebot_base ${sinebot_SOURCES} )
add_executable(sinebot_base_node src/main/sinebot_base_node.cpp)
target_link_libraries( sinebot_base_node  sinebot_base ${catkin_LIBRARIES} modbus)

#############
## Install ##
#############

# Future work

#############
## Testing ##
#############

# Future work
