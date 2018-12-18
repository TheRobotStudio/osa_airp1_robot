cmake_minimum_required(VERSION 2.8.3)
project(r2p_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  pluginlib
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include/r2p_controller
)

catkin_package( INCLUDE_DIRS src 
	LIBRARIES r2p_controller  
	CATKIN_DEPENDS pluginlib roscpp
  DEPENDS Boost
)

add_library( r2p_controller_lib src/r2p_controller.cpp )
target_link_libraries( r2p_controller_lib ${catkin_LIBRARIES} )

