cmake_minimum_required(VERSION 2.8.3)
project(airp1_controller)

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
  include/airp1_controller
)

catkin_package( INCLUDE_DIRS src 
	LIBRARIES airp1_controller  
	CATKIN_DEPENDS pluginlib roscpp
  DEPENDS Boost
)

add_library( airp1_controller_lib src/airp1_controller.cpp )
target_link_libraries( airp1_controller_lib ${catkin_LIBRARIES} )

