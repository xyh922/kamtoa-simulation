cmake_minimum_required(VERSION 2.8.3)
project(som_o_mavlink)


set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  libmavconn
  mavros
  mavlink
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
 catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES som_o_mavlink
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${mavlink_INCLUDE_DIRS}
)
