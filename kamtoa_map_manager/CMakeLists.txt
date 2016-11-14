cmake_minimum_required(VERSION 2.8.3)
project(kamtoa_map_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  roslib
  tf
  visualization_msgs
  message_generation
  rviz
)

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  poi.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  listPoi.srv
  loadPoi.srv
  resolvePoi.srv
  gotoPoi.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  kamtoa_map_manager std_msgs  geometry_msgs   move_base_msgs   nav_msgs   visualization_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include

#  LIBRARIES kamtoa_map_manager
   CATKIN_DEPENDS actionlib geometry_msgs move_base_msgs nav_msgs roscpp roslib tf visualization_msgs rviz message_runtime
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
)

## Declare a C++ library
# add_library(kamtoa_map_manager
#   src/${PROJECT_NAME}/kamtoa_map_manager.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kamtoa_map_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(kamtoa_map_manager_node src/kamtoa_map_manager_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(kamtoa_map_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(kamtoa_map_manager_node
#   ${catkin_LIBRARIES}
# )

add_executable(pointTransporter src/pointTransporter.cpp)
target_link_libraries(pointTransporter ${catkin_LIBRARIES})

# add_executable(mapSemantics src/mapSemantics.cpp)
# target_link_libraries(mapSemantics ${catkin_LIBRARIES})

add_executable(poiManager src/poiManager.cpp)
add_dependencies(poiManager kamtoa_map_manager_gencpp)
target_link_libraries(poiManager ${catkin_LIBRARIES})

# add_executable(gridtest src/map_accessor.cpp)
# target_link_libraries(gridtest ${catkin_LIBRARIES})

add_executable(localizer src/map_localizer.cpp)
target_link_libraries(localizer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(opencvtest src/opencvtest.cpp)
target_link_libraries(opencvtest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS kamtoa_map_manager kamtoa_map_manager_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kamtoa_map_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


################################################
## Declare QT's Library                       ##
################################################

## This plugin includes Qt widgets, so we must include Qt like so:
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

message(STATUS "@@@@@Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")

find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
qt5_wrap_cpp(QT_MOC include/kamtoa_map_manager/poi_manager_panel.hpp)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

set(SRC_FILES src/poi_manager_panel.cpp ${QT_MOC})

## Linking
add_library(poi_manager_panel ${SRC_FILES})
target_link_libraries(poi_manager_panel  ${QT_LIBRARIES} ${Qt5Widgets_LIBRARIES} ${catkin_LIBRARIES})

## Installing
# install(TARGETS
#   poi_manager_panel
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
