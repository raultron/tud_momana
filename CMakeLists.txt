cmake_minimum_required(VERSION 2.8.3)
project(tud_momana)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    roscpp
    rospy
    std_msgs
    std_srvs
    nav_msgs
    message_generation
    actionlib_msgs
    actionlib
    tf
)

find_package(Boost REQUIRED COMPONENTS thread)

#For Qt
#find_package(Qt4 COMPONENTS QtCore QtGui)
#INCLUDE(${QT_USE_FILE})
#ADD_DEFINITIONS(${QT_DEFINITIONS})

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_service_files(
  DIRECTORY srv
  FILES
  StartStopMomana.srv
  StartOdometry.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(DEPENDENCIES  std_msgs)

generate_messages(DEPENDENCIES std_msgs)

## Generate dynamic parameters
#generate_dynamic_reconfigure_options(
#  cfg/dynamic_param_config.cfg
#)


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
#  LIBRARIES coop_uv
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs geometry_msgs nav_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
## Declare a cpp library
# add_library(coop_uv
#   src/${PROJECT_NAME}/coop_uv.cpp
# )


set(MOMANAODOM_SOURCE_FILES
    src/momaodom.cpp
    src/momaodomnode.cpp
)

set(MOMANAODOM_HEADER_FILES
    include/tud_momana/momaodomnode.hpp
)


## Declare a cpp executable
add_executable(moma_odom ${MOMANAODOM_SOURCE_FILES} ${MOMANAODOM_HEADER_FILES})
add_executable(joy_control src/joy_control.cpp)
#add_executable(robotino_joy_control src/robotino_joy_control.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

add_dependencies(
  moma_odom
  tud_momana_generate_messages_cpp)

#add_dependencies(
#  joy_control_momana
#  tud_coop_uv_generate_messages_cpp
#  tud_momana_generate_messages_cpp
#  )

#add_dependencies(
#  robotino_joy_control
#  tud_momana_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(moma_odom ${catkin_LIBRARIES})
target_link_libraries(joy_control ${catkin_LIBRARIES})
#target_link_libraries(robotino_joy_control ${catkin_LIBRARIES})

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
# install(TARGETS coop_uv coop_uv_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_coop_uv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

##################
# FOR QT CREATOR
##################
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/tud_momana/*.hpp)
#FILE(GLOB_RECURSE INC_ALL "include/tud_momana/*.hpp")
#FILE(GLOB_RECURSE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

