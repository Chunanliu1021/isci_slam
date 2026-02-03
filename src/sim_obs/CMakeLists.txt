cmake_minimum_required(VERSION 3.0.2)
project(sim_obs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf
  gazebo_ros
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

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
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sim_obs
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  DEPENDS
    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories("/usr/include/eigen3")
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# For earthquake_plugin plugin
add_library(plannar_unitCylinder_plugin src/plannar_unitCylinder.cpp)
target_link_libraries(plannar_unitCylinder_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For earthquake_plugin plugin
add_library(plannar_Desk_plugin src/plannar_Desk.cpp)
target_link_libraries(plannar_Desk_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For earthquake_plugin plugin
add_library(plannar_T_plugin src/plannar_T.cpp)
target_link_libraries(plannar_T_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#maker pub
ADD_EXECUTABLE(marker_pub src/marker_pub.cpp )
TARGET_LINK_LIBRARIES(marker_pub ${catkin_LIBRARIES} ${Boost_LIBRARIES} rt)

#obs_telep
ADD_EXECUTABLE(obs_telep src/obs_telep.cpp )
TARGET_LINK_LIBRARIES(obs_telep ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#obs_telep
ADD_EXECUTABLE(Desk_telep src/obs_telep_Desk.cpp )
TARGET_LINK_LIBRARIES(Desk_telep ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#obs_telep
ADD_EXECUTABLE(T_telep src/obs_telep_T.cpp )
TARGET_LINK_LIBRARIES(T_telep ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})