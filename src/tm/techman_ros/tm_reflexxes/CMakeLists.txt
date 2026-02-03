cmake_minimum_required(VERSION 2.8.3)
project(tm_reflexxes)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  tm_driver
  tm_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  trajectory_msgs
  tf
  # tm_kinematics
)

# add_library(TM_print SHARED IMPORTED)
# set_target_properties(TM_print PROPERTIES
#   IMPORTED_LOCATION /home/peter/Desktop/tm_control/devel/.private/tm_driver/lib/libTM_print.so
# )

link_directories(/home/caliiu/ws_hdl_slam/devel/lib)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES TM_reflexxes RML_library TM_print
  CATKIN_DEPENDS roscpp
)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	ReflexxesTypeII/include 
  )

#############
## Build   ##
#############


file(GLOB REFLEXXES_SRC
    ReflexxesTypeII/src/*.cpp
)


add_library(RML_library ${REFLEXXES_SRC})
target_link_libraries(RML_library ${catkin_LIBRARIES})

add_library(TM_reflexxes src/tm_reflexxes.cpp)
target_link_libraries(TM_reflexxes 
  RML_library 
  ${catkin_LIBRARIES})


add_executable(tm_otg src/tm_otg.cpp)
target_link_libraries(tm_otg
	TM_reflexxes
  TM_print
	${catkin_LIBRARIES}
)

add_executable(tm_sim_rviz src/tm_sim_rviz.cpp)
target_link_libraries(tm_sim_rviz
  TM_reflexxes
  RML_library
  TM_print
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ReflexxesTypeII/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

