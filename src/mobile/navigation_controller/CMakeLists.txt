cmake_minimum_required(VERSION 2.8.3)
project(navigation_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wno-write-strings ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)

add_service_files(
  FILES
  command.srv
  AgvNav.srv
#   Service1.srv
#   Service2.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(

  CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  # 確保可以使用 OpenCV

)


add_executable(command_pub_srv_gui src/command_pub_srv_gui.cpp)
target_link_libraries(command_pub_srv_gui ${catkin_LIBRARIES})
add_dependencies(command_pub_srv_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(command_pub_srv src/command_pub_srv.cpp)
target_link_libraries(command_pub_srv ${catkin_LIBRARIES})
add_dependencies(command_pub_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(command_pub_srv_call src/command_pub_srv_call.cpp)
target_link_libraries(command_pub_srv_call ${catkin_LIBRARIES})
add_dependencies(command_pub_srv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(command_pub src/command_pub.cpp)
target_link_libraries(command_pub ${catkin_LIBRARIES})
add_dependencies(command_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(navigation_controller src/navigation_controller.cpp)
target_link_libraries(navigation_controller ${catkin_LIBRARIES})
add_dependencies(navigation_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(navigation_controller_human src/navigation_controller_human.cpp)
target_link_libraries(navigation_controller_human ${catkin_LIBRARIES})
add_dependencies(navigation_controller_human ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(navigation_controller_simple src/navigation_controller_simple.cpp)
target_link_libraries(navigation_controller_simple ${catkin_LIBRARIES})
add_dependencies(navigation_controller_simple ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(navigation_controller_simple_map_nav src/navigation_controller_simple_map_nav.cpp)
target_link_libraries(navigation_controller_simple_map_nav ${catkin_LIBRARIES})
add_dependencies(navigation_controller_simple_map_nav ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(human_detector src/human_detector.cpp)
target_link_libraries(human_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  cv_bridge
)