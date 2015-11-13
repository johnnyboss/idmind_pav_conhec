cmake_minimum_required(VERSION 2.8.3)
project(idmind_sensors)

find_package(catkin REQUIRED COMPONENTS
  idmind_motors
  idmind_serial
  message_generation
  roscpp
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  Batteries.srv
  DockUndock.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(idmind_sensors src/idmind_sensors.cpp)

add_dependencies(idmind_sensors ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_sensors idmind_serial ${catkin_LIBRARIES})

add_dependencies(idmind_sensors idmind_sensors_generate_messages_cpp)
