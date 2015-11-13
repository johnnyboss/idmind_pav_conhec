cmake_minimum_required(VERSION 2.8.3)
project(idmind_motors)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  idmind_serial
  message_generation
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  Hardstop.srv
  VoltagesStatus.srv
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

add_executable(idmind_motors src/idmind_motors.cpp src/drives.cpp)

add_dependencies(idmind_motors ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_motors idmind_serial ${catkin_LIBRARIES})

add_dependencies(idmind_motors idmind_motors_generate_messages_cpp)
