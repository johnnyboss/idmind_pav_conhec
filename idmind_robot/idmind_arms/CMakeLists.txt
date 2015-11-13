cmake_minimum_required(VERSION 2.8.3)
project(idmind_arms)

find_package(catkin REQUIRED COMPONENTS
  idmind_serial
  roscpp
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  Torque.srv
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

add_executable(idmind_arms src/idmind_arms.cpp)

target_link_libraries(idmind_arms idmind_serial ${catkin_LIBRARIES})
add_dependencies(idmind_arms idmind_arms_generate_messages_cpp)
