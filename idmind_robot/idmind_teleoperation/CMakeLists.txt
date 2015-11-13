cmake_minimum_required(VERSION 2.8.3)
project(idmind_teleoperation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  idmind_motors
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(idmind_teleoperation src/idmind_teleoperation.cpp)

add_dependencies(idmind_teleoperation ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_teleoperation ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  PROGRAMS scripts/turtlebot_teleop_key
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
