cmake_minimum_required(VERSION 2.8.3)
project(idmind_odometry)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(idmind_odometry src/idmind_odometry.cpp)

add_dependencies(idmind_odometry ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_odometry ${catkin_LIBRARIES})
