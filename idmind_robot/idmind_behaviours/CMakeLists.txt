cmake_minimum_required(VERSION 2.8.3)
project(idmind_behaviours)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
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

add_executable(idmind_behaviours src/idmind_behaviours.cpp src/read_files.cpp)

add_dependencies(idmind_behaviours ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_behaviours ${catkin_LIBRARIES})