cmake_minimum_required(VERSION 2.8.3)
project(idmind_imu)

find_package(catkin REQUIRED COMPONENTS
  idmind_serial
  roscpp
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

add_executable(idmind_imu src/idmind_imu.cpp)

add_dependencies(idmind_imu ${catkin_EXPORTED_TARGETS})

target_link_libraries(idmind_imu idmind_serial ${catkin_LIBRARIES})
