cmake_minimum_required(VERSION 2.8.3)
project(idmind_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(idmind_serial src/${PROJECT_NAME}/idmind_serial.cpp)

#add_executable(idmind_serial_tests src/idmind_serial_tests.cpp)
#add_executable(short_to_bytes src/short_to_bytes.cpp)

target_link_libraries(idmind_serial ${catkin_LIBRARIES})

#target_link_libraries(idmind_serial_tests ${catkin_LIBRARIES})
#target_link_libraries(short_to_bytes ${catkin_LIBRARIES})
