cmake_minimum_required(VERSION 3.0.2)
project(cortex_message_handling)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Spinnaker include_directories
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" CACHE PATH "Project Include Directory")
set(SPINNAKER_INCLUDE_DIR "/usr/include/spinnaker" CACHE PATH "Spinnaker Include Directory")
set(SPINNAKER_LIB_DIR "/usr/lib" CACHE PATH "Spinnaker Libs Directory")

#set(ZeroMQ_INCLUDE_DIR "/usr/include/zmq*" CACHE PATH "ZeroMQ Include Directory")
#set(ZeroMQ_LIB_DIR "/usr/lib/x86_64-linux-gnu" CACHE PATH "ZeroMQ Libs Directory")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CortexCommands.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
# LIBRARIES cortex_message_handling
 CATKIN_DEPENDS roscpp std_msgs message_runtime
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SPINNAKER_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include
)
link_directories( ${SPINNAKER_LIB_DIR})

## build debug executable
add_executable(relay_debug src/relay_debug.cpp)
target_link_libraries(relay_debug ${catkin_LIBRARIES} -ldl)
add_dependencies(relay_debug ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## build simulator_relay library
add_library(simulator_relay src/simulator_relay.cpp)
target_link_libraries(simulator_relay ${catkin_LIBRARIES})
add_dependencies(simulator_relay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## build video feed
#add_executable(video_feed src/video_feed.cpp)
#target_link_libraries(video_feed Spinnaker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
#add_dependencies(video_feed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## build blackfly_binocular_relay library
#add_library(blackfly_binocular_relay SHARED src/blackfly_binocular_relay.cpp)
#add_dependencies(blackfly_binocular_relay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(blackfly_binocular_relay Spinnaker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

## build live_binocular_relay executable
add_executable(live_binocular_relay src/live_binocular_relay.cpp src/blackfly_binocular_relay.cpp)
target_link_libraries(live_binocular_relay Spinnaker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} /usr/lib/x86_64-linux-gnu/libzmq.so)
add_dependencies(live_binocular_relay ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
