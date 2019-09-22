cmake_minimum_required(VERSION 2.8.3)
project(drone_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES DroneControl, PID
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  $(find PROJECT_SOURCE_DIR)/../devel/include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(PID
   include/PID/PID.h
   include/PID/PID.cpp
) 

add_library(DroneControl
   include/DroneControl/DroneControl.h
   include/DroneControl/DroneControl.cpp
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(drone_teleop src/drone_teleop.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(drone_teleop PID DroneControl)

## Specify libraries to link a library or executable target against
target_link_libraries(DroneControl PID ${catkin_LIBRARIES})
target_link_libraries(PID ncurses ${catkin_LIBRARIES})
target_link_libraries(drone_teleop DroneControl PID ${catkin_LIBRARIES})


