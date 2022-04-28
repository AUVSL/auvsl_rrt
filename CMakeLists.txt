cmake_minimum_required(VERSION 3.0.2)
project(auvsl_rrt)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  auvsl_control
  pcl_conversions
  pcl_ros
  )


## Generate messages in the 'msg' folder
#add_message_files(
#   FILES
#   
#)

## Generate services in the 'srv' folder
#add_service_files(
#   FILES
#   
#)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES auvsl_rrt
CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions auvsl_control auvsl_dynamics
#DEPENDS   #One day I may have to figure this out.
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  /opt/ros/noetic/include/ompl-1.5/
  include/auvsl_rrt
  /home/justin/code/AUVSL_ROS/install/include
  /home/justin/code/AUVSL_ROS/install/include/auvsl_dynamics/generated
  /usr/include/pcl-1.10
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)




add_executable(test_rrt_planner_node
  src/test_global_planner.cpp
  src/OctoTerrainMap.cpp
  src/ControlSystem.cpp
  src/GlobalParams.cpp
  src/GlobalPlanner.cpp
  src/PlannerVisualizer.cpp
  src/JackalStatePropagator.cpp
  src/TerrainMap.cpp
  src/utils.cpp
  src/VehicleStateSpace.cpp
  src/VehicleStateProjections.cpp
  src/VehicleControlSampler.cpp
  src/DirectedVehicleControlSampler.cpp
  src/VehicleRRT.cpp  
)

message(STATUS "pcl_libraries: ${PCL_LIBRARIES}")

target_link_libraries(test_rrt_planner_node ${PCL_LIBRARIES})
target_link_libraries(test_rrt_planner_node ompl)
target_link_libraries(test_rrt_planner_node ${catkin_LIBRARIES})
target_link_libraries(test_rrt_planner_node rbdl)
target_link_libraries(test_rrt_planner_node /home/justin/code/AUVSL_ROS/install/lib/libauvsl_dynamics.so)


set_target_properties(test_rrt_planner_node PROPERTIES COMPILE_FLAGS "-g -O3 -DNDEBUG -march=native -Wall -Wno-undef")





add_executable(test_terrain_node src/test_terrain_map.cpp
  src/OctoTerrainMap.cpp
  src/ControlSystem.cpp
  src/GlobalParams.cpp
  src/TerrainMap.cpp
  src/utils.cpp
  )

target_link_libraries(test_terrain_node ${PCL_LIBRARIES})
target_link_libraries(test_terrain_node ${OMPL_LIBRARIES})
target_link_libraries(test_terrain_node ${catkin_LIBRARIES})
target_link_libraries(test_terrain_node rbdl)
target_link_libraries(test_terrain_node /home/justin/code/AUVSL_ROS/install/lib/libauvsl_dynamics.so)
set_target_properties(test_terrain_node PROPERTIES COMPILE_FLAGS "-O3 -g")

add_dependencies(test_terrain_node ${test_terrain_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




add_executable(test_cs_node
  src/test_control_system.cpp
  src/GlobalParams.cpp
  src/ControlSystem.cpp
  src/TerrainMap.cpp
  src/utils.cpp
)

add_dependencies(test_cs_node ${test_cs_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_cs_node ${PCL_LIBRARIES})
target_link_libraries(test_cs_node ompl)
target_link_libraries(test_cs_node ${catkin_LIBRARIES})
target_link_libraries(test_cs_node rbdl)

set_target_properties(test_cs_node PROPERTIES COMPILE_FLAGS "-O3 -g")
