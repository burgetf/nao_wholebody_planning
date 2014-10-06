# cmake_minimum_required(VERSION 2.4.6)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# 
# # Set the build type.  Options are:
# #  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
# #  Debug          : w/ debug symbols, w/o optimization
# #  Release        : w/o debug symbols, w/ optimization
# #  RelWithDebInfo : w/ debug symbols, w/ optimization
# #  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# #set(ROS_BUILD_TYPE RelWithDebInfo)
# 
# rosbuild_init()
# 
# #set the default path for built executables to the "bin" directory
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# #set the default path for built libraries to the "lib" directory
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
# 
# #uncomment if you have defined messages
# #rosbuild_genmsg()
# #uncomment if you have defined services
# #rosbuild_gensrv()
# 
# #common commands for building c++ executables and libraries
# #rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
# #target_link_libraries(${PROJECT_NAME} another_library)
# #rosbuild_add_boost_directories()
# #rosbuild_link_boost(${PROJECT_NAME} thread)
# #rosbuild_add_executable(example examples/example.cpp)
# rosbuild_add_executable(planning_example_simulation src/planning_example_simulation.cpp)
# rosbuild_add_executable(planning_example_execution src/planning_example_execution.cpp)
# 
# 
# #target_link_libraries(example ${PROJECT_NAME})
# #target_link_libraries(planning_example ${PROJECT_NAME})





cmake_minimum_required(VERSION 2.8.3)
project(rrt_connect_planner_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #moveit_planners_ompl
  moveit_core
  moveit_ros_planning
  rrt_connect_planner
  rrt_planner_msgs
  actionlib
  actionlib_msgs
  nao_msgs
)

find_package(Eigen REQUIRED)




## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#  interface_msg.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs std_msgs geometry_msgs
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS message_runtime
    CATKIN_DEPENDS moveit_core moveit_ros_planning rrt_connect_planner rrt_planner_msgs actionlib actionlib_msgs nao_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS}
)
MESSAGE(STATUS ${Eigen_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(planning_example_simulation src/planning_example_simulation.cpp)
add_executable(planning_example_execution src/planning_example_execution.cpp)
add_executable(planning_example_execution_parallel src/planning_example_execution_parallel.cpp)

add_executable(drawer_manipulation_task_simulation src/drawer_manipulation_task_simulation.cpp)

target_link_libraries(planning_example_simulation ${catkin_LIBRARIES})
target_link_libraries(planning_example_execution ${catkin_LIBRARIES})
target_link_libraries(planning_example_execution_parallel ${catkin_LIBRARIES})

target_link_libraries(drawer_manipulation_task_simulation ${catkin_LIBRARIES})


