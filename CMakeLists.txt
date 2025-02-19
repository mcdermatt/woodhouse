cmake_minimum_required(VERSION 3.0.2)
project(woodhouse)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Add this line to define the alignment for Eigen
add_definitions(-DEIGEN_ALIGN_16)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  icet
  # eigen3
)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  KeyframeData.msg
  GetTheseClouds.msg
  HereAreTheClouds.msg
  LoopClosed.msg
  KeyframePose.msg
  KeyframePoses.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

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
 LIBRARIES woodhouse
 CATKIN_DEPENDS geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs message_runtime icet
 DEPENDS system_lib
)

# Set optimization flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fno-tree-vectorize -fno-inline") #Test for debugging-- way too slow


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include #not sure why this was breaking laptop earlier
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${icet_INCLUDE_DIRS}
)

add_executable(scan_context_node 
                src/scan_context_keyframe_grabber.cpp 
                src/Scancontext.cpp)
target_link_libraries(scan_context_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  tbb
)

add_executable(pose_graph_node 
                src/pose_graph.cpp)
target_link_libraries(pose_graph_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  tbb
)

add_executable(loop_closer_node 
                src/loop_closer.cpp)
target_link_libraries(loop_closer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  tbb
  icet
)