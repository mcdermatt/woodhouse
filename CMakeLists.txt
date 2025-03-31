cmake_minimum_required(VERSION 3.0.2)
project(woodhouse)
# include_directories(/usr/local/include/gtsam/3rdparty)

#old
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#for compatibility with GTSAM??
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lboost_system -DGTSAM_USE_SYSTEM_EIGEN=ON")


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Add this line to define the alignment for Eigen
# add_definitions(-DEIGEN_ALIGN_16)
# add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

# find_package(PCL REQUIRED PATHS ~/ROS/src/pcl-pcl-1.10.0/build NO_DEFAULT_PATH)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
# link_directories(/usr/local/lib)

find_package(GTSAM REQUIRED) #CONFIG PATHS /usr/local/lib/cmake/GTSAM)
find_package(Boost REQUIRED COMPONENTS system timer thread serialization chrono)

find_package(TBB REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  icet
)

# Explicitly find GTSAM after setting the directory
# set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM")
# set(GTSAM_INCLUDE_DIRS "/usr/local/include")
# set(GTSAM_LIBRARIES "/usr/local/lib/libgtsam.so")
# find_package(GTSAM REQUIRED)
# include_directories(${GTSAM_INCLUDE_DIR})

find_package(OpenCV REQUIRED)

# #sanitizaton
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")

add_message_files(
  FILES
  KeyframeData.msg
  GetTheseClouds.msg
  HereAreTheClouds.msg
  LoopClosed.msg
  KeyframePose.msg
  KeyframePoses.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES woodhouse
 CATKIN_DEPENDS geometry_msgs pcl_ros roscpp rospy sensor_msgs std_msgs message_runtime icet
 DEPENDS system_lib
)

# Set optimization flags
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native") #was this before
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2") #-march=native is problematic with GTSAM??
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fno-tree-vectorize -fno-inline") #Test for debugging-- way too slow


include_directories(
  include #not sure why this was breaking laptop earlier
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${icet_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${TBB_INCLUDE_DIRS}
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
  ${GTSAM_LIBRARIES}
  ${TBB_LIBRARIES}
  tbb
  gtsam
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

add_executable(loop_closer_eigen_node 
                src/loop_closer_eigen.cpp)
target_link_libraries(loop_closer_eigen_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  tbb
  icet
)