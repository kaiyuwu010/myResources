cmake_minimum_required(VERSION 2.8.3)
project(arm_planning)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  std_msgs
  pcl_msgs
)
set(Eigen3_DIR /home/wky/myLib/eigen-3.4.01/eigen-install-folder/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
# find_package(octomap REQUIRED)
# find_package(PCL REQUIRED)
# find_package(octomap_msgs REQUIRED)
 
include_directories(
  include
  dependency/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  # ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES test
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_msgs
)

add_executable(kinematicsTest src/test/kinematicsTest.cpp)
target_link_libraries(kinematicsTest 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
find_library(MY_LIB NAMES libyaml-cpp.a PATHS /dependency/lib/)
if (MY_LIB)
    target_link_libraries(kinematicsTest ${MY_LIB})
endif()
