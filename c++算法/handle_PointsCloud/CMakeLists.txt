cmake_minimum_required(VERSION 2.4.6)
project(handlePCD)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
ENDIF()
MESSAGE("Build type: " ${ROS_BUILD_TYPE})

# set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")
# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(PCL /home/wky/myLib/opencv-4.2.0/opencv-4.2.0-install-inventory/lib/cmake/opencv4)
find_package(PCL REQUIRED) 
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

rosbuild_add_executable(handle_pcd src/handlePCDFile.cpp) 
target_link_libraries(handle_pcd ${PCL_LIBRARIES}) 

