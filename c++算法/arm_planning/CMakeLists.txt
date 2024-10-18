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
set(nlohmann_json_DIR /home/wky/myLib/json/json-install-folder/share/cmake/nlohmann_json)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
# find_package(octomap REQUIRED)
# find_package(PCL REQUIRED)
# find_package(octomap_msgs REQUIRED)
 
include_directories(
  include
  dependency/yaml/include
  dependency/rm-arm/include
  dependency/gripper/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  # ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES test
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_msgs
)
add_library(trajectoryPlanning src/trajectoryPlanning.cpp)
target_link_libraries(trajectoryPlanning 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/dependency/rm-arm/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/dependency/gripper/lib
)


add_executable(armPlanning src/armPlanning.cpp)
target_link_libraries(armPlanning 
  ${catkin_LIBRARIES}
  nlohmann_json::nlohmann_json
  libRM_Service.so
  trajectoryPlanning
  ${YAML_LIB}
)

add_executable(kinematicsTest src/test/kinematicsTest.cpp)
target_link_libraries(kinematicsTest 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  libRM_Service.so
)
find_library(YAML_LIB NAMES libyaml-cpp.a PATHS /dependency/yaml/lib)
if (YAML_LIB)
    target_link_libraries(kinematicsTest ${YAML_LIB})
    message("---------------------------finded library libyaml-cpp.a")
else()
    message(WARNING "Could not find the custom library libyaml-cpp.a")
endif()

add_executable(communicat_test src/test/communit_test.cpp)
target_link_libraries(communicat_test 
  ${catkin_LIBRARIES}
)

add_executable(forceControlTest src/test/forceControlTest.cpp)
target_link_libraries(forceControlTest 
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  libRM_Service.so
  nlohmann_json::nlohmann_json
)

add_executable(arm_move_test src/test/arm_move_test.cpp)
target_link_libraries(arm_move_test 
  ${catkin_LIBRARIES}
  libRM_Service.so
  nlohmann_json::nlohmann_json
)
add_executable(trajectorytest src/test/trajectorytest.cpp)
target_link_libraries(trajectorytest 
  ${catkin_LIBRARIES}
  trajectoryPlanning
)
add_executable(visionTest src/visionTest.cpp)
target_link_libraries(visionTest 
  ${catkin_LIBRARIES}
  Threads::Threads
  libCanCmd.so
  nlohmann_json::nlohmann_json
)
# add_executable(rrtTest src/test/rrtTest.cpp src/RRTConnect.cpp)
# target_link_libraries(rrtTest 
#   ${catkin_LIBRARIES}
# )