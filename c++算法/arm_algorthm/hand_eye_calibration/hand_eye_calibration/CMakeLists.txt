cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calibration)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
set(Eigen3_DIR /home/wky/myLib/Eigen3.4.90/Eigen3.4.90-install-folder/share/eigen3/cmake)
find_package(Eigen3 REQUIRED)
 
include_directories(
  ${EIGEN3_INCLUDE_DIRS}

)

catkin_package(
  LIBRARIES hand_eye_calibration
  CATKIN_DEPENDS roscpp
)

add_executable(hand_eye_calibration ./src/hand_eye_calibration.cpp)
target_link_libraries(hand_eye_calibration 
  ${EIGEN3_LIBRARIES}
)
