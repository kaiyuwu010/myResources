cmake_minimum_required(VERSION 2.8.3)
project(yaml)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(./dependency/include/)


add_executable(yaml yaml.hpp)
target_link_libraries(yaml ${CMAKE_CURRENT_SOURCE_DIR}/dependency/lib/libyaml-cpp.a)

