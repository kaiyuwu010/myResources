cmake_minimum_required(VERSION 3.0.2)
project(scout_state_machine_communication)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


add_message_files(
FILES
BehaviorState.msg
)
add_service_files(
FILES
Event.srv
)
generate_messages(
DEPENDENCIES
std_msgs
)

# Include catkin_libraries
catkin_package(
    CATKIN_DEPENDS 
    std_msgs 
    message_runtime
)



# install(TARGETS
#   DESTINATION lib/${PROJECT_NAME}
#   PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}/
#   )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

######################################
# < End ROS
######################################
