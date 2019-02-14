cmake_minimum_required(VERSION 2.8.3)

project(px4_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

##############################################
# Generate ROS messages and ROS2 interfaces ##
##############################################

# get all msg files
file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

if(NOT "${PX4_MSGS}" STREQUAL "")
  add_message_files(
    DIRECTORY msg
    FILES
    ${PX4_MSGS}
  )
else()
  message(FATAL_ERROR "No PX4 msgs found to generate interfaces to!")
endif()

## Generate added messages
generate_messages()

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

## Add include dirs
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

#TODO

endif()
