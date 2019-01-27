cmake_minimum_required(VERSION 3.5)

project(px4_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

##############################################
# Generate ROS messages and ROS2 interfaces ##
##############################################

# get all msg files
file(GLOB PX4_MSGS "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

if(NOT "${PX4_MSGS}" STREQUAL "")
  # Generate introspection typesupport for C and C++
  rosidl_generate_interfaces(${PROJECT_NAME} ${ROS_UORB_MSGS})
else()
  message(FATAL_ERROR "No PX4 msgs found to generate interfaces to!")
endif()

ament_package()
