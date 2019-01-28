cmake_minimum_required(VERSION 3.5)

project(px4_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

##############################################
# Generate ROS messages and ROS2 interfaces ##
##############################################

# get all msg files
file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

if(NOT "${PX4_MSGS}" STREQUAL "")
  # Generate introspection typesupport for C and C++
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${PX4_MSGS}
    DEPENDENCIES builtin_interfaces
    ADD_LINTER_TESTS
  )
else()
  message(FATAL_ERROR "No PX4 msgs found to generate interfaces to!")
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
