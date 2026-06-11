cmake_minimum_required(VERSION 3.8)

project(px4_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# rosidl_default_generators must be found before the interface dependencies so
# the available typesupports are registered first; on newer rosidl (Rolling,
# Kilted) finding builtin_interfaces first fails with "No 'rosidl_typesupport_c'
# found".
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# #############################################################################
# Generate ROS messages, ROS 2 interfaces and IDL files
# #############################################################################

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

# get all srv files
set(SRVS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(GLOB PX4_SRVS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRVS_DIR}/*.srv")

# Generate introspection typesupport for C and C++ and IDL files
rosidl_generate_interfaces(${PROJECT_NAME}
  ${PX4_MSGS}
  ${PX4_SRVS}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The message/service definitions are generated from PX4-Autopilot and the
  # remaining sources carry no per-file copyright headers, so skip the
  # copyright linter (the license is declared in package.xml / LICENSE).
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
