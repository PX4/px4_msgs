cmake_minimum_required(VERSION 3.5)

project(px4_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(EnableC++XX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_dds_idl REQUIRED)

#########################################################
# Generate ROS messages, ROS2 interfaces and IDL files ##
#########################################################

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

file(GLOB DDS_IDL_DIR_LIST ${MSGS_DIR}/*.idl)
file(GLOB ROS_MSG_DIR_LIST ${MSGS_DIR}/*.msg)

set(DDS_IDL_DIR_LIST "${DDS_IDL_DIR_LIST}" CACHE INTERNAL "DDS_IDL_DIR_LIST")
set(ROS_MSG_DIR_LIST "${ROS_MSG_DIR_LIST}" CACHE INTERNAL "ROS_MSG_DIR_LIST")

if(NOT "${PX4_MSGS}" STREQUAL "")
  # Generate introspection typesupport for C and C++
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${PX4_MSGS}
    DEPENDENCIES builtin_interfaces
    ADD_LINTER_TESTS
  )

  # Generate Fast RTPS IDL files
  rosidl_generate_dds_interfaces(${PROJECT_NAME}__fastrtps_idl
                                  IDL_FILES
                                  ${ROS_MSG_DIR_LIST}
                                  OUTPUT_SUBFOLDERS
                                  "dds_fastrtps")

  set(DDS_IDL_FILES "" CACHE INTERNAL "DDS_IDL_FILES")
  set(DDS_IDL_BASE_PATH "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_dds_idl")
  foreach(_idl_file ${ROS_MSG_DIR_LIST})
    get_filename_component(_parent_folder "${_idl_file}" DIRECTORY)
    get_filename_component(_parent_folder "${_parent_folder}" NAME)
    get_filename_component(_extension "${_idl_file}" EXT)
    get_filename_component(_name "${_idl_file}" NAME_WE)
    if(_extension STREQUAL ".msg")
      list(
          APPEND
          DDS_IDL_FILES
          "${DDS_IDL_BASE_PATH}/${PROJECT_NAME}/${_parent_folder}/dds_fastrtps/${_name}_.idl"
          )
    elseif(_extension STREQUAL ".srv")
      list(
          APPEND
          DDS_IDL_FILES
          "${DDS_IDL_BASE_PATH}/${PROJECT_NAME}/${_parent_folder}/dds_fastrtps/Sample_${_name}_Request_.idl"
          )
      list(
          APPEND
          DDS_IDL_FILES
          "${DDS_IDL_BASE_PATH}/${PROJECT_NAME}/${_parent_folder}/dds_fastrtps/Sample_${_name}_Response_.idl"
          )
    endif()
  endforeach()

  # rosidl_generator_dds_idl requires a target to generate the IDL files
  # as we don't have one, we create a dummy target to serve the purpose
  file(WRITE ${MSGS_DIR}/idl.cc "int main() { return 0; }")
  add_executable(idl_target ${MSGS_DIR}/idl.cc)
  add_dependencies(idl_target ${PROJECT_NAME}__fastrtps_idl)
else()
  message(FATAL_ERROR "No PX4 msgs found to generate interfaces to!")
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
