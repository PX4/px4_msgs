cmake_minimum_required(VERSION 3.8)
project(translation_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_msgs_old REQUIRED)

add_executable(translation_node
        src/main.cpp
        src/ros_translations.cpp
        src/translations.cpp
)
ament_target_dependencies(translation_node rclcpp px4_msgs px4_msgs_old)
install(TARGETS
        translation_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
