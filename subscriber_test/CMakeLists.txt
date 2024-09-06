cmake_minimum_required(VERSION 3.8)
project(subscriber_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_msgs_old REQUIRED)

add_executable(subscriber_test src/listener.cpp)
ament_target_dependencies(subscriber_test rclcpp px4_msgs px4_msgs_old)
install(TARGETS
        subscriber_test
        DESTINATION lib/${PROJECT_NAME})

ament_package()
