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

add_library(${PROJECT_NAME}_lib
        src/pub_sub_graph.cpp
        src/translations.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp px4_msgs px4_msgs_old)
add_executable(${PROJECT_NAME}
        src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME} rclcpp px4_msgs px4_msgs_old)
install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_unit_tests
          test/graph.cpp
          test/main.cpp
  )
  target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE ${CMAKE_CURRENT_LIST_DIR})
  target_link_libraries(${PROJECT_NAME}_unit_tests ${PROJECT_NAME}_lib)
  ament_target_dependencies(${PROJECT_NAME}_unit_tests
          rclcpp
  )
endif()

ament_package()
