cmake_minimum_required(VERSION 3.10)

project(addtwoints_server LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(${PROJECT_NAME} main.c)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ament_target_dependencies_compat.cmake)

ament_target_dependencies_compat(${PROJECT_NAME}
  rcl
  rclc
  example_interfaces
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )
