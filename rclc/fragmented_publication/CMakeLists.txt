cmake_minimum_required(VERSION 3.10)

project(fragmented_publication LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)

add_executable(${PROJECT_NAME} main.c)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/ament_target_dependencies_compat.cmake)

ament_target_dependencies_compat(${PROJECT_NAME}
  rcl
  rclc
  std_msgs
  rmw_microxrcedds
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )
