cmake_minimum_required(VERSION 3.10)

project(custom_transports LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(microxrcedds_client REQUIRED)

add_executable(${PROJECT_NAME} main.c)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/ament_target_dependencies_compat.cmake)

ament_target_dependencies_compat(${PROJECT_NAME}
  rcl
  rclc
  std_msgs
  rmw_microxrcedds
  microxrcedds_client
  )

set_target_properties(${PROJECT_NAME} PROPERTIES
  CXX_STANDARD
      11
  CXX_STANDARD_REQUIRED
      YES
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${PROJECT_NAME}
  )
