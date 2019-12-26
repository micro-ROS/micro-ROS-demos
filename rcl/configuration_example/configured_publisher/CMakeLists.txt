cmake_minimum_required(VERSION 3.5)
set(CMAKE_C_CLANG_TIDY clang-tidy -checks=*)

project(configured_publisher)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rmw_microxrcedds REQUIRED)
find_package(rcl REQUIRED)

add_executable(${PROJECT_NAME} main.c)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  rmw_microxrcedds
  rcl
  )

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${PROJECT_NAME}
 )