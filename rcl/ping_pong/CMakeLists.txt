cmake_minimum_required(VERSION 3.5)

project(microros_demo_ping_pong)

#################################################
# compiler settings
#################################################

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -g -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(test_ping_pong src/main.c)
ament_target_dependencies(test_ping_pong rcl rclc std_msgs)

install(TARGETS
  test_ping_pong
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

