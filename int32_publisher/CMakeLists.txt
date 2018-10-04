cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=*)
project(int32_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclc QUIET)
find_package(std_msgs REQUIRED)

# Do not compile package if rclcpp is not found
if (NOT rclc_FOUND)
  message(WARNING "${PROJECT_NAME} will be ignored due to rclc is not installed")
  return()
endif()

add_executable(${PROJECT_NAME} main.c)
ament_target_dependencies(${PROJECT_NAME} rclc std_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
