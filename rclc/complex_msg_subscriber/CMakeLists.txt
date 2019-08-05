cmake_minimum_required(VERSION 3.5)

project(complex_msg_subscriber_rclc LANGUAGES C)

find_program(CLANG_TIDY clang-tidy)
if(CLANG_TIDY)
  set(CMAKE_C_CLANG_TIDY clang-tidy -checks=*)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclc QUIET)
find_package(complex_msgs REQUIRED)

# Do not compile package if rclcpp is not found
if (NOT rclc_FOUND)
  message(WARNING "${PROJECT_NAME} will be ignored due to rclc is not installed")
  ament_package()
  return()
endif()

add_executable(${PROJECT_NAME}  main.c)
ament_target_dependencies(${PROJECT_NAME} rclc complex_msgs)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION
    bin/${PROJECT_NAME}
  )

ament_package()
