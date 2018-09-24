cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=*)
project(actuator)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(actuator main.c)
ament_target_dependencies(actuator rclc std_msgs)

install(TARGETS
  actuator
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME} micrortps_client microcdr)

ament_package()
