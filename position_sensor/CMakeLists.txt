cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=*)
project(position_sensor)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(position_sensor main.c)
ament_target_dependencies(position_sensor rclc std_msgs)

install(TARGETS
  position_sensor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
