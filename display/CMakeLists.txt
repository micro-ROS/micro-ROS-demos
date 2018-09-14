cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=*)
project(display)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(display REQUIRED)

add_executable(display main.c)
ament_target_dependencies(display rclc std_msgs)

install(TARGETS
  display
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
