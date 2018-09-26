cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=*)
project(example_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(example_publisher main.c)
ament_target_dependencies(example_publisher rclc std_msgs)

install(TARGETS
  example_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
