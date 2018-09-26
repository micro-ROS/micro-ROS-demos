cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=*)
project(example_publisher_int32)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(example_publisher_int32 main.c)
ament_target_dependencies(example_publisher_int32 rclc std_msgs)

install(TARGETS
  example_publisher_int32
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
