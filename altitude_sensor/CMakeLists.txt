cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=*)
project(altitude_sensor)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(altitude_sensor main.c)
ament_target_dependencies(altitude_sensor rclc std_msgs)

target_link_libraries(altitude_sensor m)

install(TARGETS
  altitude_sensor
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(${PROJECT_NAME} micrortps_client microcdr)

ament_package()
