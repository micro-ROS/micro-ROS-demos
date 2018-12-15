cmake_minimum_required(VERSION 3.5)

project(int32_firos2)

include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(--std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(--std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)

# Do not build the package demo if firos is not found
find_package(firos2 QUIET)
if(NOT firos2_FOUND)
message(WARNING "${PROJECT_NAME} will be ignored due to firos2 is not installed")
    ament_package()
    return()
endif()

find_package(fastcdr QUIET)
find_package(fastrtps QUIET)

add_definitions(-DEPROSIMA_USER_DLL_EXPORT  -DBUILD_SHARED_LIBS)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS TRUE)


add_library(
    ${PROJECT_NAME} SHARED 
    userlib.cpp
)

target_link_libraries(${PROJECT_NAME} fastrtps fastcdr)
ament_target_dependencies(
    ${PROJECT_NAME}
    std_msgs
    rmw
    rmw_implementation
    firos2
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  config.xml
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()


