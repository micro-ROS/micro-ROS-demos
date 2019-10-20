cmake_minimum_required(VERSION 3.5)

project(micro_ros_demos_rclcpp)

find_package(ament_cmake REQUIRED)

include(ExternalProject)

function(export_executable subdirectory)
ExternalProject_Add(${subdirectory}
            PREFIX
                ${PROJECT_BINARY_DIR}/${subdirectory}
            SOURCE_DIR
                ${PROJECT_SOURCE_DIR}/${subdirectory}
            INSTALL_DIR
                ${PROJECT_BINARY_DIR}/temp_install
            CMAKE_CACHE_ARGS
                -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
                -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
                -DCMAKE_PREFIX_PATH:PATH=${CMAKE_PREFIX_PATH}
                -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
            )
endfunction()

export_executable(complex_msg_publisher)
export_executable(complex_msg_subscriber)
export_executable(int32_publisher)
export_executable(int32_subscriber)
export_executable(RAD0_control)
export_executable(string_publisher)
export_executable(string_subscriber)

if(EXISTS ${CMAKE_BINARY_DIR}/temp_install/)
    install(
        DIRECTORY
            ${CMAKE_BINARY_DIR}/temp_install/
        DESTINATION
            lib/${PROJECT_NAME}
        USE_SOURCE_PERMISSIONS
        )
endif()

ament_package()
