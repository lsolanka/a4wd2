cmake_minimum_required(VERSION 3.0)

include(cmake/HunterGate.cmake)
HunterGate(
    URL "https://github.com/lsolanka/hunter/archive/v0.22.4-rosdevel9.tar.gz"
    SHA1 "adbb26aec1758a6dbc77028e37e90dd6a1a48bac"
    LOCAL
)

# Set up ROS correctly so that we don't have to use catkin_make
# Still requires sourcing the setup.bash script
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_PREFIX_PATH})

project(a4wd2 VERSION 0.0.1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CATKIN_BUILD_BINARY_PACKAGE ON)

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_ROS_COMPONENTS "Build components requiring ROS" OFF)

hunter_add_package(Boost COMPONENTS system)
hunter_add_package(cxxopts)
hunter_add_package(nlohmann_json)
hunter_add_package(roboclaw)
hunter_add_package(spdlog)

if (BUILD_ROS_COMPONENTS)
    hunter_add_package(ros_comm)
    hunter_add_package(ros_common_msgs)
endif()

if(BUILD_TESTS)
    enable_testing()
    hunter_add_package(GTest)
    find_package(GTest CONFIG REQUIRED)

    macro(add_gtest target_name)
        add_executable(${target_name} ${ARGN})
        target_link_libraries(${target_name} GTest::main)
        add_test(NAME test-${target_name} COMMAND ${target_name})
    endmacro()
endif()

find_package(Boost CONFIG REQUIRED COMPONENTS system)
find_package(spdlog CONFIG REQUIRED)

add_subdirectory(src/sensor_reader)

# ROS components
if (BUILD_ROS_COMPONENTS)
    find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs roscpp)

    add_message_files(FILES
        control_command.msg
    )
    generate_messages(DEPENDENCIES std_msgs sensor_msgs)

    catkin_package(
        CATKIN_DEPENDS std_msgs message_runtime
    )

    add_subdirectory(src/sensor_publisher)
    add_subdirectory(src/ros_controller)

    # CPack
    if(CMAKE_CROSSCOMPILING)
        if("${CMAKE_LIBRARY_ARCHITECTURE}" STREQUAL "arm-linux-gnueabihf")
            set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
        else()
            message(FATAL_ERROR
                "CMAKE_LIBRARY_ARCHITECTURE is not set or not supported: "
                "'${CMAKE_LIBRARY_ARCHITECTURE}'")
        endif()
    endif()
    set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
    set(CPACK_PACKAGE_CONTACT "lsolanka AT gmail.com")
    set(CPACK_PACKAGE_VENDOR "Lukas Solanka")
    set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
    set(CPACK_PACKAGE_FILE_NAME
        "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
    set(CPACK_PACKAGING_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")
    set(CPACK_GENERATOR "DEB")

    set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
        "${CMAKE_CURRENT_SOURCE_DIR}/deb/postinst;${CMAKE_CURRENT_SOURCE_DIR}/deb/prerm")

    add_subdirectory(system)
    include(CPack)
endif()
