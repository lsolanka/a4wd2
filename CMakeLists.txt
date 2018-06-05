cmake_minimum_required(VERSION 3.0)

include(cmake/HunterGate.cmake)
HunterGate(
    URL "https://github.com/lsolanka/hunter/archive/v0.21.6-rosdevel4.tar.gz"
    SHA1 "a569b63bef4ea73e109d01c03891f91b8e32739c"
    LOCAL
)

# Set up ROS correctly so that we don't have to use catkin_make
# Still requires sourcing the setup.bash script
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_PREFIX_PATH})

project(a4wd2 VERSION 0.0.1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_ROS_COMPONENTS "Build components requiring ROS" OFF)

hunter_add_package(nlohmann_json)
if (BUILD_ROS_COMPONENTS)
    hunter_add_package(ros_comm)
    hunter_add_package(ros_common_msgs)
endif()
hunter_add_package(roboclaw)

find_package(Boost CONFIG REQUIRED COMPONENTS system log thread)

include_directories(${Boost_INCLUDE_DIRS})

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

add_subdirectory(src/sensor_reader)
#add_subdirectory(src/incubator/random_forager)

# ROS components
if (BUILD_ROS_COMPONENTS)
    find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp)
    catkin_package()

    add_subdirectory(src/sensor_publisher)
endif()
