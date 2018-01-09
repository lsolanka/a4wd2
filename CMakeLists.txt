cmake_minimum_required(VERSION 3.0)

include(cmake/HunterGate.cmake)
HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.19.197.tar.gz"
    SHA1 "243001e1b835fe4d5bc3b5c41b7e633b5be033c5"
)

# Set up ROS correctly so that we don't have to use catkin_make
# Still requires sourcing the setup.bash script
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_PREFIX_PATH})

project(a4wd2 VERSION 0.0.1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTS "Build unit tests" ON)

hunter_add_package(nlohmann_json)

find_package(Boost REQUIRED COMPONENTS system log)
find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp)

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

catkin_package()

add_subdirectory(src/sensor_reader)
add_subdirectory(src/sensor_publisher)
