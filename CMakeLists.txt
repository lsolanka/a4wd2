cmake_minimum_required(VERSION 3.0)

include(cmake/HunterGate.cmake)
HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.19.197.tar.gz"
    SHA1 "243001e1b835fe4d5bc3b5c41b7e633b5be033c5"
)

project(a4wd2 VERSION 0.0.1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTS "Build unit tests" OFF)

hunter_add_package(nlohmann_json)

find_package(Boost REQUIRED COMPONENTS system log)
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
