cmake_minimum_required(VERSION 3.0)

include(cmake/HunterGate.cmake)
HunterGate(
    URL "https://github.com/lsolanka/hunter/archive/v0.19.240-2.tar.gz"
    SHA1 "0568ce0da32b57bd478ede5fc4f4bab58a014402"
    LOCAL
)

project(random_forager)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

hunter_add_package(nlohmann_json)
hunter_add_package(roboclaw)
hunter_add_package(Boost COMPONENTS program_options system log filesystem thread)

find_package(nlohmann_json)
find_package(roboclaw)
find_package(Boost CONFIG REQUIRED COMPONENTS program_options system)


add_subdirectory(../../sensor_reader sensor_reader)

add_executable(random_forager random_forager.cpp)
target_link_libraries(random_forager nlohmann_json roboclaw::roboclaw sensor_reader Boost::boost
    Boost::program_options Boost::system)
