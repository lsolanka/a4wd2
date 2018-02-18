cmake_minimum_required(VERSION 3.0)

include(cmake/HunterGate.cmake)
HunterGate(
    URL "https://github.com/lsolanka/hunter/archive/v0.19.240-1.tar.gz"
    SHA1 "d22f66d963a2833700216c9d0b02a32e0442422a"
    LOCAL
)

project(marg_viz)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

hunter_add_package(OpenCV)
hunter_add_package(nlohmann_json)
hunter_add_package(Boost COMPONENTS system log thread)

find_package(OpenCV CONFIG REQUIRED)
find_package(VTK CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS system log thread)

add_subdirectory(../../sensor_reader sensor_reader)

add_executable(marg_viz marg_viz.cpp)
target_link_libraries(marg_viz ${OpenCV_LIBS} sensor_reader)
