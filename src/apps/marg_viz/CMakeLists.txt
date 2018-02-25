cmake_minimum_required(VERSION 3.0)

include(cmake/HunterGate.cmake)
HunterGate(
    URL "https://github.com/lsolanka/hunter/archive/v0.19.240-2.tar.gz"
    SHA1 "0568ce0da32b57bd478ede5fc4f4bab58a014402"
    LOCAL
)

project(marg_viz)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

hunter_add_package(OpenCV)
hunter_add_package(nlohmann_json)
hunter_add_package(Boost COMPONENTS system log thread math)
hunter_add_package(MadgwickAHRS)
hunter_add_package(Eigen)

find_package(OpenCV CONFIG REQUIRED)
find_package(VTK CONFIG REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS system log thread math_tr1)
find_package(MadgwickAHRS CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

add_subdirectory(../../sensor_reader sensor_reader)

add_executable(marg_viz marg_viz.cpp)
target_link_libraries(marg_viz ${OpenCV_LIBS} sensor_reader MadgwickAHRS::MadgwickAHRS)
