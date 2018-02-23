set(CMAKE_TOOLCHAIN_FILE arduino-cmake/cmake/ArduinoToolchain.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.2.0)
cmake_policy(SET CMP0048 NEW)

project(sonar-rangers VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-threadsafe-statics")

include_directories(ArduinoJson/src)
include_directories(include)

set(ARDUINO_DEFAULT_BOARD micro)
set(ARDUINO_DEFAULT_PORT /dev/ttyACM0)

generate_arduino_firmware(arduino_sensor_processor
    SRCS arduino_sensor_processor.cpp
         mpu9250.cpp
         I2Cdev/I2Cdev.cpp
    HDRS include/mpu9250/mpu9250.hpp
         I2Cdev/I2Cdev.h
    PORT  /dev/ttyACM0
    BOARD micro
)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DI2CDEV_SERIAL_DEBUG")

print_programmer_list()
