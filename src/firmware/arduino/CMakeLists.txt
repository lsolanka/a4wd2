set(CMAKE_TOOLCHAIN_FILE arduino-cmake/cmake/ArduinoToolchain.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

cmake_minimum_required(VERSION 3.2.0)
cmake_policy(SET CMP0048 NEW)

project(sonar-rangers VERSION 0.1.0)


set(ARDUINO_DEFAULT_BOARD micro)
set(ARDUINO_DEFAULT_PORT /dev/ttyACM0)

generate_arduino_firmware(srf08_serial
    SKETCH "srf08_serial.cpp"
    PORT  /dev/ttyACM0
    SERIAL picocom @SERIAL_PORT@
    BOARD micro
)

print_programmer_list()
