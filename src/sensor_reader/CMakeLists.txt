find_package(nlohmann_json CONFIG REQUIRED)

set(SRC_FILES
    sensor_reader.cpp
    string_line_reader.cpp
    srf08.cpp
    mpu9250.cpp
)
add_library(sensor_reader ${SRC_FILES})
target_include_directories(sensor_reader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
if (NOT HUNTER_ENABLED)
    target_compile_definitions(sensor_reader PUBLIC BOOST_LOG_DYN_LINK)
else()
    set(_boost_libs Boost::boost Boost::log Boost::system Boost::thread)
endif()
target_link_libraries(sensor_reader
                      nlohmann_json ${_boost_libs})

if(BUILD_TESTS)
    add_gtest(test_sensor_reader test/unit/test_sensor_reader.cpp)
    target_link_libraries(test_sensor_reader sensor_reader)

    add_gtest(test_mpu9250 test/unit/test_mpu9250.cpp)
    target_link_libraries(test_mpu9250 sensor_reader)

    add_executable(serial_reader test/integration/serial_reader.cpp)
    target_link_libraries(serial_reader sensor_reader)
endif()
