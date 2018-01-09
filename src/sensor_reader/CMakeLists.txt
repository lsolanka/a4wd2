find_package(nlohmann_json CONFIG REQUIRED)

set(SRC_FILES
    sensor_reader.cpp
    string_line_reader.cpp
    srf08.cpp
)
add_library(sensor_reader ${SRC_FILES})
target_include_directories(sensor_reader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(sensor_reader nlohmann_json ${Boost_LIBRARIES})
target_compile_definitions(sensor_reader PUBLIC BOOST_LOG_DYN_LINK)

if(BUILD_TESTS)
    add_gtest(test_sensor_reader test/unit/test_sensor_reader.cpp)
    target_link_libraries(test_sensor_reader sensor_reader)

    add_executable(serial_reader test/integration/serial_reader.cpp)
    target_link_libraries(serial_reader sensor_reader)
endif()
