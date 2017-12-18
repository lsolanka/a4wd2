find_package(nlohmann_json CONFIG REQUIRED)

set(SRC_FILES
    sensor_reader.cpp
    srf08.cpp
)
add_library(sensor_reader ${SRC_FILES})
target_include_directories(sensor_reader PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(sensor_reader nlohmann_json ${Boost_LIBRARIES})

if(BUILD_TESTS)
    add_gtest(test_sensor_reader test/test_sensor_reader.cpp)
    target_link_libraries(test_sensor_reader sensor_reader)
endif()
