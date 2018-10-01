set(CONFIG_SRCS
    config.cpp
    a4wd2/motor_controller/init.cpp
)

add_library(a4wd2_config ${CONFIG_SRCS})
target_include_directories(a4wd2_config PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(a4wd2_config cxxopts::cxxopts spdlog::spdlog roboclaw::roboclaw)
