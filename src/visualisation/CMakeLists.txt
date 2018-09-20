if(CMAKE_CROSSCOMPILING)
    return()
endif()

find_package(MRPT CONFIG REQUIRED)

add_executable(a4wd2_visualisation a4wd2_visualisation.cpp)
target_link_libraries(a4wd2_visualisation
    a4wd2_config
    cxxopts::cxxopts
    mrpt-poses
    ${catkin_LIBRARIES}
)
add_dependencies(a4wd2_visualisation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
