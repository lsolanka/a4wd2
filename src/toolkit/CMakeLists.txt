set(SRCS
    MotorController.cpp
    odometry_provider.cpp
    waypoint_generators.cpp
)

add_library(toolkit ${SRCS})
target_link_libraries(toolkit roboclaw::roboclaw ${catkin_LIBRARIES} mrpt-nav)
target_include_directories(toolkit PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
