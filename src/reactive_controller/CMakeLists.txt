set(SRCS
    mrpt_nav_interface.cpp
    laser_scan_provider.cpp
    odometry_provider.cpp
    reactive_controller.cpp
)

add_executable(reactive_controller ${SRCS})
target_link_libraries(reactive_controller a4wd2_config roboclaw::roboclaw cxxopts::cxxopts
                      ${catkin_LIBRARIES} Eigen3::Eigen mrpt-kinematics mrpt-obs mrpt-maps mrpt-system
                      mrpt-nav mrpt-math mrpt-config)

install(TARGETS reactive_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
