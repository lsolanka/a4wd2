find_package(roboclaw CONFIG REQUIRED)

add_executable(ros_controller ros_controller.cpp)
target_link_libraries(ros_controller a4wd2_config roboclaw::roboclaw cxxopts::cxxopts
                                     ${catkin_LIBRARIES})
add_dependencies(ros_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(NOT CMAKE_CROSSCOMPILING)
    hunter_add_package(OpenCV)
    find_package(OpenCV CONFIG REQUIRED)
    find_package(VTK CONFIG REQUIRED)

    add_executable(key_controller key_controller.cpp)
    target_link_libraries(key_controller ${OpenCV_LIBS} ${catkin_LIBRARIES})
    install(TARGETS key_controller
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif()

install(TARGETS ros_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
