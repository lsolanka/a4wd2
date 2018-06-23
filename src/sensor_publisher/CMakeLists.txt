set(SRC_FILES
    sensor_publisher.cpp
)

add_executable(sensor_publisher ${SRC_FILES})
target_link_libraries(sensor_publisher sensor_reader ${catkin_LIBRARIES})
target_include_directories(sensor_publisher PRIVATE ${catkin_INCLUDE_DIRS})

install(TARGETS sensor_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
