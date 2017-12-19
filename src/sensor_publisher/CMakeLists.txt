set(SRC_FILES
    sensor_publisher.cpp
)

message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")

add_executable(sensor_publisher ${SRC_FILES})
target_link_libraries(sensor_publisher sensor_reader ${catkin_LIBRARIES})
target_include_directories(sensor_publisher PRIVATE ${catkin_INCLUDE_DIRS})
