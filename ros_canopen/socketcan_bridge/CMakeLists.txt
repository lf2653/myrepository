cmake_minimum_required(VERSION 2.8.3)
project(socketcan_bridge)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    can_msgs
    roscpp
    socketcan_interface
    pb_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    socketcan_to_topic
    topic_to_socketcan
    decode
    decode_cluster
    warning
    speedinfo
    configuration
  CATKIN_DEPENDS
    can_msgs
    roscpp
    socketcan_interface
    pb_msgs
    geometry_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# socketcan_to_topic
add_library(socketcan_to_topic
  src/socketcan_to_topic.cpp
)
target_link_libraries(socketcan_to_topic
  ${catkin_LIBRARIES}
)
add_dependencies(socketcan_to_topic
  ${catkin_EXPORTED_TARGETS}
)

# topic_to_socketcan
add_library(topic_to_socketcan
  src/topic_to_socketcan.cpp
)
target_link_libraries(topic_to_socketcan
  ${catkin_LIBRARIES}
)
add_dependencies(topic_to_socketcan
  ${catkin_EXPORTED_TARGETS}
)

# decode 
add_library(decode
  src/decode.cpp
)
target_link_libraries(decode
  ${catkin_LIBRARIES}
)
add_dependencies(decode
  ${catkin_EXPORTED_TARGETS}
)

# decode_cluster
add_library(decode_cluster
  src/decode_cluster.cpp
)
target_link_libraries(decode_cluster
  ${catkin_LIBRARIES}
)
add_dependencies(decode_cluster
  ${catkin_EXPORTED_TARGETS}
)

# warning
add_library(warning
  src/warning.cpp
)
target_link_libraries(warning
  ${catkin_LIBRARIES}
)
add_dependencies(warning
  ${catkin_EXPORTED_TARGETS}
)

# speedinfo
add_library(speedinfo
  src/speedinfo.cpp
)
target_link_libraries(speedinfo
  ${catkin_LIBRARIES}
)
add_dependencies(speedinfo
  ${catkin_EXPORTED_TARGETS}
)

# configuration
add_library(configuration
  src/configuration.cpp
)
target_link_libraries(configuration
  ${catkin_LIBRARIES}
)
add_dependencies(configuration
  ${catkin_EXPORTED_TARGETS}
)

# socketcan_to_topic_node
add_executable(socketcan_to_topic_node
  src/socketcan_to_topic_node.cpp
)
target_link_libraries(socketcan_to_topic_node
  socketcan_to_topic
  ${catkin_LIBRARIES}
)

# topic_to_socketcan_node
add_executable(topic_to_socketcan_node
  src/topic_to_socketcan_node.cpp

)
target_link_libraries(topic_to_socketcan_node
  topic_to_socketcan
  ${catkin_LIBRARIES}
)

# decode_node
add_executable(decode_node
  src/decode_node.cpp
)
target_link_libraries(decode_node
  decode
  ${catkin_LIBRARIES}
)

# decode_node_cluster
add_executable(decode_node_cluster
  src/decode_node_cluster.cpp
)
target_link_libraries(decode_node_cluster
  decode_cluster
  ${catkin_LIBRARIES}
)

# warning_node
add_executable(warning_node
  src/warning_node.cpp
)
target_link_libraries(warning_node
  warning
  ${catkin_LIBRARIES}
)

# speedinfo_node
add_executable(speedinfo_node
  src/speedinfo_node.cpp

)
target_link_libraries(speedinfo_node
  speedinfo
  ${catkin_LIBRARIES}
)


# configuration_node
add_executable(configuration_node
  src/configuration_node.cpp

)
target_link_libraries(configuration_node
  configuration
  ${catkin_LIBRARIES}
)

# socketcan_bridge_node
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  topic_to_socketcan
  socketcan_to_topic
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}_node
    socketcan_to_topic
    socketcan_to_topic_node
    topic_to_socketcan
    topic_to_socketcan_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)

  roslint_cpp()
  #for use with catkin_make roslint_socketcan_bridge

  # unit test for the can_msgs::Frame and can::Frame types.
  catkin_add_gtest(test_conversion
    test/test_conversion.cpp
  )
  target_link_libraries(test_conversion
    topic_to_socketcan
    socketcan_to_topic
    ${catkin_LIBRARIES}
  )


  add_rostest_gtest(test_to_socketcan
    test/to_socketcan.test
    test/to_socketcan_test.cpp
  )
  target_link_libraries(test_to_socketcan
    topic_to_socketcan
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(test_to_topic
    test/to_topic.test
    test/to_topic_test.cpp
  )
  target_link_libraries(test_to_topic
    socketcan_to_topic
    topic_to_socketcan
    ${catkin_LIBRARIES}
  )

endif()
