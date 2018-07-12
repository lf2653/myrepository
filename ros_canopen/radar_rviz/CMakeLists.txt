cmake_minimum_required(VERSION 2.8.3)
project(radar_rviz)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    visualization_msgs
    pb_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    visualization_marker
    visualization_marker_cluster
  CATKIN_DEPENDS
    roscpp
    std_msgs
    visualization_msgs
    pb_msgs
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Objects visualization 

add_library(visualization_marker
  src/visualization_marker.cpp
)
target_link_libraries(visualization_marker
  ${catkin_LIBRARIES}
)
add_dependencies(visualization_marker
  ${catkin_EXPORTED_TARGETS}
)

add_executable(visualization_marker_node
  src/visualization_marker_node.cpp

)
target_link_libraries(visualization_marker_node
  visualization_marker
  ${catkin_LIBRARIES}
)

# Clusters visualization

add_library(visualization_marker_cluster
  src/visualization_marker_cluster.cpp
)
target_link_libraries(visualization_marker_cluster
  ${catkin_LIBRARIES}
)
add_dependencies(visualization_marker_cluster
  ${catkin_EXPORTED_TARGETS}
)

add_executable(visualization_marker_node_cluster
  src/visualization_marker_node_cluster.cpp

)
target_link_libraries(visualization_marker_node_cluster
  visualization_marker_cluster
  ${catkin_LIBRARIES}
)