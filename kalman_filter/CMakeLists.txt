project(kalman_filter)

cmake_minimum_required (VERSION 3.5)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

add_compile_options(-std=c++11)


find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    pb_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    std_msgs
    pb_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(FusionEKF
  src/FusionEKF.cpp
)
target_link_libraries(FusionEKF
  ${catkin_LIBRARIES}
)
add_dependencies(FusionEKF
  ${catkin_EXPORTED_TARGETS}
)
add_library(kalman_filter
  src/kalman_filter.cpp
)
target_link_libraries(kalman_filter
  ${catkin_LIBRARIES}
)
add_dependencies(kalman_filter
  ${catkin_EXPORTED_TARGETS}
)
add_library(tools
  src/tools.cpp
)
target_link_libraries(tools
  ${catkin_LIBRARIES}
)
add_dependencies(tools
  ${catkin_EXPORTED_TARGETS}
)
add_library(radar_input
  src/radar_input.cpp
)
target_link_libraries(radar_input
  ${catkin_LIBRARIES} 
  FusionEKF 
  kalman_filter 
  tools
)
add_dependencies(radar_input
  ${catkin_EXPORTED_TARGETS}
)


add_executable(ExtendedKF
  src/main.cpp

)
target_link_libraries(ExtendedKF
  FusionEKF
  kalman_filter
  tools
  radar_input
  ${catkin_LIBRARIES}
)

