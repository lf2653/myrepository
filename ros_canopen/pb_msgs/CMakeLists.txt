cmake_minimum_required(VERSION 2.8.3)
project(pb_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    std_msgs
)

add_message_files(DIRECTORY msg
  FILES
    ContiRadar.msg
    ContiList.msg
    ClusterRadar.msg
    ClusterList.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)
