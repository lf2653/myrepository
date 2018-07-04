# myrepository

Implementation of Continental ARS 408 Radar in Ubuntu 16.04. Visualization of objects in rviz.


# Packages

joint_state_publisher
kalman_filter
socketcan_bridge
socketcan_interface
can_msgs
pb_msgs
radar_rviz
urdf_radar

# Build and Run

-> mkdir folder
-> mkdir folder/src
-> cd folder/src
-> git clone https://github.com/sergiocasaspastor/myrepository.git
-> cd ..
-> catkin build
-> source devel/setup.bash
-> roslaunch socketcan_bridge radar.launch



