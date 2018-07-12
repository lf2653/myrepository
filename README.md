Implementation of Continental ARS 408 Radar in Ubuntu 16.04. Visualization of objects in rviz.

# Nodes

- base_link_to_radar
- decode_node
- decode_node_cluster
- extendedkf
- joint_state_publisher
- rosout
- rviz
- socketcan_bridge_node
- speedinfo_node
- visualization_marker_node
- visualization_marker_node_cluster

# Topics

- /clicked_point
- /cluster_decoded_messages
- /cluster_list_messages
- /control/arduino_output_steering
- /control/arduino_output_throttle
- /decoded_messages
- /filtered_messages
- /initialpose
- /joint_states
- /list_messages
- /move_base_simple/goal
- /received_messages
- /rosout
- /rosout_agg
- /sent_messages
- /tf
- /tf_static
- /visualization_marker
- /visualization_marker_array
- /visualization_marker_cluster

# Build and Run
```bash
mkdir folder
mkdir folder/src
cd folder/src
git clone https://github.com/sergiocasaspastor/myrepository.git
cd ..
catkin build
source devel/setup.bash
roslaunch socketcan_bridge radar.launch
```

# Radar setup

There is a command called cansend, that belongs to can-utils, used for sending configuration messages to the radar. Here are some messages proposed, but other messages can be sent (watch the manual ARS40X_Technical_Documentation_V 1.8_18.10.2017 inside documentation folder).

Installation of can-utils
```bash
sudo apt-get install can-utils
candump can0 // Watch the raw data received once the peak CAN bus is installed and connected to Radar
```

Configuration messages to choose between cluster detection or object detection
```bash
cansend can0 200#F8000000089C0000 // Objects detection with all extended properties
cansend can0 200#F8000000109C0000 // Clusters detection with all extended properties
```

Configuration messages for applying different filters.
```bash
cansend can0 202#8C0000012C // Maximum distance of objects detected 30 meters
cansend can0 202#AE06800FFF // Minimum value of object RCS -10 dBm2
cansend can0 202#C600030007 // Minimum value of objects probability of existence 75%
```
