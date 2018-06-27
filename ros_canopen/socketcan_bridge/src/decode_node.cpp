#include <ros/ros.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>
#include <pb_msgs/ContiRadar.h>
#include <pb_msgs/ContiList.h>
#include <socketcan_bridge/decode.h>

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "decode_node");

  //Create an object of class DecodeCanMessageNode that will take care of everything
  socketcan_bridge::DecodeCanMessageNode SAPObject1;

  //Create an object of class ListCanMessageNode that will take care of everything
  socketcan_bridge::ListCanMessageNode SAPObject2;

  ros::spin();

  return 0;
}

