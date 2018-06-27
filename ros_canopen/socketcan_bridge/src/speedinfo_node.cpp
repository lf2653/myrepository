#include <ros/ros.h>
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <geometry_msgs/Point32.h>
#include <socketcan_bridge/speedinfo.h>

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "speedinfo_node");

  //Create an object of class SpeedinfoNode that will take care of everything
  socketcan_bridge::SpeedinfoNode SpeedObject;

  //Create an object of class YawinfoNode that will take care of everything
  socketcan_bridge::YawinfoNode YawObject;
  
  ros::spin();

  return 0;
}

