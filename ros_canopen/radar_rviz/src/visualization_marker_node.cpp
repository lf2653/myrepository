#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pb_msgs/ContiRadar.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <markers/visualization_marker.h>

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "visualization_marker_node");

  	//Create an object of class MarkerClassDecoded that will take care of everything
  	markers::MarkerClassDecoded MarkerObject1;

  	//Create an object of class MarkerClassFiltered that will take care of everything
  	//markers::MarkerClassFiltered MarkerObject2;

  	ros::spin();

  	return 0;
}
