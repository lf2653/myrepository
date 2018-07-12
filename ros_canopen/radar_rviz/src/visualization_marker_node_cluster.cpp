#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pb_msgs/ClusterRadar.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <markers/visualization_marker_cluster.h>

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "visualization_marker_node_cluster");

  	//Create an object of class MarkerClassClusterDecoded that will take care of everything
  	markers::MarkerClassClusterDecoded MarkerTarget1;

  	//Create an object of class MarkerClassClusterFiltered that will take care of everything
  	//markers::MarkerClassClusterFiltered MarkerTarget2;

  	ros::spin();

  	return 0;
}
