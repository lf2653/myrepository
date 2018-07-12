#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <ros/ros.h>

#include "radar_input.h"


int main(int argc, char **argv) {

  //Initiate ROS
  ros::init(argc, argv, "extendedkf");

  radar_input RadarinfoObject1;

  ros::spin();
  
  return 0;
}
