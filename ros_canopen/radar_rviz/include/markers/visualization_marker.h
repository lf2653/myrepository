#ifndef MARKERS_VISUALIZATION_MARKERS_H
#define MARKERS_VISUALIZATION_MARKERS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pb_msgs/ContiRadar.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>

namespace markers
{

   class MarkerClassDecoded
   {
      public:

         MarkerClassDecoded();

         void callback(const pb_msgs::ContiRadar & input);

      private:
         
         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

   class MarkerClassFiltered
   {
      public:

         MarkerClassFiltered();

         void callback(const pb_msgs::ContiRadar & input);

      private:
         
         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

};  // namespace markers

#endif