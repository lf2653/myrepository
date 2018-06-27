#ifndef SOCKETCAN_BRIDGE_SPEEDINFO_H
#define SOCKETCAN_BRIDGE_SPEEDINFO_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <geometry_msgs/Point32.h>
#include <socketcan_bridge/data.h>

#include <cmath>

namespace socketcan_bridge
{

   class SpeedinfoNode
   {
      public:

         SpeedinfoNode();

         void callback(const geometry_msgs::Point32 & input);

         bool is_integer(float dec);

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

   float yaw;

      class YawinfoNode
   {
      public:

         YawinfoNode();

         void callback(const geometry_msgs::Point32 & input);

         bool is_integer(float dec);

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

};  // namespace socketcan_bridge

#endif 