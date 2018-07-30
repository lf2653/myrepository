#ifndef SOCKETCAN_BRIDGE_SPEEDINFO_H
#define SOCKETCAN_BRIDGE_SPEEDINFO_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <sensor_msgs/Imu.h>
#include <socketcan_bridge/data.h>

#include <cmath>

namespace socketcan_bridge
{

   float time_prev;
   
   class SpeedinfoNode
   {
      public:

         SpeedinfoNode();

         void callback(const sensor_msgs::Imu & input);

         bool is_integer(float dec);

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

   class YawinfoNode
   {
      public:

         YawinfoNode();

         void callback(const sensor_msgs::Imu & input);

         bool is_integer(float dec);

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

};  // namespace socketcan_bridge

#endif 