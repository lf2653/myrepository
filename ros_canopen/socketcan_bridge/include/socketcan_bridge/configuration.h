#ifndef SOCKETCAN_BRIDGE_CONFIGURATION_H
#define SOCKETCAN_BRIDGE_CONFIGURATION_H

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <socketcan_bridge/data.h>

#include <cmath>

namespace socketcan_bridge
{

   class ConfigurationNode
   {
      public:

         ConfigurationNode();
         void callback(const can_msgs::Frame & input);
         bool is_integer(float dec);

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

   class FilterConfigurationNode
   {
      public:

         FilterConfigurationNode();
         void callback(const can_msgs::Frame & input);
         bool is_integer(float dec);

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

   class CollisionConfigurationNode
   {
      public:

         CollisionConfigurationNode();
         void callback(const can_msgs::Frame & input);
         bool is_integer(float dec);

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

   class CollisionRegionConfigurationNode
   {
      public:

         CollisionRegionConfigurationNode();
         void callback(const can_msgs::Frame & input);
         bool is_integer(float dec);

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

   };//End of class

};  // namespace socketcan_bridge

#endif 