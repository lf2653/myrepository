#ifndef SOCKETCAN_BRIDGE_DECODE_CLUSTER_H
#define SOCKETCAN_BRIDGE_DECODE_CLUSTER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>
#include <pb_msgs/ClusterRadar.h>
#include <pb_msgs/ClusterList.h>
#include <socketcan_bridge/data.h>

#include <cmath>

namespace socketcan_bridge
{

   class ListCanMessageClusterNode
   {
      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

      public:

         ListCanMessageClusterNode();

         void callback(const can_msgs::Frame & input);

   };//End of class

   class DecodeCanMessageClusterNode
   {

      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

      public:

         DecodeCanMessageClusterNode();

         void callback(const can_msgs::Frame & input);

   };//End of class 

};  // namespace socketcan_bridge


#endif  