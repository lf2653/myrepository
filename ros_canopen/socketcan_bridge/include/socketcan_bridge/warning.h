#ifndef SOCKETCAN_BRIDGE_WARNING_H
#define SOCKETCAN_BRIDGE_WARNING_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <can_msgs/Frame.h>

#include <cmath>

namespace socketcan_bridge
{

   class WarningNode
   {
      private:

         ros::NodeHandle n_; 
         ros::Publisher pub_;
         ros::Subscriber sub_;

      public:

         WarningNode();

         void callback(const can_msgs::Frame & input);

   };//End of class

};  // namespace socketcan_bridge


#endif  