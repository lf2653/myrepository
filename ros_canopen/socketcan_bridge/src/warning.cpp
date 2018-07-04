#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <can_msgs/Frame.h>
#include <socketcan_bridge/data.h>
#include <socketcan_bridge/const_vars.h>
#include <socketcan_bridge/warning.h>

#include <iostream>
#include <cmath>

namespace socketcan_bridge
{

   WarningNode::WarningNode()
   {
      //Topic you want to publish
      pub_ = n_.advertise<std_msgs::Int32>("/warning_messages", 10);

      //Topic you want to subscribe
      sub_ = n_.subscribe("/received_messages", 10, &WarningNode::callback, this);
   }

   void WarningNode::callback(const can_msgs::Frame & input)
   {
      std_msgs::Int32 output;
      //.... do something with the input and generate the output...

      if (input.id==1550) 
      {
         output.data = 1;

         pub_.publish(output); 
      }
   }

};  // namespace socketcan_bridge