#include <ros/ros.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>
#include <pb_msgs/ClusterRadar.h>
#include <pb_msgs/ClusterList.h>
#include <socketcan_bridge/data.h>
#include <socketcan_bridge/const_vars.h>
#include <socketcan_bridge/decode_cluster.h>

#include <iostream>
#include <cmath>

namespace socketcan_bridge
{

   ListCanMessageClusterNode::ListCanMessageClusterNode()
   {
      //Topic you want to publish
      pub_ = n_.advertise<pb_msgs::ClusterList>("/cluster_list_messages", 50);

      //Topic you want to subscribe
      sub_ = n_.subscribe("/received_messages", 50, &ListCanMessageClusterNode::callback, this);
   }

   void ListCanMessageClusterNode::callback(const can_msgs::Frame & input)
   {
      pb_msgs::ClusterList output1;
      //.... do something with the input and generate the output...

      if (input.id==1536) // 600 hex in dec
      {
         output1.nof_targetsnear = input.data[0];
         output1.nof_targetsfar = input.data[1];

         Byte b2;
         Byte b3;
         
         b2.byte=input.data[2];
         b3.byte=input.data[3];

         output1.meas_counter = pow(2,0)*b3.bit1+pow(2,1)*b3.bit2+pow(2,2)*b3.bit3+pow(2,3)*b3.bit4+pow(2,4)*b3.bit5+pow(2,5)*b3.bit6+pow(2,6)*b3.bit7+pow(2,7)*b3.bit8+pow(2,8)*b2.bit1+pow(2,9)*b2.bit2+pow(2,10)*b2.bit3+pow(2,11)*b2.bit4+pow(2,12)*b2.bit5+pow(2,13)*b2.bit6+pow(2,14)*b2.bit7+pow(2,15)*b2.bit8;

         output1.interface_version=input.data[3];

         pub_.publish(output1); 
      }
   }

   DecodeCanMessageClusterNode::DecodeCanMessageClusterNode()
   {
      //Topic you want to publish
      pub_ = n_.advertise<pb_msgs::ClusterRadar>("/cluster_decoded_messages", 50);

      //Topic you want to subscribe
      sub_ = n_.subscribe("/received_messages", 50, &DecodeCanMessageClusterNode::callback, this);
   }


   void DecodeCanMessageClusterNode::callback(const can_msgs::Frame & input)
   {
      pb_msgs::ClusterRadar output2;

      //.... do something with the input and generate the output...

      if (input.id==1793) // 701 hex in dec
      {
         output2.target_id=input.data[0];

         Byte b1;
         Byte b2;
         Byte b3;
         Byte b4;
         Byte b5;
         Byte b6;

         b1.byte=input.data[1];
         b2.byte=input.data[2];
         b3.byte=input.data[3];
         b4.byte=input.data[4];
         b5.byte=input.data[5];
         b6.byte=input.data[6];

         output2.longitude_dist = TARGET_DIST_LONG_MIN + TARGET_DIST_RES*(pow(2,0)*b2.bit4+pow(2,1)*b2.bit5+pow(2,2)*b2.bit6+pow(2,3)*b2.bit7+pow(2,4)*b2.bit8+pow(2,5)*b1.bit1+pow(2,6)*b1.bit2+pow(2,7)*b1.bit3+pow(2,8)*b1.bit4+pow(2,9)*b1.bit5+pow(2,10)*b1.bit6+pow(2,11)*b1.bit7+pow(2,12)*b1.bit8);

         output2.lateral_dist = TARGET_DIST_LAT_MIN + TARGET_DIST_RES*(pow(2,0)*b3.bit1+pow(2,1)*b3.bit2+pow(2,2)*b3.bit3+pow(2,3)*b3.bit4+pow(2,4)*b3.bit5+pow(2,5)*b3.bit6+pow(2,6)*b3.bit7+pow(2,7)*b3.bit8+pow(2,8)*b2.bit1+pow(2,9)*b2.bit2);

         output2.longitude_vel = TARGET_VREL_LONG_MIN + TARGET_VREL_RES*(pow(2,0)*b5.bit7+pow(2,1)*b5.bit8+pow(2,2)*b4.bit1+pow(2,3)*b4.bit2+pow(2,4)*b4.bit3+pow(2,5)*b4.bit4+pow(2,6)*b4.bit5+pow(2,7)*b4.bit6+pow(2,8)*b4.bit7+pow(2,9)*b4.bit8);

         output2.lateral_vel = TARGET_VREL_LAT_MIN + TARGET_VREL_RES*(pow(2,0)*b6.bit6+pow(2,1)*b6.bit7+pow(2,2)*b6.bit8+pow(2,3)*b5.bit1+pow(2,4)*b5.bit2+pow(2,5)*b5.bit3+pow(2,6)*b5.bit4+pow(2,7)*b5.bit5+pow(2,8)*b5.bit6);

         output2.rcs = TARGET_RCS_MIN + TARGET_RCS_RES*input.data[7];

         pub_.publish(output2);
      } 

   }

};  // namespace socketcan_bridge
