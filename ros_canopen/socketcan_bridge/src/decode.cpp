#include <ros/ros.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>
#include <pb_msgs/ContiRadar.h>
#include <pb_msgs/ContiList.h>
#include <socketcan_bridge/data.h>
#include <socketcan_bridge/const_vars.h>
#include <socketcan_bridge/decode.h>

#include <iostream>
#include <cmath>

namespace socketcan_bridge
{

   ListCanMessageNode::ListCanMessageNode()
   {
      //Topic you want to publish
      pub_ = n_.advertise<pb_msgs::ContiList>("/list_messages", 50);

      //Topic you want to subscribe
      sub_ = n_.subscribe("/received_messages", 50, &ListCanMessageNode::callback, this);
   }

   void ListCanMessageNode::callback(const can_msgs::Frame & input)
   {
      pb_msgs::ContiList output1;
      //.... do something with the input and generate the output...

      if (input.id==1546) 
      {
         output1.nof_objects=input.data[0];

         Byte b1;
         Byte b2;
         
         b1.byte=input.data[1];
         b2.byte=input.data[2];

         output1.meas_counter = pow(2,0)*b2.bit1+pow(2,1)*b2.bit2+pow(2,2)*b2.bit3+pow(2,3)*b2.bit4+pow(2,4)*b2.bit5+pow(2,5)*b2.bit6+pow(2,6)*b2.bit7+pow(2,7)*b2.bit8+pow(2,8)*b1.bit1+pow(2,9)*b1.bit2+pow(2,10)*b1.bit3+pow(2,11)*b1.bit4+pow(2,12)*b1.bit5+pow(2,13)*b1.bit6+pow(2,14)*b1.bit7+pow(2,15)*b1.bit8;

         output1.interface_version=input.data[3];

         pub_.publish(output1); 
      }
   }

   DecodeCanMessageNode::DecodeCanMessageNode()
   {
      //Topic you want to publish
      pub_ = n_.advertise<pb_msgs::ContiRadar>("/decoded_messages", 100);

      //Topic you want to subscribe
      sub_ = n_.subscribe("/received_messages", 100, &DecodeCanMessageNode::callback, this);
   }


   void DecodeCanMessageNode::callback(const can_msgs::Frame & input)
   {
      pb_msgs::ContiRadar output2;


      //.... do something with the input and generate the output...

      if (input.id==1547) 
      {
         output2.obstacle_id=input.data[0];

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

         output2.longitude_dist = OBJECT_DIST_LONG_MIN+OBJECT_DIST_RES*(pow(2,0)*b2.bit4+pow(2,1)*b2.bit5+pow(2,2)*b2.bit6+pow(2,3)*b2.bit7+pow(2,4)*b2.bit8+pow(2,5)*b1.bit1+pow(2,6)*b1.bit2+pow(2,7)*b1.bit3+pow(2,8)*b1.bit4+pow(2,9)*b1.bit5+pow(2,10)*b1.bit6+pow(2,11)*b1.bit7+pow(2,12)*b1.bit8);

         output2.lateral_dist = OBJECT_DIST_LAT_MIN+OBJECT_DIST_RES*(pow(2,0)*b3.bit1+pow(2,1)*b3.bit2+pow(2,2)*b3.bit3+pow(2,3)*b3.bit4+pow(2,4)*b3.bit5+pow(2,5)*b3.bit6+pow(2,6)*b3.bit7+pow(2,7)*b3.bit8+pow(2,8)*b2.bit1+pow(2,9)*b2.bit2+pow(2,10)*b2.bit3);

         output2.longitude_vel = OBJECT_VREL_LONG_MIN+OBJECT_VREL_RES*(pow(2,0)*b5.bit7+pow(2,1)*b5.bit8+pow(2,2)*b4.bit1+pow(2,3)*b4.bit2+pow(2,4)*b4.bit3+pow(2,5)*b4.bit4+pow(2,6)*b4.bit5+pow(2,7)*b4.bit6+pow(2,8)*b4.bit7+pow(2,9)*b4.bit8);

         output2.lateral_vel = OBJECT_VREL_LAT_MIN+OBJECT_VREL_RES*(pow(2,0)*b6.bit6+pow(2,1)*b6.bit7+pow(2,2)*b6.bit8+pow(2,3)*b5.bit1+pow(2,4)*b5.bit2+pow(2,5)*b5.bit3+pow(2,6)*b5.bit4+pow(2,7)*b5.bit5+pow(2,8)*b5.bit6);

         output2.rcs=OBJECT_RCS_MIN+OBJECT_RCS_RES*input.data[7];

         obj[output2.obstacle_id].longitude_dist = output2.longitude_dist;
         obj[output2.obstacle_id].lateral_dist = output2.lateral_dist;
         obj[output2.obstacle_id].longitude_vel = output2.longitude_vel;
         obj[output2.obstacle_id].lateral_vel = output2.lateral_vel;
         obj[output2.obstacle_id].rcs = output2.rcs;

      } 

      if (input.id==1548)
      {
         output2.obstacle_id=input.data[0];

         Byte b6;
         b6.byte = input.data[6];

         if (b6.bit5==0) 
         {
            if (b6.bit4==0) 
            {
               if (b6.bit3==0) 
               {
                  output2.meas_state=0;
               }
               else 
               {
                  output2.meas_state=1;  
               }
            }
            else 
            {
               if (b6.bit1==0) 
               {
                  output2.meas_state=2;
               }
               else 
               {
                  output2.meas_state=3;
               }
            }
         }
         else 
         {
            if (b6.bit2==0) 
            {
               if (b6.bit1==0) 
               {
                  output2.meas_state=4;
               }
               else 
               {
                  output2.meas_state=5;  
               }
            } 
         } 

         obj2[output2.obstacle_id].meas_state = output2.meas_state;
      }

      if (input.id==1549) 
      {

         output2.header=input.header;
         output2.header.frame_id="radar";
         output2.obstacle_id=input.data[0];
         Byte b3;
         Byte b4;
         Byte b5;

         b3.byte=input.data[3];
         b4.byte=input.data[4];
         b5.byte=input.data[5];

         if (b3.bit3==0) 
         {
            if (b3.bit2==0) 
            {
               if (b3.bit1==0) 
               {
                  output2.obstacle_class="point";
               }
               else 
               {
                  output2.obstacle_class="car";  
               }
            }
            else 
            {
               if (b3.bit1==0) 
               {
                  output2.obstacle_class="truck";
               }
               else 
               {
                  output2.obstacle_class="pedestrian";
               }
            }
         }
         else 
         {
            if (b3.bit2==0) 
            {
               if (b3.bit1==0) 
               {
                  output2.obstacle_class="motorcycle";
               }
               else 
               {
                  output2.obstacle_class="bicycle";  
               }
            }
            else 
            {
               if (b3.bit1==0) 
               {
                  output2.obstacle_class="wide";
               }
               else 
               {
                  output2.obstacle_class="reserved";
               }
            }    
         } 

         output2.orientation_angle = OBJECT_ORIENTATION_ANGEL_MIN+OBJECT_ORIENTATION_ANGEL_RES*(pow(2,0)*b5.bit7+pow(2,1)*b5.bit8+pow(2,2)*b4.bit1+pow(2,3)*b4.bit2+pow(2,4)*b4.bit3+pow(2,5)*b4.bit4+pow(2,6)*b4.bit5+pow(2,7)*b4.bit6+pow(2,8)*b4.bit7+pow(2,9)*b4.bit8);

         output2.length=OBJECT_LENGTH_RES*input.data[6];

         output2.width=OBJECT_WIDTH_RES*input.data[7];

         output2.longitude_dist = obj[output2.obstacle_id].longitude_dist;
         output2.lateral_dist = obj[output2.obstacle_id].lateral_dist;
         output2.longitude_vel = obj[output2.obstacle_id].longitude_vel;
         output2.lateral_vel = obj[output2.obstacle_id].lateral_vel;
         output2.rcs = obj[output2.obstacle_id].rcs;
         output2.meas_state = obj2[output2.obstacle_id].meas_state;

         pub_.publish(output2);

      } 
   }

};  // namespace socketcan_bridge
