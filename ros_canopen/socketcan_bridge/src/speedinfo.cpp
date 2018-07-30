#include <ros/ros.h>
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <sensor_msgs/Imu.h>
#include <socketcan_bridge/data.h>
#include <socketcan_bridge/const_vars.h>
#include <socketcan_bridge/speedinfo.h>

#include <cmath>

namespace socketcan_bridge
{

   SpeedinfoNode::SpeedinfoNode()
      {
         //Topic you want to publish
         pub_ = n_.advertise<can_msgs::Frame>("/sent_messages", 10);

         //Topic you want to subscribe
         sub_ = n_.subscribe("/imu_raw", 10, &SpeedinfoNode::callback, this);
      }

   void SpeedinfoNode::callback(const sensor_msgs::Imu & input)
      {
         can_msgs::Frame output;
         //.... do something with the input and generate the output...

         output.header.stamp = ros::Time::now();
         output.id = 768; //Decimal value of id 300
         output.dlc = 2;

         float time = output.header.stamp.sec*1000000000 + output.header.stamp.nsec;

         float speedis = input.linear_acceleration.x / (time - time_prev);
         float roundspeedis = floor(50*speedis + 0.5) / 50; //Converting to speed resolution
         
         float nspeed = (roundspeedis - RADAR_SPEED_MIN) / RADAR_SPEED_RES;
         int i = 1;
         Byte b1;
         Byte b2;
         int nspeedbits1[8];
         int nspeedbits2[8];

         while (i <= 8)
         {
            nspeed /= 2;

            if (SpeedinfoNode::is_integer(nspeed) == 0)
            {
               nspeed -= 0.5;
               nspeedbits2[i] = 1;
               i++;
            }
            else
            {
               nspeedbits2[i] = 0;
               i++;
            }
         }

         i=1;

         while (i <= 5)
         {
            nspeed /= 2;

            if (SpeedinfoNode::is_integer(nspeed) == 0)
            {
               nspeed -= 0.5;
               nspeedbits1[i] = 1;
               i++;
            }
            else
            {
               nspeedbits1[i] = 0;
               i++;
            }
         }

         if (nspeed > 0)
         {
            nspeedbits1[7] = 0;
            nspeedbits1[8] = 1;
         }
         else if (nspeed == 0)
         {
            nspeedbits1[7] = nspeedbits1[8] = 0;
         }

         b1.bit1 = nspeedbits1[1];
         b1.bit2 = nspeedbits1[2];
         b1.bit3 = nspeedbits1[3];
         b1.bit4 = nspeedbits1[4];
         b1.bit5 = nspeedbits1[5];
         b1.bit6 = 0;
         b1.bit7 = nspeedbits1[7];
         b1.bit8 = nspeedbits1[8];

         b2.bit1 = nspeedbits2[1];
         b2.bit2 = nspeedbits2[2];
         b2.bit3 = nspeedbits2[3];
         b2.bit4 = nspeedbits2[4];
         b2.bit5 = nspeedbits2[5];
         b2.bit6 = nspeedbits2[6];
         b2.bit7 = nspeedbits2[7];
         b2.bit8 = nspeedbits2[8];

         output.data[0]= b1.byte;
         output.data[1]= b2.byte;

         time_prev = time;

         pub_.publish(output);
      }

   bool SpeedinfoNode::is_integer(float dec)
      {
         return std::floor(dec) == dec;
      }

   YawinfoNode::YawinfoNode()
      {
         //Topic you want to publish
         pub_ = n_.advertise<can_msgs::Frame>("/sent_messages", 10);

         //Topic you want to subscribe
         sub_ = n_.subscribe("/imu_raw", 10, &YawinfoNode::callback, this);
      }

   void YawinfoNode::callback(const sensor_msgs::Imu & input)
      {
         can_msgs::Frame output;
         //.... do something with the input and generate the output...

         output.header.stamp = ros::Time::now();
         output.id = 769; //Decimal value of id 301
         output.dlc = 2;

         float yawis = input.angular_velocity.z * (180 / M_PI);

         float roundyaw = floor(100*yawis + 0.5) / 100; //Converting to yaw resolution
         
         float nspeed = (roundyaw - RADAR_YAW_RATE_MIN) / RADAR_YAW_RATE_RES ;
         int i = 1;
         Byte b1;
         Byte b2;
         int nspeedbits1[8];
         int nspeedbits2[8];

         while (i <= 8)
         {
            nspeed /= 2;

            if (YawinfoNode::is_integer(nspeed) == 0)
            {
               nspeed -= 0.5;
               nspeedbits2[i] = 1;
               i++;
            }
            else
            {
               nspeedbits2[i] = 0;
               i++;
            }
         }

         i=1;

         while (i <= 8)
         {
            nspeed /= 2;

            if (YawinfoNode::is_integer(nspeed) == 0)
            {
               nspeed -= 0.5;
               nspeedbits1[i] = 1;
               i++;
            }
            else
            {
               nspeedbits1[i] = 0;
               i++;
            }
         }

         b1.bit1 = nspeedbits1[1];
         b1.bit2 = nspeedbits1[2];
         b1.bit3 = nspeedbits1[3];
         b1.bit4 = nspeedbits1[4];
         b1.bit5 = nspeedbits1[5];
         b1.bit6 = nspeedbits1[6];
         b1.bit7 = nspeedbits1[7];
         b1.bit8 = nspeedbits1[8];

         b2.bit1 = nspeedbits2[1];
         b2.bit2 = nspeedbits2[2];
         b2.bit3 = nspeedbits2[3];
         b2.bit4 = nspeedbits2[4];
         b2.bit5 = nspeedbits2[5];
         b2.bit6 = nspeedbits2[6];
         b2.bit7 = nspeedbits2[7];
         b2.bit8 = nspeedbits2[8];

         output.data[0]= b1.byte;
         output.data[1]= b2.byte;

         pub_.publish(output);
      }

   bool YawinfoNode::is_integer(float dec)
      {
         return std::floor(dec) == dec;
      }

};  // namespace socketcan_bridge