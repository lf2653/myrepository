#include <ros/ros.h>
#include "std_msgs/String.h"
#include <can_msgs/Frame.h>
#include <socketcan_bridge/data.h>

#include <socketcan_bridge/configuration.h>
#include <socketcan_bridge/configuration_vars.h>

#include <cmath>

namespace socketcan_bridge
{

   ConfigurationNode::ConfigurationNode()
      {
         //Topic you want to publish
         pub_ = n_.advertise<can_msgs::Frame>("/sent_messages", 1);

        //Topic you want to subscribe
         sub_ = n_.subscribe("/received_messages", 1, &ConfigurationNode::callback, this);
      }

      void ConfigurationNode::callback(const can_msgs::Frame & input)
      {
         can_msgs::Frame output;
         //.... do something with the input and generate the output...

         output.header.stamp = ros::Time::now();
         output.id = 512; //Decimal value of id 200
         output.dlc = 8;

         Byte b1, b2, b3, b4, b5, b6, b7, b8;
         int bits1[8], bits2[8], bits3[8], bits4[8], bits5[8], bits6[8], bits7[8], bits8[8];
         int i = 1;

         b1.bit1 = RADARCFG_MAXDISTANCE_VALID;
         b1.bit2 = RADARCFG_SENSORID_VALID;
         b1.bit3 = RADARCFG_RADARPOWER_VALID;
         b1.bit4 = RADARCFG_OUTPUTTYPE_VALID;
         b1.bit5 = RADARCFG_SENDQUALITY_VALID;
         b1.bit6 = RADARCFG_SENDEXTINFO_VALID;
         b1.bit7 = RADARCFG_SORTINDEX_VALID;
         b1.bit8 = RADARCFG_STOREINNVM_VALID;

         double radarcfg_maxdistance = RADARCFG_MAXDISTANCE;

         while (i <= 2)
         {
            radarcfg_maxdistance /= 2;

            if (ConfigurationNode::is_integer(radarcfg_maxdistance) == 0)
            {
               radarcfg_maxdistance -= 0.5;
               bits2[i] = 1;
               i++;
            }
            else
            {
               bits2[i] = 0;
               i++;
            }
         }

         i=1;

         while (i <= 8)
         {
            radarcfg_maxdistance /= 2;

            if (ConfigurationNode::is_integer(radarcfg_maxdistance) == 0)
            {
               radarcfg_maxdistance -= 0.5;
               bits1[i] = 1;
               i++;
            }
            else
            {
               bits1[i] = 0;
               i++;
            }
         }

         b2.bit1 = bits1[1];
         b2.bit2 = bits1[2];
         b2.bit3 = bits1[3];
         b2.bit4 = bits1[4];
         b2.bit5 = bits1[5];
         b2.bit6 = bits1[6];
         b2.bit7 = bits1[7];
         b2.bit8 = bits1[8];

         b3.bit1 = 0;
         b3.bit2 = 0;
         b3.bit3 = 0;
         b3.bit4 = 0;
         b3.bit5 = 0;
         b3.bit6 = 0;
         b3.bit7 = bits2[1];
         b3.bit8 = bits2[2];

         b4.bit1 = 0;
         b4.bit2 = 0;
         b4.bit3 = 0;
         b4.bit4 = 0;
         b4.bit5 = 0;
         b4.bit6 = 0;
         b4.bit7 = 0;
         b4.bit8 = 0;

         double radarcfg_sensorid = RADARCFG_SENSORID;
         double radarcfg_outputtype = RADARCFG_OUTPUTTYPE;
         double radarcfg_radarpower = RADARCFG_RADARPOWER;

         i = 1;
         while (i <= 3)
         {
            radarcfg_sensorid /= 2;

            if (ConfigurationNode::is_integer(radarcfg_sensorid) == 0)
            {
               radarcfg_sensorid -= 0.5;
               bits4[i] = 1;
               i++;
            }
            else
            {
               bits4[i] = 0;
               i++;
            }
         }

         while (i <= 5)
         {
            radarcfg_outputtype /= 2;

            if (ConfigurationNode::is_integer(radarcfg_outputtype) == 0)
            {
               radarcfg_outputtype -= 0.5;
               bits4[i] = 1;
               i++;
            }
            else
            {
               bits4[i] = 0;
               i++;
            }
         }

         while (i <= 8)
         {
            radarcfg_radarpower /= 2;

            if (ConfigurationNode::is_integer(radarcfg_radarpower) == 0)
            {
               radarcfg_radarpower -= 0.5;
               bits4[i] = 1;
               i++;
            }
            else
            {
               bits4[i] = 0;
               i++;
            }
         }

         b5.bit1 = bits4[1];
         b5.bit2 = bits4[2];
         b5.bit3 = bits4[3];
         b5.bit4 = bits4[4];
         b5.bit5 = bits4[5];
         b5.bit6 = bits4[6];
         b5.bit7 = bits4[7];
         b5.bit8 = bits4[8];

         double radarcfg_ctrlrelay_valid = RADARCFG_CTRLRELAY_VALID;
         double radarcfg_ctrlrelay = RADARCFG_CTRLRELAY;
         double radarcfg_sendquality = RADARCFG_SENDQUALITY;
         double radarcfg_sendextinfo = RADARCFG_SENDEXTINFO;
         double radarcfg_sortindex = RADARCFG_SORTINDEX;
         double radarcfg_storeinnvm = RADARCFG_STOREINNVM;

         i = 1;
         while (i <= 1)
         {
            radarcfg_ctrlrelay_valid /= 2;

            if (ConfigurationNode::is_integer(radarcfg_ctrlrelay_valid) == 0)
            {
               radarcfg_ctrlrelay_valid -= 0.5;
               bits5[i] = 1;
               i++;
            }
            else
            {
               bits5[i] = 0;
               i++;
            }
         }

         while (i <= 2)
         {
            radarcfg_ctrlrelay /= 2;

            if (ConfigurationNode::is_integer(radarcfg_ctrlrelay) == 0)
            {
               radarcfg_ctrlrelay -= 0.5;
               bits5[i] = 1;
               i++;
            }
            else
            {
               bits5[i] = 0;
               i++;
            }
         }

         while (i <= 3)
         {
            radarcfg_sendquality /= 2;

            if (ConfigurationNode::is_integer(radarcfg_sendquality) == 0)
            {
               radarcfg_sendquality -= 0.5;
               bits5[i] = 1;
               i++;
            }
            else
            {
               bits5[i] = 0;
               i++;
            }
         }

         while (i <= 4)
         {
            radarcfg_sendextinfo /= 2;

            if (ConfigurationNode::is_integer(radarcfg_sendextinfo) == 0)
            {
               radarcfg_sendextinfo -= 0.5;
               bits5[i] = 1;
               i++;
            }
            else
            {
               bits5[i] = 0;
               i++;
            }
         }

         while (i <= 7)
         {
            radarcfg_sortindex /= 2;

            if (ConfigurationNode::is_integer(radarcfg_sortindex) == 0)
            {
               radarcfg_sortindex -= 0.5;
               bits5[i] = 1;
               i++;
            }
            else
            {
               bits5[i] = 0;
               i++;
            }
         }

         while (i <= 8)
         {
            radarcfg_storeinnvm /= 2;

            if (ConfigurationNode::is_integer(radarcfg_storeinnvm) == 0)
            {
               radarcfg_storeinnvm -= 0.5;
               bits5[i] = 1;
               i++;
            }
            else
            {
               bits5[i] = 0;
               i++;
            }
         }

         b6.bit1 = bits5[1];
         b6.bit2 = bits5[2];
         b6.bit3 = bits5[3];
         b6.bit4 = bits5[4];
         b6.bit5 = bits5[5];
         b6.bit6 = bits5[6];
         b6.bit7 = bits5[7];
         b6.bit8 = bits5[8];

         double radarcfg_rcs_threshold_valid = RADARCFG_RCS_THRESHOLD_VALID;
         double radarcfg_rcs_threshold = RADARCFG_RCS_THRESHOLD;

         i = 1;
         while (i <= 1)
         {
            radarcfg_rcs_threshold_valid /= 2;

            if (ConfigurationNode::is_integer(radarcfg_rcs_threshold_valid) == 0)
            {
               radarcfg_rcs_threshold_valid -= 0.5;
               bits6[i] = 1;
               i++;
            }
            else
            {
               bits6[i] = 0;
               i++;
            }
         }

         while (i <= 4)
         {
            radarcfg_rcs_threshold /= 2;

            if (ConfigurationNode::is_integer(radarcfg_rcs_threshold) == 0)
            {
               radarcfg_rcs_threshold -= 0.5;
               bits6[i] = 1;
               i++;
            }
            else
            {
               bits6[i] = 0;
               i++;
            }
         }

         b7.bit1 = bits6[1];
         b7.bit2 = bits6[2];
         b7.bit3 = bits6[3];
         b7.bit4 = bits6[4];
         b7.bit5 = 0;
         b7.bit6 = 0;
         b7.bit7 = 0;
         b7.bit8 = 0;

         b8.bit1 = 0;
         b8.bit2 = 0;
         b8.bit3 = 0;
         b8.bit4 = 0;
         b8.bit5 = 0;
         b8.bit6 = 0;
         b8.bit7 = 0;
         b8.bit8 = 0;

         output.data[0]= b1.byte;
         output.data[1]= b2.byte;
         output.data[2]= b3.byte;
         output.data[3]= b4.byte;
         output.data[4]= b5.byte;
         output.data[5]= b6.byte;
         output.data[6]= b7.byte;
         output.data[7]= b8.byte;

         pub_.publish(output);
      }

   bool ConfigurationNode::is_integer(float dec)
      {
         return std::floor(dec) == dec;
      }

   FilterConfigurationNode::FilterConfigurationNode()
      {
         //Topic you want to publish
         pub_ = n_.advertise<can_msgs::Frame>("/sent_messages", 1);

        //Topic you want to subscribe
         sub_ = n_.subscribe("/received_messages", 1, &FilterConfigurationNode::callback, this);
      }

      void FilterConfigurationNode::callback(const can_msgs::Frame & input)
      {
         can_msgs::Frame output;
         //.... do something with the input and generate the output...

         output.header.stamp = ros::Time::now();
         output.id = 514; //Decimal value of id 202
         output.dlc = 5;

         Byte b1, b2, b3, b4, b5;
         int bits1[8], bits2[8], bits3[8], bits4[8], bits5[8];

         b1.bit1 = 0;
         b1.bit2 = FILTERCFG_VALID;
         b1.bit3 = FILTERCFG_ACTIVE;

         int i = 4;
         double filtercfg_index = FILTERCFG_INDEX;
         double filtercfg_type = FILTERCFG_TYPE;

         while (i <= 7)
         {
            filtercfg_index /= 2;

            if (FilterConfigurationNode::is_integer(filtercfg_index) == 0)
            {
               filtercfg_index -= 0.5;
               bits1[i] = 1;
               i++;
            }
            else
            {
               bits1[i] = 0;
               i++;
            }
         }

         while (i <= 8)
         {
            filtercfg_type /= 2;

            if (FilterConfigurationNode::is_integer(filtercfg_type) == 0)
            {
               filtercfg_type -= 0.5;
               bits1[i] = 1;
               i++;
            }
            else
            {
               bits1[i] = 0;
               i++;
            }
         }

         b1.bit4 = bits1[4];
         b1.bit5 = bits1[5];
         b1.bit6 = bits1[6];
         b1.bit7 = bits1[7];
         b1.bit8 = bits1[8];

         double filtercfg_min, filtercfg_max, filtercfg_res;
         switch (FILTERCFG_INDEX){
         case 0:
            filtercfg_min = FILTERCFG_MIN_NOFOBJ / FILTERCFG_RES_NOFOBJ;
            filtercfg_max = FILTERCFG_MAX_NOFOBJ / FILTERCFG_RES_NOFOBJ;
            break;
         case 1:
            filtercfg_min = FILTERCFG_MIN_DISTANCE / FILTERCFG_RES_DISTANCE;
            filtercfg_max = FILTERCFG_MAX_DISTANCE / FILTERCFG_RES_DISTANCE;
            break;
         case 2:
            filtercfg_min = (FILTERCFG_MIN_AZIMUTH - FILTERCFG_MINIMUM_AZIMUTH) / FILTERCFG_RES_AZIMUTH;
            filtercfg_max = (FILTERCFG_MAX_AZIMUTH - FILTERCFG_MINIMUM_AZIMUTH) / FILTERCFG_RES_AZIMUTH;
            break;
         case 3:
            filtercfg_min = FILTERCFG_MIN_VRELONCOME / FILTERCFG_RES_VRELONCOME;
            filtercfg_max = FILTERCFG_MAX_VRELONCOME / FILTERCFG_RES_VRELONCOME;
            break;
         case 4:
            filtercfg_min = FILTERCFG_MIN_VRELDEPART / FILTERCFG_RES_VRELDEPART;
            filtercfg_max = FILTERCFG_MAX_VRELDEPART / FILTERCFG_RES_VRELDEPART;
            break;
         case 5:
            filtercfg_min = (FILTERCFG_MIN_RCS - FILTERCFG_MINIMUM_RCS) / FILTERCFG_RES_RCS;
            filtercfg_max = (FILTERCFG_MAX_RCS - FILTERCFG_MINIMUM_RCS) / FILTERCFG_RES_RCS;
            break; 
         case 6:
            filtercfg_min = FILTERCFG_MIN_LIFETIME / FILTERCFG_RES_LIFETIME;
            filtercfg_max = FILTERCFG_MAX_LIFETIME / FILTERCFG_RES_LIFETIME;
            break;
         case 7:
            filtercfg_min = FILTERCFG_MIN_SIZE / FILTERCFG_RES_SIZE;
            filtercfg_max = FILTERCFG_MAX_SIZE / FILTERCFG_RES_SIZE;
            break;
         case 8:
            filtercfg_min = FILTERCFG_MIN_PROBEXISTS / FILTERCFG_RES_PROBEXISTS;
            filtercfg_max = FILTERCFG_MAX_PROBEXISTS / FILTERCFG_RES_PROBEXISTS;
            break;
         case 9:
            filtercfg_min = (FILTERCFG_MIN_Y - FILTERCFG_MINIMUM_Y) / FILTERCFG_RES_Y;
            filtercfg_max = (FILTERCFG_MAX_Y - FILTERCFG_MINIMUM_Y) / FILTERCFG_RES_Y;
            break;
         case 10:
            filtercfg_min = (FILTERCFG_MIN_X - FILTERCFG_MINIMUM_X) / FILTERCFG_RES_X;
            filtercfg_max = (FILTERCFG_MAX_X - FILTERCFG_MINIMUM_X) / FILTERCFG_RES_X;
            break;
         case 11:
            filtercfg_min = FILTERCFG_MIN_VYRIGHTLEFT / FILTERCFG_RES_VYRIGHTLEFT;
            filtercfg_max = FILTERCFG_MAX_VYRIGHTLEFT / FILTERCFG_RES_VYRIGHTLEFT;
            break;         
         case 12:
            filtercfg_min = FILTERCFG_MIN_VXONCOME / FILTERCFG_RES_VXONCOME;
            filtercfg_max = FILTERCFG_MAX_VXONCOME / FILTERCFG_RES_VXONCOME;
            break;
         case 13:
            filtercfg_min = FILTERCFG_MIN_VYLEFTRIGHT / FILTERCFG_RES_VYLEFTRIGHT;
            filtercfg_max = FILTERCFG_MAX_VYLEFTRIGHT / FILTERCFG_RES_VYLEFTRIGHT;
            break;
         case 14:
            filtercfg_min = FILTERCFG_MIN_VXDEPART / FILTERCFG_RES_VXDEPART;
            filtercfg_max = FILTERCFG_MAX_VXDEPART / FILTERCFG_RES_VXDEPART;
            break;            
         }

         i = 1;
         while (i <= 8)
         {
            filtercfg_min /= 2;

            if (FilterConfigurationNode::is_integer(filtercfg_min) == 0)
            {
               filtercfg_min -= 0.5;
               bits3[i] = 1;
               i++;
            }
            else
            {
               bits3[i] = 0;
               i++;
            }
         }

         i = 1;
         while (i <= 5)
         {
            filtercfg_min /= 2;

            if (FilterConfigurationNode::is_integer(filtercfg_min) == 0)
            {
               filtercfg_min -= 0.5;
               bits2[i] = 1;
               i++;
            }
            else
            {
               bits2[i] = 0;
               i++;
            }
         }

         b2.bit1 = bits2[1];
         b2.bit2 = bits2[2];
         b2.bit3 = bits2[3];
         b2.bit4 = bits2[4];
         b2.bit5 = bits2[5];
         b2.bit6 = 0;
         b2.bit7 = 0;
         b2.bit8 = 0;

         b3.bit1 = bits3[1];
         b3.bit2 = bits3[2];
         b3.bit3 = bits3[3];
         b3.bit4 = bits3[4];
         b3.bit5 = bits3[5];
         b3.bit6 = bits3[6];
         b3.bit7 = bits3[7];
         b3.bit8 = bits3[8];

         i = 1;
         while (i <= 8)
         {
            filtercfg_max /= 2;

            if (FilterConfigurationNode::is_integer(filtercfg_max) == 0)
            {
               filtercfg_max -= 0.5;
               bits5[i] = 1;
               i++;
            }
            else
            {
               bits5[i] = 0;
               i++;
            }
         }

         i=1;
         while (i <= 5)
         {
            filtercfg_max /= 2;

            if (FilterConfigurationNode::is_integer(filtercfg_max) == 0)
            {
               filtercfg_max -= 0.5;
               bits4[i] = 1;
               i++;
            }
            else
            {
               bits4[i] = 0;
               i++;
            }
         }

         b4.bit1 = bits4[1];
         b4.bit2 = bits4[2];
         b4.bit3 = bits4[3];
         b4.bit4 = bits4[4];
         b4.bit5 = bits4[5];
         b4.bit6 = 0;
         b4.bit7 = 0;
         b4.bit8 = 0;

         b5.bit1 = bits5[1];
         b5.bit2 = bits5[2];
         b5.bit3 = bits5[3];
         b5.bit4 = bits5[4];
         b5.bit5 = bits5[5];
         b5.bit6 = bits5[6];
         b5.bit7 = bits5[7];
         b5.bit8 = bits5[8];

         output.data[0]= b1.byte;
         output.data[1]= b2.byte;
         output.data[2]= b3.byte;
         output.data[3]= b4.byte;
         output.data[4]= b5.byte;

         pub_.publish(output);
      }

      bool FilterConfigurationNode::is_integer(float dec)
      {
         return std::floor(dec) == dec;
      }

      CollisionConfigurationNode::CollisionConfigurationNode()
      {
         //Topic you want to publish
         pub_ = n_.advertise<can_msgs::Frame>("/sent_messages", 1);

        //Topic you want to subscribe
         sub_ = n_.subscribe("/received_messages", 1, &CollisionConfigurationNode::callback, this);
      }

      void CollisionConfigurationNode::callback(const can_msgs::Frame & input)
      {
         can_msgs::Frame output;
         //.... do something with the input and generate the output...

         output.header.stamp = ros::Time::now();
         output.id = 1024; //Decimal value of id 400
         output.dlc = 2;

         Byte b1, b2;
         int bits2[8];

         b1.bit1 = COLLDETCFG_WARNINGRESET;
         b1.bit2 = COLLDETCFG_ACTIVATION;
         b1.bit3 = 0;
         b1.bit4 = COLLDETCFG_MINTIME_VALID;
         b1.bit5 = 0;
         b1.bit6 = 0;
         b1.bit7 = 0;
         b1.bit8 = COLLDETCFG_CLEARREGIONS;
         
         double colldetcfg_mintime = COLLDETCFG_MINTIME / COLLDETCFG_RESTIME;
         int i = 1;

         while (i <= 8)
         {
            colldetcfg_mintime /= 2;

            if (CollisionConfigurationNode::is_integer(colldetcfg_mintime) == 0)
            {
               colldetcfg_mintime -= 0.5;
               bits2[i] = 1;
               i++;
            }
            else
            {
               bits2[i] = 0;
               i++;
            }
         }

         b2.bit1 = bits2[1];
         b2.bit2 = bits2[2];
         b2.bit3 = bits2[3];
         b2.bit4 = bits2[4];
         b2.bit5 = bits2[5];
         b2.bit6 = bits2[6];
         b2.bit7 = bits2[7];
         b2.bit8 = bits2[8];

         output.data[0]= b1.byte;
         output.data[1]= b2.byte;

         pub_.publish(output);
      }

      bool CollisionConfigurationNode::is_integer(float dec)
      {
         return std::floor(dec) == dec;
      }

      CollisionRegionConfigurationNode::CollisionRegionConfigurationNode()
      {
         //Topic you want to publish
         pub_ = n_.advertise<can_msgs::Frame>("/sent_messages", 1);

        //Topic you want to subscribe
         sub_ = n_.subscribe("/received_messages", 1, &CollisionRegionConfigurationNode::callback, this);
      }

      void CollisionRegionConfigurationNode::callback(const can_msgs::Frame & input)
      {
         can_msgs::Frame output;
         //.... do something with the input and generate the output...

         output.header.stamp = ros::Time::now();
         output.id = 1025; //Decimal value of id 401
         output.dlc = 8;

         Byte b1, b2, b3, b4, b5, b6, b7, b8;
         int bits1[8], bits2[8], bits3[8], bits4[8], bits5[8], bits6[8], bits7[8], bits8[8];

         b1.bit1 = 0;
         b1.bit2 = COLLDETREGCFG_ACTIVATION;
         b1.bit3 = COLLDETREGCFG_COORDINATES_VALID;
         b1.bit4 = 0;
         b1.bit5 = 0;
         b1.bit6 = 0;
         b1.bit7 = 0;
         b1.bit8 = 0;
         
         double colldetregcfg_regionid = COLLDETREGCFG_REGIONID;
         int i = 1;

         while (i <= 3)
         {
            colldetregcfg_regionid /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_regionid) == 0)
            {
               colldetregcfg_regionid -= 0.5;
               bits2[i] = 1;
               i++;
            }
            else
            {
               bits2[i] = 0;
               i++;
            }
         }

         b2.bit1 = bits2[1];
         b2.bit2 = bits2[2];
         b2.bit3 = bits2[3];
         b2.bit4 = 0;
         b2.bit5 = 0;
         b2.bit6 = 0;
         b2.bit7 = 0;
         b2.bit8 = 0;

         double colldetregcfg_point1x = COLLDETREGCFG_POINT1X / COLLDETREGCFG_POINTRES;
         double colldetregcfg_point1y = COLLDETREGCFG_POINT1Y / COLLDETREGCFG_POINTRES;
         double colldetregcfg_point2x = COLLDETREGCFG_POINT2X / COLLDETREGCFG_POINTRES;
         double colldetregcfg_point2y = COLLDETREGCFG_POINT2Y / COLLDETREGCFG_POINTRES;
         
         i = 4;
         while (i <= 8)
         {
            colldetregcfg_point1x /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_point1x) == 0)
            {
               colldetregcfg_point1x -= 0.5;
               bits4[i] = 1;
               i++;
            }
            else
            {
               bits4[i] = 0;
               i++;
            }
         }
         i = 1;
         while (i <= 8)
         {
            colldetregcfg_point1x /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_point1x) == 0)
            {
               colldetregcfg_point1x -= 0.5;
               bits3[i] = 1;
               i++;
            }
            else
            {
               bits3[i] = 0;
               i++;
            }
         }

         i = 1;
         while (i <= 8)
         {
            colldetregcfg_point1y /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_point1y) == 0)
            {
               colldetregcfg_point1y -= 0.5;
               bits5[i] = 1;
               i++;
            }
            else
            {
               bits5[i] = 0;
               i++;
            }
         }
         i = 1;
         while (i <= 3)
         {
            colldetregcfg_point1y /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_point1y) == 0)
            {
               colldetregcfg_point1y -= 0.5;
               bits4[i] = 1;
               i++;
            }
            else
            {
               bits4[i] = 0;
               i++;
            }
         }

         i = 4;
         while (i <= 8)
         {
            colldetregcfg_point2x /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_point2x) == 0)
            {
               colldetregcfg_point2x -= 0.5;
               bits7[i] = 1;
               i++;
            }
            else
            {
               bits7[i] = 0;
               i++;
            }
         }
         i = 1;
         while (i <= 8)
         {
            colldetregcfg_point2x /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_point2x) == 0)
            {
               colldetregcfg_point2x -= 0.5;
               bits6[i] = 1;
               i++;
            }
            else
            {
               bits6[i] = 0;
               i++;
            }
         }

         i = 1;
         while (i <= 8)
         {
            colldetregcfg_point2y /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_point2y) == 0)
            {
               colldetregcfg_point2y -= 0.5;
               bits8[i] = 1;
               i++;
            }
            else
            {
               bits8[i] = 0;
               i++;
            }
         }
         i = 1;
         while (i <= 3)
         {
            colldetregcfg_point2y /= 2;

            if (CollisionRegionConfigurationNode::is_integer(colldetregcfg_point2y) == 0)
            {
               colldetregcfg_point2y -= 0.5;
               bits7[i] = 1;
               i++;
            }
            else
            {
               bits7[i] = 0;
               i++;
            }
         }

         b3.bit1 = bits3[1];
         b3.bit2 = bits3[2];
         b3.bit3 = bits3[3];
         b3.bit4 = bits3[4];
         b3.bit5 = bits3[5];
         b3.bit6 = bits3[6];
         b3.bit7 = bits3[7];
         b3.bit8 = bits3[8];

         b4.bit1 = bits4[1];
         b4.bit2 = bits4[2];
         b4.bit3 = bits4[3];
         b4.bit4 = bits4[4];
         b4.bit5 = bits4[5];
         b4.bit6 = bits4[6];
         b4.bit7 = bits4[7];
         b4.bit8 = bits4[8];

         b5.bit1 = bits5[1];
         b5.bit2 = bits5[2];
         b5.bit3 = bits5[3];
         b5.bit4 = bits5[4];
         b5.bit5 = bits5[5];
         b5.bit6 = bits5[6];
         b5.bit7 = bits5[7];
         b5.bit8 = bits5[8];

         b6.bit1 = bits6[1];
         b6.bit2 = bits6[2];
         b6.bit3 = bits6[3];
         b6.bit4 = bits6[4];
         b6.bit5 = bits6[5];
         b6.bit6 = bits6[6];
         b6.bit7 = bits6[7];
         b6.bit8 = bits6[8];

         b7.bit1 = bits7[1];
         b7.bit2 = bits7[2];
         b7.bit3 = bits7[3];
         b7.bit4 = bits7[4];
         b7.bit5 = bits7[5];
         b7.bit6 = bits7[6];
         b7.bit7 = bits7[7];
         b7.bit8 = bits7[8];

         b8.bit1 = bits8[1];
         b8.bit2 = bits8[2];
         b8.bit3 = bits8[3];
         b8.bit4 = bits8[4];
         b8.bit5 = bits8[5];
         b8.bit6 = bits8[6];
         b8.bit7 = bits8[7];
         b8.bit8 = bits8[8];

         output.data[0]= b1.byte;
         output.data[1]= b2.byte;
         output.data[2]= b3.byte;
         output.data[3]= b4.byte;
         output.data[4]= b5.byte;
         output.data[5]= b6.byte;
         output.data[6]= b7.byte;
         output.data[7]= b8.byte;

         pub_.publish(output);
      }

      bool CollisionRegionConfigurationNode::is_integer(float dec)
      {
         return std::floor(dec) == dec;
      }

};  // namespace socketcan_bridge