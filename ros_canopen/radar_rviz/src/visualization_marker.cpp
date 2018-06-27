#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pb_msgs/ContiRadar.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <markers/visualization_marker.h>

#include <iostream>
#include <cmath>

namespace markers
{

    MarkerClassDecoded::MarkerClassDecoded()
        {
           //Topic you want to publish
           pub_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 50);

           //Topic you want to subscribe
           sub_ = n_.subscribe("/decoded_messages", 50, &MarkerClassDecoded::callback, this);
        }

   	void MarkerClassDecoded::callback(const pb_msgs::ContiRadar & input)
      	{	
         	visualization_msgs::Marker points, line_strip;
            //.... do something with the input and generate the output...

         	points.header.frame_id = line_strip.header.frame_id = "/base_link";
         	points.ns = line_strip.ns = "markers";
         	points.action = line_strip.action = visualization_msgs::Marker::ADD;
         	points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

         	points.header.stamp = line_strip.header.stamp = ros::Time();
         	points.lifetime = line_strip.lifetime = ros::Duration(0.2);

         	points.id = input.obstacle_id / 10;
         	line_strip.id = input.obstacle_id;

         	points.type = visualization_msgs::Marker::POINTS;
         	line_strip.type = visualization_msgs::Marker::LINE_STRIP;

         	points.scale.x = 0.1;
         	points.scale.y = 0.1;
         	line_strip.scale.x = 0.1;

            if (input.meas_state == 0 || input.meas_state == 4)
            {
                line_strip.color.r = 1.0f;
                line_strip.color.a = 1.0;
            }
            else if (input.meas_state == 1 || input.meas_state == 5)
            {
                line_strip.color.g = 1.0f;
                line_strip.color.a = 1.0; 
            }
            else if (input.meas_state == 2)
            {
                line_strip.color.b = 1.0f;
                line_strip.color.a = 1.0;
            }
            else if (input.meas_state == 3)
            {
                line_strip.color.b = 0.8f;
                line_strip.color.g = 0.8f;
                line_strip.color.a = 1.0;
            }


     		// Create the vertices for the points and lines
     		for (uint32_t i = 0; i < 5; i++)
			{
         		float y;
         		float z;
         		if (i==0 || i==4)
     			{
            		y = input.lateral_dist + input.width / 2;
            		z = input.length / 2;
         		}
         		else if (i==1)
         		{
            		y = input.lateral_dist - input.width / 2;
            		z = input.length / 2;
         		}
         		else if (i==2)
         		{
            		y = input.lateral_dist - input.width / 2;
            		z = - input.length / 2;
         		}
         		else if (i==3)
         		{
            		y = input.lateral_dist + input.width / 2;
            		z = - input.length / 2;
         		}
   
         		geometry_msgs::Point p;
         		p.x = input.longitude_dist;
         		p.y = y;
         		p.z = z;
  
        		line_strip.points.push_back(p);

      		}
  
         	pub_.publish(line_strip);

      	}
        

    MarkerClassFiltered::MarkerClassFiltered()
        {
           //Topic you want to publish
           pub_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker", 50);

           //Topic you want to subscribe
           sub_ = n_.subscribe("/filtered_messages", 50, &MarkerClassFiltered::callback, this);
        }

    void MarkerClassFiltered::callback(const pb_msgs::ContiRadar & input)
        {   
            visualization_msgs::Marker points, line_strip;
            //.... do something with the input and generate the output...

            points.header.frame_id = line_strip.header.frame_id = "/base_link";
            points.ns = line_strip.ns = "markers";
            points.action = line_strip.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

            points.header.stamp = line_strip.header.stamp = ros::Time();
            points.lifetime = line_strip.lifetime = ros::Duration(0.2);

            points.id = input.obstacle_id / 10;
            line_strip.id = input.obstacle_id;

            points.type = visualization_msgs::Marker::POINTS;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;

            points.scale.x = 0.1;
            points.scale.y = 0.1;
            line_strip.scale.x = 0.1;

            if (input.meas_state == 0 || input.meas_state == 4)
            {
                line_strip.color.r = 1.0f;
                line_strip.color.a = 1.0;
            }
            else if (input.meas_state == 1 || input.meas_state == 5)
            {
                line_strip.color.g = 1.0f;
                line_strip.color.a = 1.0; 
            }
            else if (input.meas_state == 2)
            {
                line_strip.color.b = 1.0f;
                line_strip.color.a = 1.0;
            }
            else if (input.meas_state == 3)
            {
                line_strip.color.b = 0.8f;
                line_strip.color.g = 0.8f;
                line_strip.color.a = 1.0;
            }


            // Create the vertices for the points and lines
            for (uint32_t i = 0; i < 5; i++)
            {
                float y;
                float z;
                if (i==0 || i==4)
                {
                    y = input.lateral_dist + input.width / 2;
                    z = input.length / 2;
                }
                else if (i==1)
                {
                    y = input.lateral_dist - input.width / 2;
                    z = input.length / 2;
                }
                else if (i==2)
                {
                    y = input.lateral_dist - input.width / 2;
                    z = - input.length / 2;
                }
                else if (i==3)
                {
                    y = input.lateral_dist + input.width / 2;
                    z = - input.length / 2;
                }
   
                geometry_msgs::Point p;
                p.x = input.longitude_dist;
                p.y = y;
                p.z = z;
  
                line_strip.points.push_back(p);

            }
  
            pub_.publish(line_strip);

        }
}  //Namespace markers
