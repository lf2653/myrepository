#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pb_msgs/ClusterRadar.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <markers/visualization_marker_cluster.h>

#include <iostream>
#include <cmath>

namespace markers
{

    MarkerClassClusterDecoded::MarkerClassClusterDecoded()
        {
           //Topic you want to publish
           pub_ = n_.advertise<visualization_msgs::Marker>("/visualization_marker_cluster", 50);

           //Topic you want to subscribe
           sub_ = n_.subscribe("/cluster_decoded_messages", 50, &MarkerClassClusterDecoded::callback, this);
        }

   	void MarkerClassClusterDecoded::callback(const pb_msgs::ClusterRadar & input)
      	{	
         	visualization_msgs::Marker sphere;
            //.... do something with the input and generate the output...

         	sphere.header.frame_id = "/radar";
         	sphere.ns = "markers";
         	sphere.action = visualization_msgs::Marker::ADD;

         	sphere.header.stamp = ros::Time();
         	//sphere.lifetime = ros::Duration(0.5);

         	sphere.id = input.target_id;

         	sphere.type = visualization_msgs::Marker::SPHERE;

         	sphere.scale.x = 0.1;
            sphere.scale.y = 0.1;
            sphere.scale.z = 0.1;

            sphere.color.r = 1.0f;
            sphere.color.a = 1.0;

            sphere.pose.orientation.w = 1.0;
            sphere.pose.position.x = input.longitude_dist;
            sphere.pose.position.y = input.lateral_dist;
            sphere.pose.position.z = 0;

         	pub_.publish(sphere);

      	}
        
/*
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

            points.header.frame_id = line_strip.header.frame_id = "/radar";
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
                float x;
                float y;
                float z;
                if (i==0 || i==4)
                {
                    x = input.longitude_dist + (input.width/2)*tan((input.orientation_angle*M_PI)/180);
                    y = input.lateral_dist + input.width / 2;
                    z = input.length / 2;
                }
                else if (i==1)
                {
                    x = input.longitude_dist - (input.width/2)*tan((input.orientation_angle*M_PI)/180);
                    y = input.lateral_dist - input.width / 2;
                    z = input.length / 2;
                }
                else if (i==2)
                {
                    x = input.longitude_dist - (input.width/2)*tan((input.orientation_angle*M_PI)/180);
                    y = input.lateral_dist - input.width / 2;
                    z = - input.length / 2;
                }
                else if (i==3)
                {
                    x = input.longitude_dist + (input.width/2)*tan((input.orientation_angle*M_PI)/180);
                    y = input.lateral_dist + input.width / 2;
                    z = - input.length / 2;
                }
   
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
  
                line_strip.points.push_back(p);

            }
  
            pub_.publish(line_strip);

        } */

}  //Namespace markers
