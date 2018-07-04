#include <ros/ros.h>
#include <string>
#include <pb_msgs/ContiRadar.h>
#include "radar_input.h"

#include <iostream>
#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

radar_input::radar_input()
    {
        //Topic you want to publish
        pub_ = n_.advertise<pb_msgs::ContiRadar>("/filtered_messages", 50);

        //Topic you want to subscribe
        sub_ = n_.subscribe("/decoded_messages", 50, &radar_input::callback, this);
    }

void radar_input::callback(const pb_msgs::ContiRadar & input)
    {
        ROS_INFO_STREAM("Radar input stream recieved..");
        pb_msgs::ContiRadar output;

        string sensor_type;
        MeasurementPackage meas_package;

        meas_package.sensor_type_ = MeasurementPackage::RADAR;
        meas_package.raw_measurements_ = VectorXd(4);

        float x = input.longitude_dist;
        float y = input.lateral_dist;
        float vx = input.longitude_vel;
        float vy = input.lateral_vel;
        meas_package.raw_measurements_ << x, y, vx, vy;
        meas_package.timestamp_ = input.header.stamp.sec*1000000000 + input.header.stamp.nsec;
        meas_package.id_ = input.obstacle_id;

        // Create a Fusion EKF instance
        FusionEKF fusionEKF;
        ROS_INFO_STREAM("Radar input: Calling process measurement. ");
        fusionEKF.ProcessMeasurement(meas_package);
        ROS_INFO_STREAM("Radar input: Finished process measurement. ");

        // output the estimation
        output.longitude_dist = ekf_.x_(0);
        output.lateral_dist = ekf_.x_(1);
        output.longitude_vel = ekf_.x_(2);
        output.lateral_vel = ekf_.x_(3);

        // transfer all the properties to filtered_messages topic
        output.header = input.header;
        output.obstacle_id = input.obstacle_id;
        output.rcs = input.rcs;
        output.orientation_angle = input.orientation_angle;
        output.length = input.length;
        output.width = input.width;
        output.obstacle_class = input.obstacle_class;
        output.meas_state = input.meas_state;

        pub_.publish(output);

    }