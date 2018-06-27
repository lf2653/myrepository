#ifndef RADAR_INFO_H_
#define RADAR_INFO_H_
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <ros/ros.h>
#include <pb_msgs/ContiRadar.h>
#include <stdlib.h>
#include "FusionEKF.h"
#include "kalman_filter.h"
#include "ground_truth_package.h"
#include "measurement_package.h"

#include <iostream>

class radar_input
   {

   public:

      radar_input();

      void callback(const pb_msgs::ContiRadar & input);

   private:
         
      ros::NodeHandle n_; 
      ros::Publisher pub_;
      ros::Subscriber sub_;

};//End of class

#endif // RADAR_INFO_H_