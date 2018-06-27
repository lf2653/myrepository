#include "FusionEKF.h"
#include "tools.h"
#include <iostream>
#include <vector>
#include <ros/ros.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {

/*  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_      = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
//  TODO:
  //  * Finish initializing the FusionEKF.
   // * Set the process and measurement noises

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1; */
}
 
/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  ROS_INFO_STREAM("Start ProcessMeasurement.. ");

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/

  // Id of the object
  ekf_.id_ = measurement_pack.id_;

  // Push kalman vector

  if (kalman_vector.size() == 0)
  {
    for (int n = 0; n<10; n++)
    {
      kalman_vector.push_back(kalman_data());
    }
  } 

  // First time computation

  if (kalman_vector[ekf_.id_].previous_timestamp_ == 0) {

  	// Increase the size of the kalman_vector
    ROS_INFO_STREAM("First computation. Ekf_ id: " << ekf_.id_ );

    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
    */
    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       Initialize state
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);      
      ekf_.x_(2) = measurement_pack.raw_measurements_(2);
      ekf_.x_(3) = measurement_pack.raw_measurements_(3);
    }

    ekf_.previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
  //  is_initialized_ = true;

  	//state covariance matrix P
  	ekf_.P_ = MatrixXd(4, 4);
  	ekf_.P_ << 1, 0, 0, 0,
       	  	   0, 1, 0, 0,
          	   0, 0, 1000, 0,
          	   0, 0, 0, 1000;
    //return;
  }

  else
  {
  	ekf_.x_ = kalman_vector[ekf_.id_].x_;
  	ekf_.P_ = kalman_vector[ekf_.id_].P_;
  	ekf_.previous_timestamp_ = kalman_vector[ekf_.id_].previous_timestamp_;
    ROS_INFO_STREAM("New data update after init.");
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
        	0, 1, 0, 1,
           	0, 0, 1, 0,
           	0, 0, 0, 1;

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - ekf_.previous_timestamp_) / 1000000000;	//dt - expressed in seconds
  ekf_.previous_timestamp_ = measurement_pack.timestamp_;

  float dt_2 = dt   * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;

  //set the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
             0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
             dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

  ROS_INFO_STREAM("Update EKF Predict..");

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
   // Tools tools;
    Hj_      = MatrixXd(3, 4);
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    R_radar_ = MatrixXd(3, 3);
    
    //measurement covariance matrix - radar
  	R_radar_ << 0.09, 0, 0,
              	0, 0.0009, 0,
              	0, 0, 0.09;

    ekf_.R_ = R_radar_;

    ROS_INFO_STREAM("Calling update EKF.");
    
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // Store all the values for next measurement
  kalman_vector[ekf_.id_].x_ = ekf_.x_;
  kalman_vector[ekf_.id_].P_ = ekf_.P_;
  kalman_vector[ekf_.id_].previous_timestamp_ = ekf_.previous_timestamp_;

}
