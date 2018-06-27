#include <math.h>       
#include "kalman_filter.h"
#include <ros/ros.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

/*void KalmanFilter::Init(int &id_in, VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  id_= id_in;
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
} */

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &m) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  ROS_INFO_STREAM("EKF: Posterior update.");
  float rho_pred = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  float phi_pred = atan2(x_(1), x_(0));
  float rho_dot_pred;
  if (fabs(rho_pred) < 0.0001) {
    rho_dot_pred = 0;
  } else {
    rho_dot_pred = (x_(0)*x_(2) + x_(1)*x_(3))/rho_pred;
  }
  float rho = sqrt(m(0)*m(0) + m(1)*m(1));
  float phi = atan2(m(1), m(0));
  float rho_dot;
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (m(0)*m(2) + m(1)*m(3))/rho;
  }
  ROS_INFO_STREAM("EKF: Measurement update.");
  VectorXd z_pred(3);
  VectorXd z(3);
  z_pred << rho_pred, phi_pred, rho_dot_pred;
  z << rho, phi, rho_dot;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  ROS_INFO_STREAM("EKF: Measurement update done.");
}
