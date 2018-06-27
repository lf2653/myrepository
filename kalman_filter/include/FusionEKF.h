#ifndef FusionEKF_H_
#define FusionEKF_H_
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

private:
  
  // check whether the tracking toolbox was initiallized or not (first measurement)
 // bool is_initialized_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  bool initialization_done_;
};

/**
 * Kalman Filter update and prediction math lives in here.
*/
 
KalmanFilter ekf_;

// Struct with kalman data and timestamp for next time
struct kalman_data{
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  long previous_timestamp_;
};

// Vector for kalman_data structure
std::vector <kalman_data> kalman_vector;

#endif /* FusionEKF_H_ */
