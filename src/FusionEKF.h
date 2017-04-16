/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 6: Extended Kalman Filter
 * Date: 16th April 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: FusionEKF.h
 * Description: This is a high level class containing the Sensor Fusion.
 * ProcessMeasurement() expects a measurement package described in the
 * MeasurementPackage class (file: measurement_package.h)
\****************************************************************************/

#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
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
  * @param measurement_pack The last measurement
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the FusionEKF toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  
  // measurement covariance matrices and measurement matrices for laser and radar
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd Hj_;
  
  //acceleration noise components
  float noise_ax;
  float noise_ay;
};

#endif /* FusionEKF_H_ */
