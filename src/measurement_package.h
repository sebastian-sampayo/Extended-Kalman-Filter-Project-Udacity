/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 6: Extended Kalman Filter
 * Date: 16th April 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: tools.h
 * Description: A class for the measurements values.
\****************************************************************************/

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */
