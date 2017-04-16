/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 6: Extended Kalman Filter
 * Date: 16th April 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: tools.h
 * Description: A class for the ground truth values.
\****************************************************************************/

#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd gt_values_;
};

#endif /* GROUND_TRUTH_PACKAGE_H_ */
