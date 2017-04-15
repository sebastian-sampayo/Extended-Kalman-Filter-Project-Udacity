#include <iostream>
#include <stdlib.h>

#include "common.h"
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}

VectorXd Tools::CalculatePolar2Cartesian(const Eigen::VectorXd& polar) {
  /**
  * Cartesian velocity is the actual velocity of the object, while polar velocity (ro_dot) is the 
  * measure from the radar.
  * Cartesian velocity v cannot be retrieved from ro_dot, because ro_dot is a linear projection, 
  * so in theory we have lost information about one dimension.
  * However, we have at least "some" information about velocity. In this approach we assume that
  * the actual velocity is a linear scaled version of the measured velocity. The amount of scale 
  * is a parameter to be defined in "init_velocity_scale".
  */
  // Get polar coordinates from the input
  const float ro = polar[0];
  const float phi = polar[1];
  const float ro_dot = polar[2];
  
  // Calculate cartesian position
  const float p_x = ro * cos(phi);
  const float p_y = ro * sin(phi);
  
  // Calculate cartesian velocity
  const float v_x = init_velocity_scale * ro_dot * cos(phi);
  const float v_y = init_velocity_scale * ro_dot * sin(phi);
  
  // Create output
  VectorXd cartesian(4);
  cartesian << p_x, p_y, v_x, v_y;

  return cartesian;
}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  return estimations[0];
}