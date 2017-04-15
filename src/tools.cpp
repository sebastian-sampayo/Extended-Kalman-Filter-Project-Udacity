#include "tools.h"

#include <iostream>
#include <stdlib.h>

#include "common.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::cerr;
using std::endl;

Tools::Tools() {}

Tools::~Tools() {}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
    * If the radio of the measurement is 0, it would mean that the object being tracked is located
    * at the same position as the radar, which is inside the car. This is unlikely and we will
    * consider it as a measurement error, skipping the update step.
  */
  // Create output matrix
  MatrixXd Hj = MatrixXd::Zero(3,4);
  
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);

  //check division by zero. If c1 = 0 it means that the radio of the measurement is 0
  if(fabs(c1) < 0.0001){
    cerr << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

#ifdef DEBUG
  cout << "Hj: " << endl << Hj << endl;
#endif
  return Hj;
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