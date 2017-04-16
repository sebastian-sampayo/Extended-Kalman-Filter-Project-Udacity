/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 6: Extended Kalman Filter
 * Date: 16th April 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: FusionEKF.cpp
 * Description: Implementation of FusionEKF class (see header for details)
\****************************************************************************/

#include "FusionEKF.h"

#include <assert.h>
#include <iostream>

#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*****************************************************************************
 *  PUBLIC
 ****************************************************************************/

// ----------------------------------------------------------------------------
/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  ekf_.x_ = VectorXd(4);
  ekf_.F_ = MatrixXd(4, 4);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);

  //the initial transition matrix F_
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
            

  //measurement matrix - laser
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // measurement non linear function - radar
  ekf_.h_ = [](const VectorXd &x_state) {
    VectorXd z(3);
    //recover state parameters
    const double px = x_state(0);
    const double py = x_state(1);
    const double vx = x_state(2);
    const double vy = x_state(3);
    
    //z = h(x)
    z(0) = sqrt(px*px + py*py);
    z(1) = atan2(py, px);
    z(2) = (px*vx + py*vy) / z(0);
    return z;
  };

  //state covariance matrix P
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  //the process covariance matrix Q_
  ekf_.Q_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            1, 0, 1, 0,
            0, 1, 0, 1;

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;

}

// ----------------------------------------------------------------------------
/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

// ----------------------------------------------------------------------------
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

#ifdef DEBUG
    cout << "---------- New measurement ---------" << endl;
#endif

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * TODO: DONE
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
    */
    // first measurement
#ifdef DEBUG
    cout << "x State init" << endl;
#endif
    ekf_.x_ = VectorXd(4);

      // Initialize state.
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ekf_.x_ = tools.ConvertRadar2State(measurement_pack.raw_measurements_);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
                 measurement_pack.raw_measurements_[1],
                 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    previous_timestamp_ = measurement_pack.timestamp_;  
    return;
  }


  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO: DONE
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds and timestamps are in microseconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //compute the time elapsed between the current and previous measurements
  //dt - expressed in seconds
  const float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  const float dt_2 = dt * dt;
  const float dt_3 = dt_2 * dt;
  const float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
               0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
               dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
               0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO: DONE
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     * Note: In this case the measurement covariance matrices do not change for
     * each time step, so we could set them in the initialization instead of 
     * every time. However, in a more complex case we could have the error
     * information from the sensors datasheets, so R would change in each time step.
     * Note on note: Actually NOT! We have one 'R' for laser and another 'R' for radar,
     * so we must set 'R' of the KF on each update step.
     * Set measurement matrix and measurement covariance matrix in each case, 
     * then update the state.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // TODO: there might be some shit going on when the phi measurement changes suddenly from pi to -pi. Check this.
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    if (ekf_.H_ != MatrixXd::Zero(3,4)) { // sanity check
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
  } else {
    // Laser updates
    assert(measurement_pack.sensor_type_ == MeasurementPackage::LASER);
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

#ifdef DEBUG
  // print the output
  cout << "x_ = " << endl << ekf_.x_ << endl;
  cout << "P_ = " << endl << ekf_.P_ << endl;
#endif
}
