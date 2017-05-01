/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project 6: Extended Kalman Filter
 * Date: 16th April 2017
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: kalman_filter.cpp
 * Description: Implementation of KalmanFilter class (see header for details)
\****************************************************************************/

#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*****************************************************************************
 *  PUBLIC
 ****************************************************************************/

// ----------------------------------------------------------------------------
KalmanFilter::KalmanFilter() {}

// ----------------------------------------------------------------------------
KalmanFilter::~KalmanFilter() {}

// ----------------------------------------------------------------------------
void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in,
                        VectorField &h_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  h_ = h_in;
}

// ----------------------------------------------------------------------------
void KalmanFilter::Predict() {
  const MatrixXd Ft = F_.transpose();

  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
}

// ----------------------------------------------------------------------------
void KalmanFilter::Update(const VectorXd &z) {
  //Simple Kalman Filter
  const VectorXd z_pred = H_ * x_;
  Update_(z, z_pred);
}

// ----------------------------------------------------------------------------
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //Extended Kalman Filter
  assert(h_); // h_ must be set.
  const VectorXd z_pred = h_(x_);
  Update_(z, z_pred);
}

/*****************************************************************************
 *  PRIVATE
 ****************************************************************************/
 
// ----------------------------------------------------------------------------
void KalmanFilter::Update_(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred) {
  //Calculate the kalman gain matrix
  const VectorXd y = z - z_pred;
  const MatrixXd Ht = H_.transpose();
  const MatrixXd S = H_ * P_ * Ht + R_;
  const MatrixXd Si = S.inverse();
  const MatrixXd PHt = P_ * Ht;
  const MatrixXd K = PHt * Si;

  //angle normalization
  while(y(1) > M_PI) y(1) -= 2.*M_PI;
  while(y(1) < M_PI) y(1) += 2.*M_PI;

  //new estimates
  x_ = x_ + (K * y);
  const long x_size = x_.size();
  const MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}