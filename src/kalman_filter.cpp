#include "kalman_filter.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::cerr;
using std::endl;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

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

void KalmanFilter::Predict() {
  /**
  TODO: DONE
    * predict the state
  */
  MatrixXd Ft = F_.transpose();

  x_ = F_ * x_;
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO: DONE
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  Update_(z, z_pred);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO: DONE
    * update the state by using Extended Kalman Filter equations
  */
  if (h_) {
    VectorXd z_pred = h_(x_);
    Update_(z, z_pred);
  } else {
    // skip update step
    cerr << 
      "KalmanFilter::UpdateEKF() - WARNING: h_ empty. Set h(x) function before calling UpdateEKF()"
      << endl;
  }
}

/*****************************************************************************
 *  PRIVATE
 ****************************************************************************/
void KalmanFilter::Update_(const Eigen::VectorXd &z, const Eigen::VectorXd &z_pred) {
  // Calculate the kalman gain matrix
  const VectorXd y = z - z_pred;
  const MatrixXd Ht = H_.transpose();
  const MatrixXd S = H_ * P_ * Ht + R_;
  const MatrixXd Si = S.inverse();
  const MatrixXd PHt = P_ * Ht;
  const MatrixXd K = PHt * Si;

  //new estimates
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}