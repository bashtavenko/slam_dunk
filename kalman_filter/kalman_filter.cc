#include "kalman_filter.h"

namespace slam_dunk {

KalmanFilter::KalmanFilter(double dt, const Eigen::MatrixXd& A,
                           const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q,
                           const Eigen::MatrixXd& R, const Eigen::MatrixXd& P)
    : A_(A),
      C_(C),
      Q_(Q),
      R_(R),
      P0_(P),
      m_(C.rows()),
      n_(A.rows()),
      dt_(dt),
      I_(n_, n_),
      initialized_(false),
      x_hat_(n_) {}

void KalmanFilter::Init(double t0, const Eigen::VectorXd& x0) {
  x_hat_ = x0;
  P_ = P0_;
  t0_ = t0;
  t_ = t0;
  initialized_ = true;
}

void KalmanFilter::Init() {
  x_hat_.setZero();
  P_ = P0_;
  t0_ = 0.0;
  t_ = 0.0;
  initialized_ = true;
}

bool KalmanFilter::Predict() {
  if (!initialized_) return false;
  x_hat_ = A_ * x_hat_;
  P_ = A_ * P_ * A_.transpose() + Q_;
  t_ += dt_;
  return true;
}

bool KalmanFilter::Predict(double dt, const Eigen::MatrixXd& A) {
  if (!initialized_) return false;
  dt_ = dt;
  A_ = A;
  return Predict();
}

bool KalmanFilter::Correct(const Eigen::VectorXd& y) {
  if (!initialized_) return false;
  // TODO: Cholesky
  K_ = P_ * C_.transpose() * (C_ * P_ * C_.transpose() + R_).inverse();
  x_hat_ += K_ * (y - C_ * x_hat_);
  // TODO: Joseph form
  P_ = (I_ - K_ * C_) * P_;
  return true;
}

bool KalmanFilter::Update(const Eigen::VectorXd& y) {
  if (!initialized_) return false;
  Predict();
  return Correct(y);
}

bool KalmanFilter::Update(const Eigen::VectorXd& y, double dt,
                          const Eigen::MatrixXd& A) {
  A_ = A;
  dt_ = dt;
  return Update(y);
}

}  // namespace slam_dunk
