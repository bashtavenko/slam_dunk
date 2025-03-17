// Based on https://github.com/hmartiro/kalman-cpp
#ifndef SLAM_DUNK_KALMAN_FILTER_KALMAN_FILTER_H_
#define SLAM_DUNK_KALMAN_FILTER_KALMAN_FILTER_H_
#include <Eigen/Eigen>

namespace slam_dunk {

class KalmanFilter {
 public:
  // Create a Kalman filter with the specified matrices.
  // A - System dynamics matrix
  // C - Output matrix
  // Q - Process noise covariance.
  //     Uncertainty in the process model. In static system it should be 0;
  // R - Measurement noise covariance
  //     This is based on the sensor noise characteristic.
  //     It tells the filter how much to trust the measurement.
  // P - Estimate error covariance
  //     This is the key covariance that the filter updates after every
  //     measurement. It reflects the uncertainty in the current
  //     state estimate x_hat
  KalmanFilter(double dt, const Eigen::MatrixXd& A, const Eigen::MatrixXd& C,
               const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
               const Eigen::MatrixXd& P);

  // Create a blank estimator.
  KalmanFilter() {};

  // Initialize the filter with initial states as zero.
  void Init();

  // Initialize the filter with a guess for initial states.
  void Init(double t0, const Eigen::VectorXd& x0);

  // Update the estimated state based on measured values. The
  // time step is assumed to remain constant. Returns true of successful.
  bool Update(const Eigen::VectorXd& y);

  // Update the estimated state based on measured values,
  // using the given time step and dynamics matrix.
  // Returns true of successful.
  bool Update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

  // Return the current state and time.
  Eigen::VectorXd State() const { return x_hat_; };
  double Time() const { return t_; };

  Eigen::MatrixXd EstimateErrorCovariance() const { return P_; }
  void SetMeasurementNoiseCovariance(const Eigen::MatrixXd& noise) {
    R_ = noise;
  }

 private:
  // Matrices for computation
  Eigen::MatrixXd A_;
  Eigen::MatrixXd C_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd K_;
  Eigen::MatrixXd P0_;

  // System dimensions
  int32_t m_;
  int32_t n_;

  // Discrete time step
  double dt_;

  // n-size identity
  Eigen::MatrixXd I_;

  // Is the filter initialized?
  bool initialized_;

  // Initial and current time
  double t0_;
  double t_;

  // Estimated states
  Eigen::VectorXd x_hat_;
  Eigen::VectorXd x_hat_new_;
};

}  // namespace slam_dunk

#endif  // SLAM_DUNK_KALMAN_FILTER_KALMAN_FILTER_H_
