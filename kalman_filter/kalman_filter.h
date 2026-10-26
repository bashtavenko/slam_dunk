#ifndef SLAM_DUNK_KALMAN_FILTER_KALMAN_FILTER_H_
#define SLAM_DUNK_KALMAN_FILTER_KALMAN_FILTER_H_
#include <Eigen/Eigen>

namespace slam_dunk {

class KalmanFilter {
 public:

  // A - State transition matrix (n x n)
  // C - Measurement/output matrix (m x n)
  // Q - Process noise covariance (n x n)
  // R - Measurement noise covariance (m x m)
  // P - Initial estimate error covariance (n x n)
  KalmanFilter(double dt, const Eigen::MatrixXd& A, const Eigen::MatrixXd& C,
               const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R,
               const Eigen::MatrixXd& P);

  // Create a blank estimator.
  KalmanFilter() {};

  // Initialize state to zero
  void Init();

  // Initialize with a prior state guess at time t0
  void Init(double t0, const Eigen::VectorXd& x0);

  // Time update (Predict)
  // Propagates state and covariance forward using the process model.
  // Returns true of successful.
  bool Predict();
  bool Predict(double dt, const Eigen::MatrixXd& A);  // variable-rate overload

  // Measurement update (Correct)
  // Incorporates a new sensor measurement y.
  // Call independently per sensor at its own arrival rate.
  // Returns false if the filter has not been initialized.
  bool Correct(const Eigen::VectorXd& y);

  //  Single-step operation. Predict + correct.
  bool Update(const Eigen::VectorXd& y);

  // Single-step operation. Has optional A and dt
  bool Update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd& A);

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
};

}  // namespace slam_dunk

#endif  // SLAM_DUNK_KALMAN_FILTER_KALMAN_FILTER_H_
