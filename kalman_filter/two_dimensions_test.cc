#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "gtest/gtest.h"
#include "kalman_filter/kalman_filter.h"

namespace slam_dunk::kalman_filter {
TEST(MobileRobotKFTest, UltrasonicMeasurementUpdate) {
  // Suppose we have a 2D state: [position, velocity]
  constexpr int n = 2;
  constexpr int m = 1;  // measurement is distance from an ultrasonic sensor
  constexpr double dt = 0.1;  // time step

  // State transition matrix for constant velocity model
  Eigen::MatrixXd A(n, n);
  A << 1, dt, 0, 1;

  // Measurement matrix: we measure position only.
  Eigen::MatrixXd C(m, n);
  C << 1, 0;

  // Process noise covariance: assume some uncertainty in acceleration.
  Eigen::MatrixXd Q(n, n);
  Q << 0.05, 0.01, 0.01, 0.05;

  // Measurement noise covariance: based on ultrasonic sensor specs.
  // Suppose our ultrasonic sensor has a variance of 0.2^2 = 0.04.
  Eigen::MatrixXd R(m, m);
  R << 0.04;

  // Initial estimate error covariance: assume moderate uncertainty.
  Eigen::MatrixXd P(n, n);
  P << 1, 0, 0, 1;

  // Construct the Kalman filter
  KalmanFilter kf(dt, A, C, Q, R, P);

  // Initialize the filter with an initial state guess.
  double t = 0.0;
  Eigen::VectorXd x0(n);
  // Let's say we guess the robot starts at 5.0 meters, velocity 0.
  x0 << 5.0, 0.0;
  kf.Init(t, x0);

  // Simulate a measurement from the ultrasonic sensor:
  // For instance, the sensor reads 5.1 meters.
  Eigen::VectorXd y(m);
  y << 5.1;
  t += dt;
  kf.Update(y);

  // Now, based on the Kalman filter equations and the values above,
  // we can expect the state to adjust slightly.
  // (The expected value here would be calculated beforehand or through
  // simulation.)
  Eigen::VectorXd x_est = kf.State();
  // For demonstration, let's suppose we expect position to be ~5.08.
  EXPECT_NEAR(x_est(0), 5.08, 0.1);

  LOG(INFO) << absl::StreamFormat("Updated position: %.3f, velocity: %.3f",
                                  x_est(0), x_est(1));
}
}  // namespace slam_dunk::kalman_filter
