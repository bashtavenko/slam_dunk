#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "kalman_filter/kalman_filter.h"

namespace slam_dunk {
namespace kalman_filter {

// For a static system, the KF isn’t “tracking” a moving state but is instead
// used to improve the estimate of a constant parameter (here, the equilibrium
// position) as more measurements come in. Every time you get a new measurement,
// you update your current best estimate and reduce its uncertainty.
TEST(SpringExampleTest, EquilibriumFusion) {
  // For a static system, we have a 1-D state (the equilibrium position)
  constexpr int n = 1;  // number of states
  constexpr int m = 1;  // number of measurements
  constexpr double dt = 1.0;  // time step (irrelevant for static system)

  // Define the system matrices for a static model: x[k+1] = x[k]
  Eigen::MatrixXd A(n, n);
  Eigen::MatrixXd C(m, n);
  Eigen::MatrixXd Q(n, n);  // process noise covariance
  Eigen::MatrixXd R(n, n);  // measurement noise covariance (initially set to sensor 1)
  Eigen::MatrixXd P(n, n);  // estimate error covariance

  A << 1;
  C << 1;
  Q << 0;       // no process noise (static system)
  R << 1;       // sensor 1: left spring measurement noise variance (stiffer spring, lower variance)
  P << 100;     // high initial uncertainty

  // Construct the Kalman filter
  KalmanFilter kf(dt, A, C, Q, R, P);

  // Initialize the filter; initial state guess can be arbitrary, say 5
  double t = 0.0;
  Eigen::VectorXd x0(n);
  x0 << 5;
  kf.Init(t, x0);

  // First measurement from sensor 1 (left spring at x1 = 0)
  Eigen::VectorXd y(m);
  y << 0;
  t += dt;
  kf.Update(y);
  // After this update, the filter has fused the initial guess with a measurement of 0.
  // (The exact value depends on the high initial uncertainty.)

  // Now simulate the second measurement from sensor 2 (right spring at x2 = 10)
  // For sensor 2, we assume a higher measurement noise variance since the spring is looser.
  Eigen::MatrixXd R2(n, n);
  R2 << 2;  // sensor 2: higher variance (looser spring)
  kf.SetMeasurementNoiseCovariance(R2);

  y << 10;  // measurement from sensor 2
  t += dt;
  kf.Update(y);

  // With optimal sequential fusion, the overall estimate should mimic a weighted average:
  // Weight for sensor1 = 1/R1 / (1/R1+1/R2) = 1/1 / (1 + 1/2) = 0.667,
  // Weight for sensor2 = 1/R2 / (1/R1+1/R2) = (1/2)/1.5 = 0.333.
  // Therefore, fused equilibrium = 0.667*0 + 0.333*10 = 3.33.
  const double x_final = kf.State()(0);
  EXPECT_NEAR(x_final, 3.33, 0.05);
}

TEST(MeasurementFusionTest, TapeAndLaser) {
  // In this test we fuse two measurements of a length:
  // - Tape measure: reading 100.0 with standard deviation sigma_tape = 3.0 (variance = 9)
  // - Laser: reading 102.0 with standard deviation sigma_laser = 1.0 (variance = 1)
  // The optimal fusion should yield:
  //   weight for tape = 1/9 / (1/9 + 1) = 0.1,
  //   weight for laser = 1 / (1/9 + 1) = 0.9,
  // and thus a fused estimate: 0.1*100 + 0.9*102 = 101.8.
  // Strictly speaking, KF is not needed for a simple to fuse
  // two static measurements. You could just need to compute
  // the weighted average using the inverse variances.


  constexpr int n = 1;  // one-dimensional state (the measured length)
  constexpr int m = 1;  // one-dimensional measurement
  constexpr double dt = 1.0;  // time step (arbitrary for static fusion)

  // Define system matrices for a static model: x[k+1] = x[k]
  Eigen::MatrixXd A(n, n);
  Eigen::MatrixXd C(m, n);
  Eigen::MatrixXd Q(n, n);  // process noise covariance (none, for static system)
  Eigen::MatrixXd R(n, n);  // measurement noise covariance
  Eigen::MatrixXd P(n, n);  // estimate error covariance

  A << 1;
  C << 1;
  Q << 0;       // no process noise for a static quantity
  R << 9;       // start with tape measure variance (sigma_tape^2 = 9)
  P << 100;     // high initial uncertainty

  // Construct the Kalman filter
  KalmanFilter kf(dt, A, C, Q, R, P);

  // Initialize the filter; the initial guess can be arbitrary, e.g., 101
  double t = 0.0;
  Eigen::VectorXd x0(n);
  x0 << 101;
  kf.Init(t, x0);

  // First measurement: tape measure reading of 100.0
  Eigen::VectorXd y(m);
  y << 100.0;
  t += dt;
  kf.Update(y);

  // Now update with the laser measurement.
  // For the laser, we assume a measurement variance of 1 (sigma_laser^2 = 1).
  Eigen::MatrixXd R_laser(n, n);
  R_laser << 1;
  // Assume we have a read-only setter for measurement noise covariance:
  kf.SetMeasurementNoiseCovariance(R_laser);

  // Second measurement: laser reading of 102.0
  y << 102.0;
  t += dt;
  kf.Update(y);

  // The final fused estimate should be close to 101.8.
  const double fused_estimate = kf.State()(0);
  EXPECT_NEAR(fused_estimate, 101.8, 0.05);
}

}  // namespace kalman_filter
}  // namespace slam_dunk