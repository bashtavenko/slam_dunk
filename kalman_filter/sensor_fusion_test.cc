// Sensor fusion of 2D sensor and LiDAR sensor
#include <gtest/gtest.h>
#include "glog/logging.h"
#include "gmock/gmock.h"
#include "kalman_filter/kalman_filter.h"

namespace slam_dunk {
namespace {

TEST(SensorFusionTest, CameraAndLidarAsynchronousFusion) {
  constexpr double dt = 0.1;
  constexpr int32_t state_dim = 4;  // [x, y, vx, vy]
  constexpr int32_t meas_dim = 2;   // [x, y]

  // State transition matrix A (constant velocity model)
  Eigen::MatrixXd A(state_dim, state_dim);
  A << 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, dt, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0;

  // Measurement matrix C (both sensors measure position [x, y])
  Eigen::MatrixXd C(meas_dim, state_dim);
  C << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;

  // Process noise covariance Q
  Eigen::MatrixXd Q(state_dim, state_dim);
  Q.setIdentity();
  Q *= 0.01;

  // Initial estimate error covariance P
  Eigen::MatrixXd P(state_dim, state_dim);
  P.setIdentity();
  P *= 1.0;

  // Measurement noise covariances (Matrices)
  // Camera: Higher noise / uncertainty
  Eigen::MatrixXd R_cam(meas_dim, meas_dim);
  R_cam << 2.0, 0.0, 0.0, 2.0;

  // LiDAR: High precision / very low noise
  Eigen::MatrixXd R_lidar(meas_dim, meas_dim);
  R_lidar << 0.02, 0.0, 0.0, 0.02;

  // Initialize Kalman Filter with initial R_cam
  KalmanFilter kf(dt, A, C, Q, R_cam, P);

  // State prior guess [x=0, y=0, vx=1.5, vy=0.8]
  Eigen::VectorXd x0(state_dim);
  x0 << 0.0, 0.0, 1.5, 0.8;
  kf.Init(0.0, x0);

  Eigen::VectorXd true_state = x0;
  constexpr size_t total_steps = 50;

  for (size_t step = 1; step <= total_steps; ++step) {
    // --- PREDICT ---
    bool pred_success = kf.Predict();
    EXPECT_TRUE(pred_success);

    // Ground truth simulation.
    // Assume we have an actual measurement
    true_state = A * true_state;

    //  CAMERA UPDATE (Every step)
    kf.SetMeasurementNoiseCovariance(R_cam);

    Eigen::Vector2d cam_noise_sample(0.5, -0.4);
    // Measurement = Expected + Noise
    Eigen::VectorXd y_cam = C * true_state + cam_noise_sample;
    bool cam_correct_success = kf.Correct(y_cam);
    EXPECT_TRUE(cam_correct_success);

    //  LIDAR UPDATE (Every 3rd step)
    if (step % 3 == 0) {
      kf.SetMeasurementNoiseCovariance(R_lidar);
      Eigen::Vector2d lidar_noise_sample(0.01, -0.01);
      // Measurement = Expected + Noise
      Eigen::VectorXd y_lidar = C * true_state + lidar_noise_sample;
      bool lidar_correct_success = kf.Correct(y_lidar);
      EXPECT_TRUE(lidar_correct_success);
    }
  }
}
}  // namespace
}  // namespace slam_dunk