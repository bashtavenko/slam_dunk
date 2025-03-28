#include "kalman_filter.h"
#include <Eigen/Eigen>
#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

namespace slam_dunk {
namespace {

constexpr double kMaxAbsError = 0.1;

using ::testing::DoubleNear;

TEST(KalmanFilter, BasicWorks) {
  const int n = 3;  // Number of states
  const int m = 1;  // Number of measurements

  constexpr double dt = 1.0 / 30;  // Time step

  Eigen::MatrixXd A(n, n);  // System dynamics matrix
  Eigen::MatrixXd C(m, n);  // Output matrix
  Eigen::MatrixXd Q(n, n);  // Process noise covariance
  Eigen::MatrixXd R(m, m);  // Measurement noise covariance
  Eigen::MatrixXd P(n, n);  // Estimate error covariance

  // Discrete LTI projectile motion, measuring position only
  A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
  R << 5;
  P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

  // Construct the filter
  KalmanFilter kf(dt, A, C, Q, R, P);

  // List of noisy position measurements (y)
  const std::vector<double> measurements = {
      1.04202710058,  1.10726790452,  1.2913511148,    1.48485250951,
      1.72825901034,  1.74216489744,  2.11672039768,   2.14529225112,
      2.16029641405,  2.21269371128,  2.57709350237,   2.6682215744,
      2.51641839428,  2.76034056782,  2.88131780617,   2.88373786518,
      2.9448468727,   2.82866600131,  3.0006601946,    3.12920591669,
      2.858361783,    2.83808170354,  2.68975330958,   2.66533185589,
      2.81613499531,  2.81003612051,  2.88321849354,   2.69789264832,
      2.4342229249,   2.23464791825,  2.30278776224,   2.02069770395,
      1.94393985809,  1.82498398739,  1.52526230354,   1.86967808173,
      1.18073207847,  1.10729605087,  0.916168349913,  0.678547664519,
      0.562381751596, 0.355468474885, -0.155607486619, -0.287198661013,
      -0.602973173813};

  // Best guess of initial states
  Eigen::VectorXd x0(n);
  double t = 0;
  // [1.04203 0 -9.81]
  x0 << measurements.front(), 0, -9.81;
  kf.Init(t, x0);

  // Feed measurement into filter
  Eigen::VectorXd y(m);
  for (size_t i = 0; i < measurements.size(); ++i) {
    t += dt;
    y << measurements[i];
    kf.Update(y);
    LOG(INFO) << absl::StreamFormat(
        "t = %.2f y[%i] = %.2f x_hat[%i] = [%.2f, %.2f, %.2f]", t, i,
        y.transpose().x(), i, kf.State().transpose()[0],
        kf.State().transpose()[1], kf.State().transpose()[2]);
  }
  // This should be close to x0 -9.81, I guess.
  EXPECT_THAT(kf.State().transpose()[2], DoubleNear(-9.2, kMaxAbsError));
}

}  // namespace
}  // namespace slam_dunk
