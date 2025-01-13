#ifndef SLAM_DUNK__LIDAR_H_
#define SLAM_DUNK__LIDAR_H_
#include <memory>
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "third_party/rplidar/include/sl_lidar_driver.h"

namespace slam_dunk {

struct ScanResponse {
  u_int16_t theta;
  uint32_t distance_mm;
  uint8_t quality;
  uint8_t flag;

  bool operator<(const ScanResponse& that) const {
    return theta < that.theta;
  }
};

class Lidar {
 public:
  static absl::StatusOr<std::unique_ptr<Lidar>> Create(
      absl::string_view usb_port, int32_t baud_rate);
  ~Lidar();

  absl::StatusOr<std::vector<ScanResponse>>Scan(size_t count = 360);

  Lidar(const Lidar&) = delete;
  Lidar& operator=(const Lidar&) = delete;

 private:
  Lidar(std::unique_ptr<sl::ILidarDriver> driver,
        std::unique_ptr<sl::IChannel> channel,
        const sl_lidar_response_device_info_t device_info);
  std::unique_ptr<sl::ILidarDriver> driver_;
  std::unique_ptr<sl::IChannel> channel_;
  sl_lidar_response_device_info_t device_info_;
};

}  // namespace slam_dunk

#endif  // SLAM_DUNK__LIDAR_H_
