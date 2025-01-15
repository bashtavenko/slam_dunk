#ifndef SLAM_DUNK__LIDAR_H_
#define SLAM_DUNK__LIDAR_H_
#include <memory>
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "third_party/rplidar/include/sl_lidar_driver.h"

namespace slam_dunk {

// Container for one scan of lidar response.
struct ScanResponse {
  // A fixed-point representation of angles, where the angle
  // is encoded in 14 fractional bits.
  // To convert this value to degrees, multiply by 90 and divide by
  // 16384 (or divide by 2^14, i.e. 1 << 14)
  u_int16_t theta;
  // A fixed-point representation of distance in millimeters
  // encoded in 2 fractional bits.
  // float distance_in_meters = node.dist_mm_q2 / 1000.f / (1 << 2);
  uint32_t distance_mm;
  // The quality value reflects the strength and reliability of the laser
  // signal returned from an object.
 //  Bit Composition:
 // The quality data is typically an 8-bit value:
 // Upper 6 bits: Represent the strength of the reflected signal.
 // Lower 2 bits: Indicate the status of the measurement:
 // 01: Marks the start of a new scan (i.e., the first measurement
  // after the 0-degree position).
  // 10: Represents subsequent measurements within the same scan.
  uint8_t quality;
  // It seems that this flag only contains
  // SL_LIDAR_RESP_HQ_FLAG_SYNCBIT to signifies the starting of the scan.
  uint8_t flag;

  bool operator<(const ScanResponse& that) const {
    return theta < that.theta;
  }
};

// Lidar parameters
struct DeviceInfo {
  std::string model;
  std::string firmware;
  std::string hardware;
  std::string serial_number;
};

// Aggregation of Slamtec RPLidar.
class Lidar {
 public:
  // Creates lidar with given parameters
  static absl::StatusOr<std::unique_ptr<Lidar>> Create(
      absl::string_view usb_port, int32_t baud_rate);
  ~Lidar();

  // Returns response for the given number of node (points)
  absl::StatusOr<std::vector<ScanResponse>>Scan(size_t count = 8192);

  // Returns information about initiated lidar.
  DeviceInfo GetDeviceInfo() const;

  // Not copyable
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
