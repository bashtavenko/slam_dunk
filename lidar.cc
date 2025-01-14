#include "lidar.h"
#include <algorithm>
#include "absl/memory/memory.h"

namespace slam_dunk {

Lidar::Lidar(std::unique_ptr<sl::ILidarDriver> driver,
             std::unique_ptr<sl::IChannel> channel,
             const sl_lidar_response_device_info_t device_info)
    : driver_(std::move(driver)),
      channel_(std::move(channel)),
      device_info_(device_info) {}

Lidar::~Lidar() {
  driver_->stop();
  driver_->disconnect();
}

absl::StatusOr<std::unique_ptr<Lidar>> Lidar::Create(absl::string_view usb_port,
                                                     int32_t baud_rate) {
  auto driver_result = sl::createLidarDriver();
  if (!driver_result)
    return absl::InternalError("Failed in sl::createLidarDriver()");
  std::unique_ptr<sl::ILidarDriver> driver =
      absl::WrapUnique(driver_result.value);

  auto channel_status = sl::createSerialPortChannel(usb_port.data(), baud_rate);
  if (!channel_status)
    return absl::InternalError("Failed in sl::createSerialPortChannel");
  std::unique_ptr<sl::IChannel> channel =
      absl::WrapUnique(channel_status.value);

  sl_result status = SL_RESULT_OK;
  status = driver->connect(channel.get());
  if (SL_IS_FAIL(status)) {
    return absl::InternalError(
        absl::StrFormat("Failed to connect: 0%x", status));
  }
  sl_lidar_response_device_info_t device_info;
  status = driver->getDeviceInfo(device_info);
  if (SL_IS_FAIL(status)) {
    return absl::InternalError(
        absl::StrFormat("Failed to getDeviceInfo: 0%x", status));
  }

  std::vector<sl::LidarScanMode> scan_modes;
  driver->getAllSupportedScanModes(scan_modes);
  if (scan_modes.empty()) {
    return absl::InternalError("No supported scan modes.");
  }
  driver->startScan(  /*force=*/false, scan_modes[0].id);

  return absl::WrapUnique(
      new Lidar(std::move(driver), std::move(channel), device_info));
}

absl::StatusOr<std::vector<ScanResponse>> Lidar::Scan(size_t count) {
  auto nodes = std::vector<sl_lidar_response_measurement_node_hq_t>(count);
  sl_result status = driver_->grabScanDataHq(&nodes[0], count);
  if (SL_IS_FAIL(status)) {
    return absl::InternalError(
        absl::StrFormat("Failed to grabScanDataHq: 0%x", status));
  }
  auto response = std::vector<ScanResponse>(count);
  for (size_t i = 0; i < count; ++i) {
    response.emplace_back(ScanResponse{
        .theta = nodes[i].angle_z_q14,
        .distance_mm = nodes[i].dist_mm_q2,
        .quality = nodes[i].quality,
        .flag = nodes[i].flag,
    });
  }
  std::sort(response.begin(), response.end());
  return response;
}

}  // namespace slam_dunk