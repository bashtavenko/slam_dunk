#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "third_party/rplidar/include/sl_lidar_driver.h"

ABSL_FLAG(std::string, usb_port, "/dev/ttyUSB0", "USB port");
ABSL_FLAG(int32_t, baud_rate, 115200, "Default baud rate for A1");

int Run() {
  auto driver_result = sl::createLidarDriver();
  CHECK(driver_result);
  std::unique_ptr<sl::ILidarDriver> driver(driver_result.value);
  CHECK(driver);

  auto channel_status = sl::createSerialPortChannel(
      absl::GetFlag(FLAGS_usb_port), absl::GetFlag(FLAGS_baud_rate));
  CHECK(channel_status);
  std::unique_ptr<sl::IChannel> channel(channel_status.value);
  CHECK(channel);

  sl_result status = SL_RESULT_OK;
  status = driver->connect(channel.get());
  if (SL_IS_FAIL(status)) {
    LOG(ERROR) << absl::StrFormat("Failed to connect: 0%x", status);
    return EXIT_FAILURE;
  }
  sl_lidar_response_device_info_t device_info;
  status = driver->getDeviceInfo(device_info);
  if (SL_IS_FAIL(status)) {
    LOG(ERROR) << absl::StrFormat("Failed to getDeviceInfo: 0%x", status);
    return EXIT_FAILURE;
  }

  driver->startScan(/*force=*/false, /*useTypicalScan=*/true);
  sl_lidar_response_measurement_node_hq_t nodes[1000];
  size_t count = sizeof(nodes) / sizeof(nodes[0]);
  status = driver->grabScanDataHq(nodes, count);
  if (SL_IS_FAIL(status)) {
    LOG(ERROR) << absl::StrFormat("Failed to grabScanDataHq: 0%x", status);
    return EXIT_FAILURE;
  }
  driver->ascendScanData(nodes, count);
  for (const auto& node : nodes) {
    LOG(INFO) << absl::StreamFormat(
        "%s theta: %03.2f Dist: %08.2f Q: %d",
        (node.flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S " : "  ",
        (node.angle_z_q14 * 90.f) / 16384.f, node.dist_mm_q2 / 4.0f,
        node.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
  }
  driver->stop();
  driver->disconnect();
  return EXIT_SUCCESS;
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  absl::ParseCommandLine(argc, argv);
  gflags::SetCommandLineOption("logtostderr", "1");
  return Run();
}