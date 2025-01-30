// Lidar runner

// Show real-time lidar data
// blaze run //:runner_main -- --usb_port=/dev/ttyUSB0 --visualizer_port=9000
//
// Show saved lidar data
// blaze run //:runner_main -- in_path=testdata/lidar.txtpb
// --visualizer_port=9000
//
// Run lidar and save data in the text proto format
// blaze run //:runner_main -- --usb_port=/dev/ttyUSB0
// --out_path=/tmp/lidar.txtpb

#include <google/protobuf/text_format.h>
#include <fstream>
#include <iostream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/memory/memory.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "lidar.h"
#include "proto/lidar_response.pb.h"
#include "visualizer_client.h"

ABSL_FLAG(std::string, usb_port, "", "USB port");
ABSL_FLAG(int32_t, visualizer_port, 0, "UDP port to connect to the visualizer");
ABSL_FLAG(std::string, out_path, "", "Lidar response data in proto format.");
ABSL_FLAG(std::string, in_path, "",
          "Input path for the file in text proto format.");
ABSL_FLAG(int32_t, baud_rate, 115200, "Default baud rate for A1");

absl::StatusOr<std::string> ConvertScanResponseToTextProtoString(
    const std::vector<slam_dunk::ScanResponse>& scan_response) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  slam_dunk::proto::ScanResponse proto_response;
  for (const auto& item : scan_response) {
    if (item.theta == 0 || item.distance_mm == 0) continue;
    auto p = proto_response.add_items();
    p->set_theta(item.theta);
    p->set_distance_mm(item.distance_mm);
    p->set_quality(item.quality);
    p->set_flag(item.flag);
  }
  std::string proto_text;
  if (!google::protobuf::TextFormat::PrintToString(proto_response,
                                                   &proto_text)) {
    return absl::InternalError("Failed in TextFormat::PrintToString");
  }
  return proto_text;
}

// Gets one scan and saves response into file with
// text proto format.
absl::Status ScanAndSaveResponse(slam_dunk::Lidar& lidar) {
  auto scan_data = lidar.Scan();
  if (!scan_data.ok()) return scan_data.status();
  auto data = ConvertScanResponseToTextProtoString(scan_data.value());
  if (!data.ok()) return data.status();

  std::ofstream output_file(absl::GetFlag(FLAGS_out_path));
  if (!output_file) {
    return absl::InternalError(
        absl::StrCat("Failed to write to ", absl::GetFlag(FLAGS_out_path)));
  }
  output_file << data.value();
  output_file.close();
  return absl::OkStatus();
}

absl::Status VisualizeFromFile(slam_dunk::VisualizerClient& client) {
  std::ifstream file(absl::GetFlag(FLAGS_in_path));
  std::string data((std::istreambuf_iterator<char>(file)),
                   std::istreambuf_iterator<char>());

  if (data.empty()) {
    return absl::InternalError("Failed to open data file " +
                               absl::GetFlag(FLAGS_in_path));
  }

  auto result = client.SendData(data);
  if (!result.has_value())
    return absl::InternalError("Failed to send data Visualizer");
  LOG(INFO) << "Sent data to Visualizer: " << result.value();
  return absl::OkStatus();
}

absl::Status ShowRealTimeData(slam_dunk::Lidar& lidar,
                              slam_dunk::VisualizerClient& client) {
  while (1) {
    auto scan_data = lidar.Scan();
    if (!scan_data.ok()) return scan_data.status();
    auto data = ConvertScanResponseToTextProtoString(scan_data.value());
    if (!data.ok()) return data.status();
    auto result = client.SendData(data.value());
    if (!result.has_value())
      return absl::InternalError("Failed to send data to visualizer");
  }
  return absl::OkStatus();
}

int main(int argc, char** argv) {
  using slam_dunk::Lidar;
  using slam_dunk::VisualizerClient;
  google::InitGoogleLogging(*argv);
  absl::ParseCommandLine(argc, argv);
  gflags::SetCommandLineOption("logtostderr", "1");

  auto client = VisualizerClient::Create(absl::GetFlag(FLAGS_visualizer_port));
  absl::Status status;

  // From saved file to visualizer
  if (!absl::GetFlag(FLAGS_in_path).empty() &&
      absl::GetFlag(FLAGS_visualizer_port) != 0) {
    status = VisualizeFromFile(*client->get());
  }
  if (!status.ok()) {
    LOG(ERROR) << status.message();
    return EXIT_FAILURE;
  }

  // From lidar to either visualization or saving data
  if (!absl::GetFlag(FLAGS_usb_port).empty() &&
      (!absl::GetFlag(FLAGS_out_path).empty() ||
       absl::GetFlag(FLAGS_visualizer_port) != 0)) {
    auto lidar_status = Lidar::Create(absl::GetFlag(FLAGS_usb_port),
                                      absl::GetFlag(FLAGS_baud_rate));
    if (!lidar_status.ok()) {
      LOG(ERROR) << lidar_status.status();
      return EXIT_FAILURE;
    }
    auto& lidar = lidar_status.value();

    auto result = lidar->Scan();
    if (!result.ok()) {
      LOG(ERROR) << result.status();
      return EXIT_FAILURE;
    }
    auto device_info = lidar->GetDeviceInfo();
    LOG(INFO) << absl::StreamFormat(
        "Model: %s Firmware: %s Hardware: %s Serial: %s", device_info.model,
        device_info.firmware, device_info.hardware, device_info.serial_number);

    // Show real-time data
    if (absl::GetFlag(FLAGS_visualizer_port) != 0) {
      auto show_status = ShowRealTimeData(*lidar.get(), *client->get());
      if (!show_status.ok()) {
        LOG(ERROR) << show_status.message();
        return EXIT_FAILURE;
      }
    }

    // Saving one scan
    if (!absl::GetFlag(FLAGS_out_path).empty()) {
      auto status = ScanAndSaveResponse(*lidar.get());
      if (!status.ok()) {
        LOG(ERROR) << status.message();
        return EXIT_FAILURE;
      }
    }
  }

  LOG(INFO) << "Done.";
  return EXIT_SUCCESS;
}