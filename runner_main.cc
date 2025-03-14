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

#include <fstream>
#include <iostream>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/memory/memory.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "lidar.h"
#include "proto/lidar_response.pb.h"
#include "proto_utils.h"
#include "status_macros.h"
#include "visualizer_client.h"

ABSL_FLAG(std::string, usb_port, "", "USB port");
ABSL_FLAG(int32_t, visualizer_port, 0, "UDP port to connect to the visualizer");
ABSL_FLAG(std::string, out_path, "", "Lidar response data in proto format.");
ABSL_FLAG(std::string, in_path, "",
          "Input path for the file in text proto format.");
ABSL_FLAG(int32_t, baud_rate, 115200, "Default baud rate for A1");

// Gets one scan and saves response into file with
// text proto format.
absl::Status ScanAndSaveResponse(slam_dunk::Lidar& lidar) {
  auto scan_data = lidar.Scan();
  RETURN_IF_ERROR(scan_data.status());
  RETURN_IF_ERROR(
      slam_dunk::SaveToFile(scan_data.value(), absl::GetFlag(FLAGS_out_path)));
}

absl::Status VisualizeFromFile(slam_dunk::VisualizerClient& client) {
  ASSIGN_OR_RETURN(auto file_data,
                   slam_dunk::GetTextFromFile(absl::GetFlag(FLAGS_in_path)));
  const auto result = client.SendData(file_data.data());
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
    ASSIGN_OR_RETURN(auto data,
                     ConvertScanResponseToTextProtoString(scan_data.value()));
    if (auto result = client.SendData(data); !result.has_value())
      return absl::InternalError("Failed to send data to visualizer");
  }
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
    const auto& lidar = lidar_status.value();

    if (const auto result = lidar->Scan(); !result.ok()) {
      LOG(ERROR) << result.status();
      return EXIT_FAILURE;
    }
    auto [model, firmware, hardware, serial_number] = lidar->GetDeviceInfo();
    LOG(INFO) << absl::StreamFormat(
        "Model: %s Firmware: %s Hardware: %s Serial: %s", model, firmware,
        hardware, serial_number);

    // Show real-time data
    if (absl::GetFlag(FLAGS_visualizer_port) != 0) {
      if (auto show_status = ShowRealTimeData(*lidar.get(), *client->get());
          !show_status.ok()) {
        LOG(ERROR) << show_status.message();
        return EXIT_FAILURE;
      }
    }

    // Saving one scan
    if (!absl::GetFlag(FLAGS_out_path).empty()) {
      status = ScanAndSaveResponse(*lidar.get());
      if (!status.ok()) {
        LOG(ERROR) << status.message();
        return EXIT_FAILURE;
      }
    }
  }

  LOG(INFO) << "Done.";
  return EXIT_SUCCESS;
}