#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/memory/memory.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "lidar.h"
#include "proto/lidar_response.pb.h"
#include <fstream>
#include <google/protobuf/text_format.h>

ABSL_FLAG(std::string, usb_port, "/dev/ttyUSB0", "USB port");
ABSL_FLAG(int32_t, baud_rate, 115200, "Default baud rate for A1");
ABSL_FLAG(std::string, out_path, "/tmp/lidar.txtpb", "Lidar response data in proto format." );

absl::Status SaveResponse(const std::vector<slam_dunk::ScanResponse>& scan_response) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  slam_dunk::proto::ScanResponse proto_response;
  for (const auto& item : scan_response) {
    auto p = proto_response.add_items();
    p->set_theta(item.theta);
    p->set_distance_mm(item.distance_mm);
    p->set_quality(item.quality);
    p->set_flag(item.flag);
  }
  std::string text_format;
  if (!google::protobuf::TextFormat::PrintToString(proto_response, &text_format)) {
    return absl::InternalError("Failed in TextFormat::PrintToString");
  }
  std::ofstream output_file(absl::GetFlag(FLAGS_out_path));
  if (!output_file) {
    return absl::InternalError(absl::StrCat("Failed to write to ", absl::GetFlag(FLAGS_out_path)));
  }
  output_file << text_format;
  output_file.close();
  return absl::OkStatus();
}

absl::Status RunLidar() {
  using slam_dunk::Lidar;
  auto lidar = Lidar::Create(absl::GetFlag(FLAGS_usb_port),
                             absl::GetFlag(FLAGS_baud_rate));
  if (!lidar.ok()) return lidar.status();

  auto result = lidar.value()->Scan();
  if (!result.ok()) return result.status();

  auto save_status = SaveResponse(result.value());
  if (!save_status.ok()) return save_status;
  return absl::OkStatus();
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(*argv);
  absl::ParseCommandLine(argc, argv);
  gflags::SetCommandLineOption("logtostderr", "1");

  absl::Status status = RunLidar();
  if (!status.ok()) {
    LOG(ERROR) << status.message();
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}