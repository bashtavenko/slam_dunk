#include "proto_utils.h"
#include <fstream>
#include "google/protobuf/text_format.h"
#include "proto/lidar_response.pb.h"

namespace slam_dunk {

absl::StatusOr<std::string> ConvertScanResponseToTextProtoString(
    const std::vector<slam_dunk::ScanResponse>& scan_response) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  slam_dunk::proto::ScanResponse proto_response;
  for (const auto& item : scan_response) {
    // Uncomment this line to ignore zeros
    // if (item.theta == 0 || item.distance_mm == 0) continue;
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

absl::Status SaveToFile(
    const std::vector<slam_dunk::ScanResponse>& scan_response,
    absl::string_view file_path) {
  auto data = ConvertScanResponseToTextProtoString(scan_response);
  if (!data.ok()) return data.status();

  std::ofstream output_file(file_path.data());
  if (!output_file) {
    return absl::InternalError(absl::StrCat("Failed to write to ", file_path));
  }

  output_file << data.value();
  output_file.close();
  return absl::OkStatus();
}

absl::StatusOr<std::string> GetTextFromFile(absl::string_view file_path) {
  std::ifstream file(file_path.data());
  std::string data((std::istreambuf_iterator<char>(file)),
                   std::istreambuf_iterator<char>());

  if (data.empty()) {
    return absl::InternalError("Failed to open data file " +
                               std::string(file_path));
  }
  return data;
}

}  // namespace slam_dunk
