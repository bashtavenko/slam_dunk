// Conversions from and to protos
#ifndef SLAM_DUNK__PROTO_UTILS_H_
#define SLAM_DUNK__PROTO_UTILS_H_

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "lidar.h"

namespace slam_dunk {

absl::StatusOr<std::string> Foo();

absl::StatusOr<std::string> ConvertScanResponseToTextProtoString(
    const std::vector<slam_dunk::ScanResponse>& scan_response);

absl::Status SaveToFile(
    const std::vector<slam_dunk::ScanResponse>& scan_response,
    absl::string_view file_path);

absl::StatusOr<std::string> GetTextFromFile(absl::string_view file_path);

}  // namespace slam_dunk

#endif  // SLAM_DUNK__PROTO_UTILS_H_
