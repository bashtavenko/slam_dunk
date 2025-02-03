#include "proto_utils.h"
#include <google/protobuf/text_format.h>
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"
#include "lidar.h"
#include "proto/lidar_response.pb.h"

namespace slam_dunk {
namespace {

TEST(ScanResponseToTextProtoString, Works) {
  auto text_proto =
      ConvertScanResponseToTextProtoString(std::vector<ScanResponse>{
          ScanResponse{.theta = 5566, .distance_mm = 2257, .quality = 60},
          ScanResponse{.theta = 5822, .distance_mm = 2243, .quality = 60}});

  // Convert from proto text format to actual proto message
  slam_dunk::proto::ScanResponse message;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text_proto.value(),
                                                            &message));
}

}  // namespace
}  // namespace slam_dunk
