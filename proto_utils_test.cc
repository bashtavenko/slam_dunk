#include "proto_utils.h"
#include <google/protobuf/text_format.h>
#include "absl/status/status_matchers.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"
#include "lidar.h"
#include "proto/lidar_response.pb.h"
#include "protobuf-matchers/protocol-buffer-matchers.h"
#include "tools/cpp/runfiles/runfiles.h"

namespace slam_dunk {
namespace {

using ::absl_testing::IsOk;
using ::absl_testing::IsOkAndHolds;
using ::bazel::tools::cpp::runfiles::Runfiles;
using ::protobuf_matchers::EqualsProto;
using ::testing::HasSubstr;
using ::testing::NotNull;

TEST(ScanResponseToTextProtoString, Works) {
  auto text_proto =
      ConvertScanResponseToTextProtoString(std::vector<ScanResponse>{
          ScanResponse{.theta = 5566, .distance_mm = 2257, .quality = 60},
          ScanResponse{.theta = 5822, .distance_mm = 2243, .quality = 60}});

  // Convert from proto text format to actual proto message
  slam_dunk::proto::ScanResponse message;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(text_proto.value(),
                                                            &message));

  EXPECT_THAT(
      message,
      EqualsProto(
          R"pb(items { theta: 5566 distance_mm: 2257 quality: 60 }
               items { theta: 5822 distance_mm: 2243 quality: 60 })pb"));
}

TEST(SaveAndGetFile, Works) {
  const Runfiles* files = Runfiles::CreateForTest();
  ASSERT_THAT(files, NotNull());
  const std::string test_file = files->Rlocation("lidar.txtpb");
  EXPECT_THAT(
      SaveToFile(
          std::vector<ScanResponse>{
              ScanResponse{.theta = 5566, .distance_mm = 2257, .quality = 60},
              ScanResponse{.theta = 5822, .distance_mm = 2243, .quality = 60}},
          test_file),
      IsOk());
  EXPECT_THAT(GetTextFromFile(test_file), IsOkAndHolds(HasSubstr("items")));
}

}  // namespace
}  // namespace slam_dunk
