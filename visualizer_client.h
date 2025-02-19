#ifndef SLAM_DUNK__VISUALIZER_CLIENT_H_
#define SLAM_DUNK__VISUALIZER_CLIENT_H_
#include <arpa/inet.h>
#include <stdint.h>
#include <sys/socket.h>
#include <memory>
#include "absl/status/statusor.h"
#include "absl/types/optional.h"

namespace slam_dunk {

// Encapsulates initializing and sending UDP data to visualizer.
class VisualizerClient {
  static constexpr char kMachine[] = "127.0.0.1";

 public:
  // Creates client for the given port if successful.
  static absl::StatusOr<std::unique_ptr<VisualizerClient>> Create(int32_t port);
  // Sends text data (usually proto in text format)
  absl::optional<int32_t> SendData(absl::string_view data);

  // Not copyable
  VisualizerClient(const VisualizerClient&) = delete;
  VisualizerClient& operator=(const VisualizerClient&) = delete;

 private:
  VisualizerClient() {}
  sockaddr_in server_;
  int32_t socket_id_;
};

}  // namespace slam_dunk

#endif  // SLAM_DUNK__VISUALIZER_CLIENT_H_
