#include "visualizer_client.h"
#include "absl/memory/memory.h"

namespace slam_dunk {

absl::StatusOr<std::unique_ptr<VisualizerClient>> VisualizerClient::Create(
    int32_t port) {
  auto client = absl::WrapUnique(new VisualizerClient);
  client->socket_id_ = socket(/*domain=*/AF_INET,
                              /*type=*/SOCK_DGRAM,
                              /*protocol=*/0);
  if (client->socket_id_ < 0) {
    return absl::InternalError("Failed to create socket");
  }
  client->server_.sin_family = AF_INET;
  client->server_.sin_port = htons(port);

  int32_t result = inet_pton(AF_INET, kMachine, &client->server_.sin_addr);
  if (result <= 0) {
    close(client->socket_id_);
    return absl::InternalError(absl::StrFormat(
        "Invalid address format %s:%i. Result: %i", kMachine, port, result));
  }
  return client;
}

absl::optional<int32_t> VisualizerClient::SendData(absl::string_view data) {
  ssize_t sent_bytes = sendto(socket_id_, data.data(), data.size(), 0,
                              (struct sockaddr*)&server_, sizeof(server_));
  if (sent_bytes < 0) {
    close(socket_id_);
    return std::nullopt;
  }
  return sent_bytes;
}

}  // namespace slam_dunk