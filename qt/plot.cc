#include "qt/plot.h"
#include "absl//strings/string_view.h"
#include "absl/memory/memory.h"

namespace slam_dunk {

absl::StatusOr<std::unique_ptr<Plot>> Plot::Create(int& argc, char** argv) {
  auto app = std::make_unique<QGuiApplication>(argc, argv);
  auto engine = std::make_unique<QQmlApplicationEngine>("plot.qml");
  return absl::WrapUnique(new Plot(std::move(app), std::move(engine)));
}

absl::Status Plot::Run(const std::vector<ScanResponse>& data) {
  app_->exec();
  return absl::OkStatus();
}

}  // namespace slam_dunk
