#include "qt/plot.h"
#include "absl/memory/memory.h"

namespace slam_dunk {

absl::StatusOr<std::unique_ptr<Plot>> Plot::Create(int& argc, char** argv) {
  auto app = std::make_unique<QApplication>(argc, argv);
  auto viewer = std::make_unique<QQuickView>();
  viewer->setSource(QUrl::fromLocalFile("plot.qml"));
  return absl::WrapUnique(new Plot(std::move(app), std::move(viewer)));
}

absl::Status Plot::Run(const std::vector<ScanResponse>& data) {
  viewer_->show();
  app_->exec();
  return absl::OkStatus();
}

}  // namespace slam_dunk
