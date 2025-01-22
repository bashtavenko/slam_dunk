#ifndef SLAM_DUNK_QT_PLOT_H_
#define SLAM_DUNK_QT_PLOT_H_
#include <QtQuick/QQuickView>
#include <QtWidgets/QApplication>
#include <memory>
#include <vector>
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "lidar.h"

namespace slam_dunk {

// Shows and displays lidar data
class Plot {
 public:
  static absl::StatusOr<std::unique_ptr<Plot>> Create(int& argc, char** argv);

  absl::Status Run(const std::vector<ScanResponse>& data);

 private:
  Plot(std::unique_ptr<QApplication> app, std::unique_ptr<QQuickView> viewer)
      : app_(std::move(app)), viewer_(std::move(viewer)) {};
  std::unique_ptr<QApplication> app_;
  std::unique_ptr<QQuickView> viewer_;
};

}  // namespace slam_dunk

#endif  // SLAM_DUNK_QT_PLOT_H_
