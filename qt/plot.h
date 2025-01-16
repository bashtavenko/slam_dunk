#ifndef SLAM_DUNK_QT_PLOT_H_
#define SLAM_DUNK_QT_PLOT_H_
#include <QtGui/QGuiApplication>
#include <QtQml/QQmlApplicationEngine>
#include <memory>
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include <vector>
#include "lidar.h"

namespace slam_dunk {

class Plot {
 public:
  static absl::StatusOr<std::unique_ptr<Plot>> Create(int& argc, char** argv);

  absl::Status Run(const std::vector<ScanResponse>& data);

 private:
  Plot(std::unique_ptr<QGuiApplication> app,
       std::unique_ptr<QQmlApplicationEngine> engine)
      : app_(std::move(app)),
        engine_(std::move(engine)) {};
  std::unique_ptr<QGuiApplication> app_;
  std::unique_ptr<QQmlApplicationEngine> engine_;
};

}  // namespace slam_dunk

#endif  // SLAM_DUNK_QT_PLOT_H_
