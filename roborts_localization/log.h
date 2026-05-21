/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_LOCALIZATION_LOG_H
#define ROBORTS_LOCALIZATION_LOG_H

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iostream>

#include "glog/logging.h"
#include "glog/raw_logging.h"

namespace roborts_localization {

constexpr const char kLocalizationLoggerName[] = "localization";

inline rclcpp::Logger localization_logger() {
  return rclcpp::get_logger(kLocalizationLoggerName);
}

// GLOG remains for CHECK_*, VLOG, and lifecycle of InitGoogleLogging.
class GLogWrapper {
 public:
  explicit GLogWrapper(char *program) {
    google::InitGoogleLogging(program);
    FLAGS_stderrthreshold = google::WARNING;
    FLAGS_colorlogtostderr = true;
    FLAGS_v = 3;
    google::InstallFailureSignalHandler();
  }

  ~GLogWrapper() {
    google::ShutdownGoogleLogging();
  }
};

}  // namespace roborts_localization

#endif  // ROBORTS_LOCALIZATION_LOG_H
