/**
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <signal.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "absl/log/absl_log.h"
#include "absl/log/globals.h"
#include "absl/log/initialize.h"
#include "handlandmark_task.h"

ABSL_FLAG(long, camera_unit, (long)-1,
          "The camera unit to open."
          "Set in a .conf file passed to sensor's -c argument at boot.");
ABSL_FLAG(uint32_t, num_hands, 2, "Number of hands to detect.");
ABSL_FLAG(std::string, qpp_path, "/dev/qpp", "Path to QPP.");
ABSL_FLAG(
    std::string, model_path,
    "mediapipe/handlandmark_task/gesture_recognizer.task",
    "Path to the gesture_recognizer.task model bundle. The default is the "
    "runfiles-relative path, so it resolves when launched via bazel run.");
ABSL_FLAG(bool, verbose, false, "Prints output to CLI");
ABSL_FLAG(bool, publish_gestures, true,
          "Publish recognised gestures in each hand's \"pose_detected\". The "
          "gesture model ships in the same bundle as the landmark models and "
          "runs regardless, so turning this off only suppresses the output.");

namespace {
volatile sig_atomic_t g_running = 1;

void on_signal(int /*sig*/) { g_running = 0; }
}  // namespace

/**
 * Startup will do the following
 * 1) Connect to QPP
 * 2) Get the camera going
 * 4) Setup init QPP state
 * 3) Get mediapipe task running
 * 5) Start running state publishing hand landmarks
 *
 * @note Only "info" is published to QPP so far. Per-frame landmarks still go
 *       to stdout; see the detection paths in handlandmark_task.cc.
 */
int main(int argc, char **argv) {
  absl::SetProgramUsageMessage(
      "Detect hand landmarks from the Raspberry Pi camera.\n"
      "Publishes the stream description to <qpp_path>/physicalai/hands/info; "
      "per-frame landmarks currently go to stdout.");
  absl::ParseCommandLine(argc, argv);
  absl::InitializeLog();
  // Line-buffer stdout so landmark lines appear as they happen even when
  // redirected to a file or a pipe, where it would otherwise be block-buffered
  // and hold ~4KB back. Saves an fflush() after every result.
  setvbuf(stdout, nullptr, _IOLBF, 0);
  // absl defaults --stderrthreshold to ERROR, which swallows every
  // ABSL_LOG(INFO). Lower it so the startup and publish lines are actually
  // visible.
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);

  const long camera_unit_long = absl::GetFlag(FLAGS_camera_unit);
  if (camera_unit_long < 0) {
    ABSL_LOG(ERROR) << "--camera_unit is required (see the sensor .conf passed "
                       "to -c at boot for the unit number)";
    return 1;
  }

  const std::string model_path = absl::GetFlag(FLAGS_model_path);
  if (model_path.empty() || !std::filesystem::exists(model_path)) {
    ABSL_LOG(ERROR) << "--model_path does not exist: '" << model_path
                    << "' (the default is runfiles-relative; pass an absolute "
                       "path to gesture_recognizer.task)";
    return 1;
  }

  LandmarkFinder::Options options{};
  options.camera_unit = static_cast<camera_unit_t>(camera_unit_long);
  options.model_path = model_path;
  options.qpp_path = absl::GetFlag(FLAGS_qpp_path);
  options.num_hands = static_cast<int>(absl::GetFlag(FLAGS_num_hands));
  options.verbose = absl::GetFlag(FLAGS_verbose);
  options.publish_gestures = absl::GetFlag(FLAGS_publish_gestures);

  LandmarkFinder finder;
  int err = finder.init(options);
  if (err != EOK) {
    ABSL_LOG(ERROR) << "init failed: " << strerror(err);
    return 1;
  }

  err = finder.start();
  if (err != EOK) {
    ABSL_LOG(ERROR) << "start failed: " << strerror(err);
    return 1;
  }

  // TODO change to a sigwait()
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  ABSL_LOG(INFO) << "Running, ctrl-c to stop.";

  // Detection runs on the camera and MediaPipe threads; nothing to do here.
  while (g_running) {
    pause();
  }

  ABSL_LOG(INFO) << "Stopping.";
  finder.stop();
  return 0;
}
