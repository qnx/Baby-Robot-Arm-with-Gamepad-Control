#
# Copyright (c) 2026, BlackBerry Limited. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

licenses(["notice"])

package(default_visibility = ["//mediapipe/__subpackages__"])

load(
    "//mediapipe/framework/tool:mediapipe_files.bzl",
    "mediapipe_files",
)

mediapipe_files(
    srcs = [
        "gesture_recognizer.task"
    ]
)

cc_binary(
    name = "handlandmark_task",
    srcs = [
        "handlandmark_task.cc",
        "handlandmark_task.h",
        "main.cc",
        "qpp_reader.cc",
        "qpp_reader.h",
        "rpi_cam.cc",
        "rpi_cam.h",
    ],
    includes = ["."],
    deps = [
        "//mediapipe/tasks/cc/vision/gesture_recognizer:gesture_recognizer",
        "//mediapipe/tasks/cc/vision/hand_landmarker:hand_landmarker",
        "//mediapipe/tasks/cc/vision/core:running_mode",
        "//mediapipe/tasks/cc/core:base_options",
        "//mediapipe/framework/formats:image",
        "//mediapipe/framework/formats:image_frame",
        "//mediapipe/framework/formats:image_frame_opencv",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/status:statusor",
        "@com_google_absl//absl/flags:flag",
        "@com_google_absl//absl/flags:parse",
        "@com_google_absl//absl/flags:usage",
        "@com_google_absl//absl/log:initialize",
        "@com_google_absl//absl/log:globals",
        "@com_google_absl//absl/log:absl_check",
        "@com_google_absl//absl/log:absl_log",
        "//third_party:opencv",
        "@com_github_nlohmann_json//:json"
    ],
    data = [
        "gesture_recognizer.task",
    ],
    linkopts = [
        "-lcamapi",
        "-lscreen",
    ]
)