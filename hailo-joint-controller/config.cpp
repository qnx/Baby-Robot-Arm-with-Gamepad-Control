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

#include "config.hpp"

#include <filesystem>
#include <iostream>

#include <getopt.h>

static void print_help(bool short_help) {
    std::cout << "Usage: hailo-joint-controller <hef_pose_model> [hef_depth_model] [options]\n"
              << "Baby Robot Arm joint controller using the RPi AI Hat+ and pose detection\n";
    if (short_help) {
        std::cout << "Use -h/--help to see all options.\n";
        return;
    }

    std::cout
        << "\n"
        << "Options:\n"
        << "  -h,--help               Print this menu and exit.\n"
        << "  -m,--qpp-pose-path <path> Absolute path to the catalog-backed QPP pose branch.\n"
        << "                            (default: /dev/qpp/physicalai/pose)\n"
        << "  -l,--log-level <LEVEL>  Log level for controller logging. Must be one of\n"
        << "                          <trace, debug, info, warn, error>. (default: info)\n"
        << "  -c,--cam-id <int>       Camera id used for pose detection. Note only\n"
        << "                          RPi camera module is tested. (default: 1)\n"
        << "  -t,--ros2-topic <topic> ROS2 topic joint messages will be published on. (default: bra/joints)\n"
        << "  -r,--roi <x,y,w,h>      Starting ROI (region of interest) for the camera as a percent. Changing the ROI\n"
        << "                          with cause the image to be cropped before being fed to the ML model.\n"
        << "                          This is used to limit the detection in a crowed area (default:0,0,100,100)\n";
}

int parse_cli(int argc, char *argv[], HailoJointConfig &config) {
    static const struct option long_options[] = {{"help", no_argument, 0, 'h'},
                                                 {"qpp-pose-path", required_argument, 0, 'm'},
                                                 {"log-level", required_argument, 0, 'l'},
                                                 {"cam-id", required_argument, 0, 'c'},
                                                 {"ros2-topic", required_argument, 0, 't'},
                                                 {"roi", required_argument, 0, 'r'},
                                                 {0, 0, 0, 0}};

    int c = 0;
    int option_index = 0;

    while ((c = getopt_long(argc, (char *const *)argv, "hm:l:i:b:c:t:r:", (const struct option *)&long_options,
                            &option_index))
           != -1) {
        switch (c) {
            case 'h':
                print_help(false);
                exit(0);
            case 'm':
                if (!std::filesystem::path(optarg).is_absolute()) {
                    spdlog::error("--qpp-pose-path argument must be an absolute path.");
                    return EINVAL;
                }
                config.qpp_pose_path = optarg;
                break;
            case 'l':
                config.log_level = spdlog::level::from_str(optarg);
                if (config.log_level == spdlog::level::off) {
                    spdlog::error("Invalid log level {}, must be one of (trace, debug, info, warn, error)");
                    return EINVAL;
                }
                spdlog::set_level(config.log_level);
                spdlog::info("Setting log level to {}", optarg);
                break;
            case 'c':
                try {
                    std::string optstr = optarg;
                    size_t idx;
                    int cam_id = std::stoi(optstr, &idx);
                    if (cam_id < CAMERA_UNIT_1 || cam_id > CAMERA_UNIT_64) {
                        throw std::out_of_range("Invalid range");
                    } else if (idx != optstr.size()) {
                        throw std::invalid_argument("Invalid arg");
                    }

                    config.camera_unit = (camera_unit_t)cam_id;
                    break;
                } catch (const std::invalid_argument &e) {
                    spdlog::error("Invalid number for camera unit must be integer [1,64]");
                    return EINVAL;
                } catch (const std::out_of_range &e) {
                    spdlog::error("Invalid range for camera unit must be integer [1,64]");
                    return ERANGE;
                }
            case 't':
                config.ros2_joint_topic = optarg;
                break;
            case 'r': {
                int x, y, w, h;
                if (sscanf(optarg, "%d,%d,%d,%d", &x, &y, &w, &h) != 4) {
                    spdlog::error("Invalid format for ROI, must be \"%d,%d,%d,%d\" with values from [0,100]");
                    return EINVAL;
                }
                if (x < 0 || x > 100 || y < 0 || y > 100 || w < 0 || w > 100 || h < 0 || y > 100) {
                    spdlog::error("Invalid range for ROI, must be \"%d,%d,%d,%d\" with values from [0,100]");
                    return ERANGE;
                }
                config.region_of_interest = {static_cast<uint8_t>(x), static_cast<uint8_t>(y), static_cast<uint8_t>(w),
                                             static_cast<uint8_t>(h)};
                break;
            }
            default:
                spdlog::error("Invalid argument or missing argument");
                print_help(true);
                return EINVAL;
        }
    }

    // Get the hef
    if (optind + 1 < (argc - 1)) {
        spdlog::error("Too many arguments.");
        print_help(true);

        return EINVAL;
    }
    if (optind == argc) {
        spdlog::error("Missing hef model path argument.");
        print_help(true);
        return EINVAL;
    }

    config.pose_model_path = argv[optind];
    if (!std::filesystem::exists(config.pose_model_path)) {
        spdlog::error("Hef model({}) doesn't exist", config.pose_model_path);
        return ENOENT;
    }
    optind++;

    if (optind != argc) {
        config.depth_model_path = argv[optind];
        if (!std::filesystem::exists(*config.depth_model_path)) {
            spdlog::error("Hef model({}) doesn't exist", *config.depth_model_path);
            return ENOENT;
        }
    }

    return EOK;
}
