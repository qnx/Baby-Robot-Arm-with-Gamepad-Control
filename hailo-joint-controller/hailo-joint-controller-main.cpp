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
#include <algorithm>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

#include <spdlog/spdlog.h>

#include <hailo/hailort.hpp>

#include "config.hpp"
#include "hailo_joint_controller.hpp"
#include "qpp/hailo_joint_qpp.hpp"
#include "util.hpp"

void wait_for_signal(sigset_t signal_set) {
    int sig = 0;
    do {
        (void)sigwait(&signal_set, &sig);
    } while (sigismember(&signal_set, sig) == 0);
    spdlog::info("Exit Requested");
}

int main(int argc, char **argv) {
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGINT);
    sigaddset(&signal_set, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &signal_set, NULL);

    HailoJointConfig cfg;
    int res = parse_cli(argc, argv, cfg);
    if (res != EOK) {
        return res;  // Error message printed in func
    }

    joint::HailoJointController controller(cfg);

    res = controller.init();
    if (res != EOK) {
        spdlog::error("Failed to initialize controller.");
        return res;
    }

    // QPP owns the resource manager. This adapter publishes to and consumes
    // signals which were created from the PhysicalAI signal catalog.
    joint::qpp::HailoJointQpp qpp(cfg.qpp_pose_path, cfg.qpp_bra_path, controller);

    res = qpp.init();
    if (res != EOK) {
        spdlog::error("Failed to initialize QPP adapter at {}.", cfg.qpp_pose_path);
        return res;
    }

    res = qpp.start();
    if (res != EOK) {
        spdlog::error("Failed to start QPP adapter with: {}", strerror(res));
        return res;
    }

    res = controller.start();
    if (res != EOK) {
        spdlog::error("Failed to start joint controller with: {}", strerror(res));
        qpp.stop();
        return res;
    }

    wait_for_signal(signal_set);

    // Stop inference first so no callback can write while QPP descriptors are
    // being shut down.
    controller.stop();
    qpp.stop();

    return 0;
}
