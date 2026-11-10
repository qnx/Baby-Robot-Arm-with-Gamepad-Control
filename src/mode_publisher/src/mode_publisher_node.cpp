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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

class ModePublisher : public rclcpp::Node {
public:
  ModePublisher() : Node("mode_publisher_node") {
    this->declare_parameter<std::string>("qpp_base_path", "/dev/qpp/physicalai/bra");
    path_ = this->get_parameter("qpp_base_path").as_string();

    std::string mode_path = path_ + "/mode";
    std::string screensaver_path = path_ + "/screensaver";

    fd_mode_ = open(mode_path.c_str(), O_WRONLY);
    if (fd_mode_ == -1) {
      RCLCPP_FATAL(get_logger(), "open(%s) failed with: %s", mode_path.c_str(), strerror(errno));
      throw std::runtime_error("QPP open failed");
    }
    RCLCPP_INFO(get_logger(), "Opened %s", mode_path.c_str());

    fd_screensaver_ = open(screensaver_path.c_str(), O_WRONLY);
    if (fd_screensaver_ == -1) {
      RCLCPP_FATAL(get_logger(), "open(%s) failed with: %s", screensaver_path.c_str(), strerror(errno));
      throw std::runtime_error("QPP open failed");
    }
    RCLCPP_INFO(get_logger(), "Opened %s", screensaver_path.c_str());

    sub_ = create_subscription<std_msgs::msg::String>(
        "/mode", 10, std::bind(&ModePublisher::mode_callback, this, std::placeholders::_1));
  }

  ~ModePublisher() {
    if (fd_mode_ >= 0)
      (void)close(fd_mode_);
    if (fd_screensaver_ >= 0)
      (void)close(fd_screensaver_);
  }

private:
  void mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    const std::string &data = msg->data;

    if (data.rfind("screensaver:", 0) == 0) {
      // Handle screensaver command
      std::string pattern = data.substr(strlen("screensaver:"));
      err = write(fd_mode_, "screensaver", strlen("screensaver") + 1);
      if (err == -1UL) {
        RCLCPP_ERROR(get_logger(), "write to mode failed with: %s", strerror(errno));
      }
      err = write(fd_screensaver_, pattern.c_str(), pattern.length() + 1);
      if (err == -1UL) {
        RCLCPP_ERROR(get_logger(), "write to screensaver failed with: %s", strerror(errno));
      }
    } else {
      // Handle mode command
      err = write(fd_screensaver_, "", 1);
      if (err == -1UL) {
        RCLCPP_ERROR(get_logger(), "write to screensaver failed with: %s", strerror(errno));
      }
      err = write(fd_mode_, data.c_str(), data.length() + 1);
      if (err == -1UL) {
        RCLCPP_ERROR(get_logger(), "write to mode failed with: %s", strerror(errno));
      }
    }
  }

  std::string path_;
  int fd_mode_ = -1;
  int fd_screensaver_ = -1;
  size_t err = 0;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModePublisher>());
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}