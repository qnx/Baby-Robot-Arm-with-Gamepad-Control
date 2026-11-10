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

cmake_minimum_required(VERSION 3.20)
project(hailo_joint_publisher CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(hailo_joint_publisher
  hailo_joint_publisher.cpp
  hailo-joint-publisher-main.cpp

)
target_link_libraries(hailo_joint_publisher spdlog cjson)
target_include_directories(hailo_joint_publisher PUBLIC
  ${QNX_TARGET}/usr/local/include
)


ament_target_dependencies(hailo_joint_publisher
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  hailo_joint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
