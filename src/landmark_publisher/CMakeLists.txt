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
#
cmake_minimum_required(VERSION 3.20)

project(landmark_publisher)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../qpp_reader ${CMAKE_CURRENT_BINARY_DIR}/qpp_reader)

add_executable(landmark_publisher
  landmark_publisher_node.cpp
)

target_link_libraries(landmark_publisher nlohmann_json::nlohmann_json qpp_reader)

target_include_directories(landmark_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(landmark_publisher
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
)

# ===================================================================
# Install
# ===================================================================

install(TARGETS landmark_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
