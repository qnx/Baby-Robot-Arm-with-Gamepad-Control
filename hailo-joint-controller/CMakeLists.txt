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
project(hailo-joint-controller CXX)

add_executable(hailo-joint-controller
    hailo-joint-controller-main.cpp
    hailo_joint_controller.cpp
    rpi_cam.cpp
    config.cpp

    # ML
    ml/model_manager.cpp
    ml/pose_detection.cpp
    ml/depth_detection.cpp
    ml/object_detection.cpp
    ml/models/vit_pose_detection.cpp
    ml/models/fast_depth_detection.cpp
    ml/models/resnet_obj_detection.cpp

    # Catalog-backed QPP adapter
    qpp/hailo_joint_qpp.cpp
)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../hailort/install/lib/cmake/")

if (DEFINED QNX_TARGET)
    list(APPEND CMAKE_PREFIX_PATH ${QNX_TARGET}/aarch64le/usr/share/)
endif()

# HailoRT doesn't set HailoRT_FOUND so check for the library
find_package(HailoRT)
if(NOT TARGET HailoRT::libhailort)
    message(FATAL_ERROR "Missing libharlort")
endif()

find_package(OpenCV REQUIRED)

include(FetchContent)
FetchContent_Declare(
      spdlog
      GIT_REPOSITORY https://github.com/gabime/spdlog.git
      GIT_TAG v1.17.0
)
FetchContent_MakeAvailable(spdlog)

target_include_directories(hailo-joint-controller PRIVATE ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hailo-joint-controller HailoRT::libhailort ${OpenCV_LIBS} spdlog::spdlog cjson)

if (${CMAKE_SYSTEM_NAME} STREQUAL QNX)
    target_include_directories(hailo-joint-controller PRIVATE ${QNX_TARGET}/usr/local/include)
    target_link_libraries(hailo-joint-controller camapi sensor)
endif()
