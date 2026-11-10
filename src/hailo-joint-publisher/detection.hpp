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
#pragma once

/**
 * Contains structures and helps relating to the detection results from the hailo controller resource manager
 *
 *
 */

#include <array>
#include <string>

struct Point {
    std::string name;
    int x;
    int y;
    float z;
    float confidence;
};

struct Box {
    int x;
    int y;
    int end_x;
    int end_y;
};

static constexpr size_t num_joints_ = 17;

struct Person {
    int id;  // ID associated with the pose
    float confidence;
    Box box;
    std::array<Point, num_joints_> key_points;
};
