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

#include <deque>

#include "detection.hpp"

/**
 * @brief Represents a joint of the robot keeping track of its position as well as associated points
 *
 *
 */
struct Joint {
    enum class JointType {
        BASE,
        SHOULDER,
        ELBOW,
        WRIST,
        HAND,
        GRIPPER,
        FACE
    } type;

    float angle_rad;        // The current angle of the joint
    float len_sq;           // The length between the 2 joints
    float start_len_sq;     // The length we saw on the first frame
    float start_angle_rad;  // angle at first

    // History of last few points so we can remove outliers
    std::deque<Point> root_points;
    std::deque<Point> stem_points;

    // Every joint is made up a 2 points
    Point root;
    Point stem;

    Point get_average_root() const {
        Point avg_point = {.x = 0, .y = 0, .z = 0, .confidence = 0};

        // Check if we have no point to avoid div by 0
        if (root_points.empty()) {
            return avg_point;
        }

        for (const auto &point : root_points) {
            avg_point.confidence += point.confidence;
            avg_point.x += point.x;
            avg_point.y += point.y;
            avg_point.z += point.z;
        }

        avg_point.confidence /= root_points.size();
        avg_point.x /= root_points.size();
        avg_point.y /= root_points.size();
        avg_point.z /= root_points.size();

        return avg_point;
    }

    Point get_average_stem() const {
        Point avg_point = {.x = 0, .y = 0, .z = 0, .confidence = 0};

        // Check if we have no point to avoid div by 0
        if (root_points.empty()) {
            return avg_point;
        }

        for (const auto &point : stem_points) {
            avg_point.confidence += point.confidence;
            avg_point.x += point.x;
            avg_point.y += point.y;
            avg_point.z += point.z;
        }

        avg_point.confidence /= root_points.size();
        avg_point.x /= root_points.size();
        avg_point.y /= root_points.size();
        avg_point.z /= root_points.size();

        return avg_point;
    }

    float get_2d_angle() const {
        auto avg_root = get_average_root();
        auto avg_stem = get_average_stem();

        float o = avg_stem.y - avg_root.y;
        float a = avg_stem.x - avg_root.x;
        // auto h = std::sqrtf(o * o + a * a);
        return atan(o / a);
    }

    /**
     * @brief Calculates the angle relative to another point
     *
     * @param joint Joint which plane you want to be relative too
     *
     * @return Angle
     */
    float get_2d_angle_from_joint(const Joint &joint) {
        auto avg_root = get_average_root();
        auto avg_stem = get_average_stem();
        auto avg_root_plane = joint.get_average_root();
        auto avg_stem_plane = joint.get_average_stem();

        auto a_x = avg_stem_plane.x - avg_root_plane.x;
        auto a_y = avg_stem_plane.y - avg_root_plane.y;
        float a_mag = sqrtf(a_x * a_x + a_y * a_y);

        auto b_x = avg_stem.x - avg_root.x;
        auto b_y = avg_stem.y - avg_root.y;
        float b_mag = sqrtf(b_x * b_x + b_y * b_y);

        float dot = a_x * b_x + a_y * b_y;

        float angle = acos(dot / (a_mag * b_mag));

        // Use the cross product to find if the angle is on the opposite side of the vector (negative size)
        float cross_product = a_x * b_y - a_y * b_x;
        if (cross_product < 0) {
            angle = (2 * M_PI) - angle;
        }
        return angle;
    }
};
