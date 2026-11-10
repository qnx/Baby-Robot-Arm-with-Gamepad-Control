/*
 * Copyright (c) 2026, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef POSE_DATA_H_
#define POSE_DATA_H_

#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>

typedef struct _GtkWidget GtkWidget;

#define MAX_SELECTION_STATE_LEN 32
#define MAX_ROBOT_MODE_LEN 32
#define MAX_SCREENSAVER_MODE_LEN 32
#define MAX_JOINT_NAME_LEN 32
#define MAX_POSE_DETECTED_TEXT_LEN 128
#define MAX_POSES 32
#define MAX_JOINTS_PER_POSE 36
#define MAX_KEYPOINT_EDGES 64

typedef struct {
    int from_index;
    int to_index;
} keypoint_edge_t;

typedef struct {
    char name[MAX_JOINT_NAME_LEN];
    int x;
    int y;
    int tx;
    int ty;
    float confidence;
} joint_info_t;

typedef struct {
    int x;
    int y;
    int width;
    int height;
    int tx;
    int ty;
    int t_width;
    int t_height;
    float confidence;
} person_info_t;

typedef struct {
    int x;
    int y;
    int end_x;
    int end_y;
    int tx;
    int ty;
    int t_end_x;
    int t_end_y;
} roi_info_t;

typedef struct {
    int id;
    person_info_t person;
    char pose_detected_text[MAX_POSE_DETECTED_TEXT_LEN];
    size_t joint_count;
    joint_info_t joints[MAX_JOINTS_PER_POSE];
} pose_info_t;

typedef struct {
    int selected_pose_id;
    char selection_state[MAX_SELECTION_STATE_LEN];
    char qpp_selected_pose[MAX_JOINT_NAME_LEN];
    char qpp_selected_pose_state[MAX_SELECTION_STATE_LEN];
    char robot_mode[MAX_ROBOT_MODE_LEN];
    char screensaver_mode[MAX_SCREENSAVER_MODE_LEN];
    bool has_global_confidence;
    double global_confidence;
    int width;
    int height;
    roi_info_t roi;
    size_t keypoint_edge_count;
    keypoint_edge_t keypoint_edges[MAX_KEYPOINT_EDGES];
    size_t pose_count;
    pose_info_t poses[MAX_POSES];
} resource_info_t;

typedef struct {
    resource_info_t info;
    pthread_t thread;
    pthread_mutex_t mutex;
    int wake_pipe[2];
    bool running;
    bool stopping;
    bool redraw_pending;
    bool hand_info_loaded;
    int hand_info_width;
    int hand_info_height;
    size_t hand_info_edge_count;
    keypoint_edge_t hand_info_edges[MAX_KEYPOINT_EDGES];
    GtkWidget *overlay;
} pose_data_t;

/* Prepare the QPP pose reader without starting background work yet. */
void pose_data_init(pose_data_t *pose_data);

/* Start the background reader thread and attach redraw notifications. */
bool pose_data_start(pose_data_t *pose_data, GtkWidget *overlay);

/* Stop the reader thread and close any open data source. */
void pose_data_stop(pose_data_t *pose_data);

/* Copy the latest parsed pose snapshot into `info_out`. */
void pose_data_get_info(pose_data_t *pose_data, resource_info_t *info_out);

#endif
