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

#include "pose_data.h"

#include "app_config.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <gtk/gtk.h>

#include <cjson/cJSON.h>

#define MAX_JSON_SIZE 16384
#define MAX_QPP_TEXT_SNAPSHOTS 32

typedef struct {
    char path[128];
    char value[MAX_JSON_SIZE];
    int fd;
    bool has_value;
} qpp_text_snapshot_t;

static qpp_text_snapshot_t g_qpp_text_snapshots[MAX_QPP_TEXT_SNAPSHOTS];
static pthread_mutex_t g_qpp_text_snapshots_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    char message[256];
} pose_parse_error_t;

typedef enum {
    QPP_DETECTION_INVALID = -1,
    QPP_DETECTION_EMPTY = 0,
    QPP_DETECTION_APPENDED = 1
} qpp_detection_result_t;

/* Convert a timespec into nanoseconds for throttle and timing checks. */
static uint64_t pose_timespec_to_nsec(const struct timespec *ts)
{
    return (uint64_t)ts->tv_sec * 1000 * 1000 * 1000 + (uint64_t)ts->tv_nsec;
}

/* Reset parse error state before processing input. */
static void pose_parse_error_clear(pose_parse_error_t *error)
{
    if (error != NULL) {
        error->message[0] = '\0';
    }
}

/* Initialize overlay data with safe display defaults. */
static void init_default_resource_info(resource_info_t *info)
{
    if (info == NULL) {
        return;
    }

    memset(info, 0, sizeof(*info));
    info->selected_pose_id = -1;
    info->roi.x = 0;
    info->roi.y = 0;
    info->roi.end_x = 100;
    info->roi.end_y = 100;
    strlcpy(info->selection_state, "noone", sizeof(info->selection_state));
}

/* Assign the COCO joint name associated with an index. */
static void set_joint_name_for_index(joint_info_t *joint, size_t index)
{
    if (joint == NULL) {
        return;
    }

    snprintf(joint->name, sizeof(joint->name), "kp%02zu", index);
}

/* Record the first parse error encountered. */
static void pose_parse_error_set(pose_parse_error_t *error, const char *message)
{
    if (error == NULL || message == NULL) {
        return;
    }

    snprintf(error->message, sizeof(error->message), "%s", message);
}

/* Record a formatted parse error when none is set. */
static void pose_parse_error_setf(pose_parse_error_t *error, const char *fmt, ...)
{
    va_list args;

    if (error == NULL || fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    vsnprintf(error->message, sizeof(error->message), fmt, args);
    va_end(args);
}

/* Determine whether a throttled diagnostic may be logged. */
static bool pose_should_log_throttled(uint64_t now_ns,
                                      uint64_t *last_log_time_ns,
                                      int *suppressed_count)
{
    if (last_log_time_ns == NULL || suppressed_count == NULL) {
        return true;
    }

    if (*last_log_time_ns == 0 ||
        now_ns - *last_log_time_ns >= POSE_DATA_ERROR_LOG_THROTTLE_NS) {
        *last_log_time_ns = now_ns;
        *suppressed_count = 0;
        return true;
    }

    (*suppressed_count)++;
    return false;
}

/* Log a throttled warning for invalid QPP input. */
static void pose_log_qpp_input_warning(const char *message)
{
    static uint64_t last_log_time_ns;
    static int suppressed_count;
    struct timespec now;
    uint64_t now_ns;

    if (message == NULL) {
        return;
    }

    clock_gettime(CLOCK_MONOTONIC, &now);
    now_ns = pose_timespec_to_nsec(&now);
    if (pose_should_log_throttled(now_ns, &last_log_time_ns,
                                  &suppressed_count)) {
        printf("WARN: QPP overlay input: %s\n", message);
    }
}

/* Clear the pending redraw flag and repaint on the GTK thread. */
static gboolean pose_data_queue_redraw(gpointer user_data)
{
    pose_data_t *pose_data = user_data;

    pthread_mutex_lock(&pose_data->mutex);
    pose_data->redraw_pending = false;
    pthread_mutex_unlock(&pose_data->mutex);

    if (pose_data->overlay != NULL && !DEBUG_DISABLE_OVERLAY_QUEUE_DRAW) {
        gtk_widget_queue_draw(pose_data->overlay);
    }

    return G_SOURCE_REMOVE;
}

/* Coalesce redraw requests so multiple updates share one idle callback. */
static void pose_data_schedule_redraw(pose_data_t *pose_data)
{
    bool should_schedule = false;

    pthread_mutex_lock(&pose_data->mutex);
    if (!pose_data->redraw_pending) {
        pose_data->redraw_pending = true;
        should_schedule = true;
    }
    pthread_mutex_unlock(&pose_data->mutex);

    if (should_schedule) {
        /* Camera frames are dispatched at G_PRIORITY_HIGH_IDLE.  Use the
         * same priority for this coalesced update so a continuous camera
         * stream cannot starve pose/overlay redraws until the window changes
         * state. */
        g_idle_add_full(G_PRIORITY_HIGH_IDLE, pose_data_queue_redraw,
                        pose_data, NULL);
    }
}

/* Parse detected poses from a JSON array. */
static int parse_pose_detected(const cJSON *pose_detected_json,
                               pose_info_t *pose,
                               pose_parse_error_t *error)
{
    cJSON *gesture_json = NULL;
    bool first_entry = true;
    size_t used_len = 0;

    if (pose == NULL) {
        return -1;
    }

    if (pose_detected_json == NULL) {
        return 0;
    }

    if (!cJSON_IsArray(pose_detected_json)) {
        pose_parse_error_setf(error, "Invalid pose.pose_detected for pose %d",
                              pose->id);
        return -1;
    }

    cJSON_ArrayForEach(gesture_json, pose_detected_json) {
        const char *gesture = NULL;
        size_t gesture_len = 0;

        if (!cJSON_IsString(gesture_json) || gesture_json->valuestring == NULL) {
            pose_parse_error_setf(error,
                                  "Invalid pose.pose_detected entry for pose %d",
                                  pose->id);
            return -1;
        }

        gesture = gesture_json->valuestring;
        gesture_len = strlen(gesture);

        if (!first_entry) {
            if (used_len + 2 >= sizeof(pose->pose_detected_text)) {
                pose_parse_error_setf(error,
                                      "pose.pose_detected text too long for pose %d",
                                      pose->id);
                return -1;
            }

            memcpy(&pose->pose_detected_text[used_len], ", ", 2);
            used_len += 2;
        }

        if (used_len + gesture_len >= sizeof(pose->pose_detected_text)) {
            pose_parse_error_setf(error,
                                  "pose.pose_detected text too long for pose %d",
                                  pose->id);
            return -1;
        }

        memcpy(&pose->pose_detected_text[used_len], gesture, gesture_len);
        used_len += gesture_len;
        pose->pose_detected_text[used_len] = '\0';
        first_entry = false;
    }

    return 0;
}

/* Publish physical QPP data into the shared runtime structure. */
static bool pose_data_is_stopping(pose_data_t *pose_data)
{
    bool stopping;

    pthread_mutex_lock(&pose_data->mutex);
    stopping = pose_data->stopping;
    pthread_mutex_unlock(&pose_data->mutex);

    return stopping;
}

/* Publish a parsed overlay snapshot to readers. */
static void pose_data_publish_info(pose_data_t *pose_data,
                                   const resource_info_t *info)
{
    pthread_mutex_lock(&pose_data->mutex);
    pose_data->info = *info;
    pthread_mutex_unlock(&pose_data->mutex);

    pose_data_schedule_redraw(pose_data);
}

/* Persistent descriptors are used solely as poll() notification sources. */
static bool read_qpp_text_value(const char *path, char *buffer, size_t buffer_size)
{
    int fd;
    ssize_t bytes_read;

    if (path == NULL || buffer == NULL || buffer_size < 2U) {
        return false;
    }
    fd = open(path, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        return false;
    }
    do {
        bytes_read = read(fd, buffer, buffer_size - 1U);
    } while (bytes_read < 0 && errno == EINTR);
    close(fd);
    if (bytes_read <= 0) {
        return false;
    }
    while (bytes_read > 0 && (buffer[bytes_read - 1] == '\n' ||
                              buffer[bytes_read - 1] == '\r' ||
                              buffer[bytes_read - 1] == '\0')) {
        bytes_read--;
    }
    buffer[bytes_read] = '\0';
    return bytes_read > 0;
}

/* Read one consistent text snapshot from an open descriptor. */
static bool read_text_snapshot_once(const char *path,
                                    char *buffer,
                                    size_t buffer_size)
{
    qpp_text_snapshot_t *snapshot = NULL;
    int i;

    if (path == NULL || buffer == NULL || buffer_size < 2U) {
        return false;
    }

    pthread_mutex_lock(&g_qpp_text_snapshots_mutex);
    for (i = 0; i < MAX_QPP_TEXT_SNAPSHOTS; i++) {
        if (g_qpp_text_snapshots[i].path[0] != '\0' &&
            strcmp(g_qpp_text_snapshots[i].path, path) == 0) {
            snapshot = &g_qpp_text_snapshots[i];
            break;
        }
        if (snapshot == NULL && g_qpp_text_snapshots[i].path[0] == '\0') {
            snapshot = &g_qpp_text_snapshots[i];
        }
    }
    if (snapshot == NULL) {
        pthread_mutex_unlock(&g_qpp_text_snapshots_mutex);
        return false;
    }
    if (snapshot->path[0] == '\0') {
        strlcpy(snapshot->path, path, sizeof(snapshot->path));
        snapshot->fd = -1;
    }
    if (snapshot->fd == -1) {
        snapshot->fd = open(path, O_RDONLY | O_NONBLOCK);
    }
    if (read_qpp_text_value(path, snapshot->value, sizeof(snapshot->value))) {
        snapshot->has_value = true;
    }

    if (!snapshot->has_value) {
        pthread_mutex_unlock(&g_qpp_text_snapshots_mutex);
        return false;
    }
    strlcpy(buffer, snapshot->value, buffer_size);
    pthread_mutex_unlock(&g_qpp_text_snapshots_mutex);
    return true;
}

/* Close all cached QPP text snapshot descriptors. */
static void close_qpp_text_snapshots(void)
{
    int i;

    pthread_mutex_lock(&g_qpp_text_snapshots_mutex);
    for (i = 0; i < MAX_QPP_TEXT_SNAPSHOTS; i++) {
        if (g_qpp_text_snapshots[i].path[0] != '\0' &&
            g_qpp_text_snapshots[i].fd != -1) {
            close(g_qpp_text_snapshots[i].fd);
        }
        g_qpp_text_snapshots[i].fd = -1;
        g_qpp_text_snapshots[i].path[0] = '\0';
        g_qpp_text_snapshots[i].has_value = false;
    }
    pthread_mutex_unlock(&g_qpp_text_snapshots_mutex);
}

/* Open the physical-AI descriptors before entering the blocking poll loop. */
static void prepare_physical_qpp_text_snapshots(void)
{
    static const char *const paths[] = {
        PHYSICAL_QPP_ROBOT_MODE_PATH,
        PHYSICAL_QPP_SCREENSAVER_MODE_PATH,
        PHYSICAL_QPP_ROBOT_ROI_PATH,
        PHYSICAL_QPP_HAND_INFO_PATH,
        PHYSICAL_QPP_LEFT_HAND_DETECTION_PATH,
        PHYSICAL_QPP_RIGHT_HAND_DETECTION_PATH,
        PHYSICAL_QPP_HAND_SELECTED_POSE_PATH,
        PHYSICAL_QPP_HAND_SELECTED_POSE_STATE_PATH,
    };
    char ignored[MAX_JSON_SIZE];
    size_t i;

    for (i = 0; i < sizeof(paths) / sizeof(paths[0]); i++) {
        (void)read_text_snapshot_once(paths[i], ignored, sizeof(ignored));
    }
}

/* Parse a QPP JSON snapshot into overlay data. */
static bool parse_qpp_overlay_info_json(const char *json,
                                        pose_data_t *pose_data,
                                        pose_parse_error_t *error)
{
    cJSON *edge_json = NULL;
    cJSON *mapping = NULL;
    cJSON *root = NULL;
    cJSON *height_item = NULL;
    cJSON *width_item = NULL;
    int edge_count = 0;
    int i = 0;

    if (json == NULL || pose_data == NULL) {
        pose_parse_error_set(error, "Invalid QPP overlay info arguments");
        return false;
    }

    root = cJSON_Parse(json);
    if (root == NULL) {
        return false;
    }

    width_item = cJSON_GetObjectItemCaseSensitive(root, "width");
    height_item = cJSON_GetObjectItemCaseSensitive(root, "height");
    mapping = cJSON_GetObjectItemCaseSensitive(root, "keypoints_mapping");

    if (!cJSON_IsNumber(width_item) || !cJSON_IsNumber(height_item) ||
        !cJSON_IsArray(mapping)) {
        pose_parse_error_set(error, "Invalid or missing QPP overlay info payload");
        cJSON_Delete(root);
        return false;
    }
    if (width_item->valueint <= 0 || height_item->valueint <= 0) {
        pose_parse_error_set(error, "QPP overlay dimensions must be positive");
        cJSON_Delete(root);
        return false;
    }

    edge_count = cJSON_GetArraySize(mapping);
    if (edge_count < 0 || edge_count > MAX_KEYPOINT_EDGES) {
        pose_parse_error_setf(error,
                              "Too many QPP overlay keypoint edges (max=%d)",
                              MAX_KEYPOINT_EDGES);
        cJSON_Delete(root);
        return false;
    }

    pose_data->hand_info_width = width_item->valueint;
    pose_data->hand_info_height = height_item->valueint;
    pose_data->hand_info_edge_count = (size_t)edge_count;

    cJSON_ArrayForEach(edge_json, mapping) {
        cJSON *from_item = NULL;
        cJSON *to_item = NULL;

        if (i >= edge_count) {
            break;
        }

        if (!cJSON_IsArray(edge_json) || cJSON_GetArraySize(edge_json) != 2) {
            pose_parse_error_set(error, "Invalid QPP overlay keypoint edge entry");
            cJSON_Delete(root);
            return false;
        }

        from_item = cJSON_GetArrayItem(edge_json, 0);
        to_item = cJSON_GetArrayItem(edge_json, 1);
        if (!cJSON_IsNumber(from_item) || !cJSON_IsNumber(to_item)) {
            pose_parse_error_set(error, "Invalid QPP overlay keypoint edge values");
            cJSON_Delete(root);
            return false;
        }

        pose_data->hand_info_edges[i].from_index = from_item->valueint;
        pose_data->hand_info_edges[i].to_index = to_item->valueint;
        i++;
    }

    pose_data->hand_info_loaded = true;
    cJSON_Delete(root);
    return true;
}

/* Load physical hand-selection data when it is available. */
static bool ensure_physical_hand_info_loaded(pose_data_t *pose_data,
                                             pose_parse_error_t *error)
{
    char *buffer;
    bool parsed;

    if (pose_data == NULL) {
        return false;
    }

    if (pose_data->hand_info_loaded) {
        return true;
    }

    buffer = g_malloc(MAX_JSON_SIZE);
    if (buffer == NULL) {
        pose_parse_error_set(error, "Out of memory reading QPP hands.info");
        return false;
    }

    if (!read_text_snapshot_once(PHYSICAL_QPP_HAND_INFO_PATH,
                                 buffer,
                                 MAX_JSON_SIZE)) {
        g_free(buffer);
        return false;
    }

    parsed = parse_qpp_overlay_info_json(buffer, pose_data, error);
    g_free(buffer);
    return parsed;
}

/* An absent hands.info signal is normal when hand tracking is not publishing. */
static bool physical_hand_overlay_available(pose_data_t *pose_data)
{
    pose_parse_error_t error;

    pose_parse_error_clear(&error);
    if (ensure_physical_hand_info_loaded(pose_data, &error)) {
        return true;
    }

    if (error.message[0] != '\0') {
        pose_log_qpp_input_warning(error.message);
    }
    return false;
}

/* Parse one QPP detection and append it when a hand is present. */
static qpp_detection_result_t append_qpp_detection_json(
    const char *json,
    const pose_data_t *pose_data,
    resource_info_t *info,
    int pose_id,
    pose_parse_error_t *error)
{
    cJSON *bbox = NULL;
    cJSON *item = NULL;
    cJSON *joint_json = NULL;
    cJSON *keypoints = NULL;
    cJSON *root = NULL;
    bool has_bbox = false;
    bool has_confident_keypoint = false;
    int joint_count = 0;
    int stored_joint_count = 0;
    int i = 0;
    int max_x = 0;
    int max_y = 0;
    int min_x = 0;
    int min_y = 0;
    pose_info_t *pose = NULL;

    if (json == NULL || pose_data == NULL || info == NULL) {
        pose_parse_error_set(error, "Invalid QPP detection arguments");
        return QPP_DETECTION_INVALID;
    }

    if (pose_id < 0 || (size_t)pose_id >= MAX_POSES) {
        pose_parse_error_set(error, "Invalid QPP detection pose id");
        return QPP_DETECTION_INVALID;
    }

    root = cJSON_Parse(json);
    if (root == NULL) {
        return QPP_DETECTION_INVALID;
    }

    if (cJSON_IsObject(root) && root->child == NULL) {
        cJSON_Delete(root);
        return QPP_DETECTION_EMPTY;
    }

    pose = &info->poses[pose_id];
    memset(pose, 0, sizeof(*pose));
    pose->id = pose_id;

    bbox = cJSON_GetObjectItemCaseSensitive(root, "bbox");
    keypoints = cJSON_GetObjectItemCaseSensitive(root, "keypoints");
    if (!cJSON_IsArray(keypoints)) {
        pose_parse_error_set(error, "Invalid or missing QPP detection payload");
        cJSON_Delete(root);
        return QPP_DETECTION_INVALID;
    }

    if (cJSON_IsObject(bbox)) {
        has_bbox = true;
        item = cJSON_GetObjectItemCaseSensitive(bbox, "conf");
        if (!cJSON_IsNumber(item)) {
            pose_parse_error_set(error, "Invalid QPP detection bbox.conf");
            cJSON_Delete(root);
            return QPP_DETECTION_INVALID;
        }
        pose->person.confidence = (float)item->valuedouble;

        item = cJSON_GetObjectItemCaseSensitive(bbox, "x");
        if (!cJSON_IsNumber(item)) {
            pose_parse_error_set(error, "Invalid QPP detection bbox.x");
            cJSON_Delete(root);
            return QPP_DETECTION_INVALID;
        }
        pose->person.x = item->valueint;

        item = cJSON_GetObjectItemCaseSensitive(bbox, "y");
        if (!cJSON_IsNumber(item)) {
            pose_parse_error_set(error, "Invalid QPP detection bbox.y");
            cJSON_Delete(root);
            return QPP_DETECTION_INVALID;
        }
        pose->person.y = item->valueint;

        item = cJSON_GetObjectItemCaseSensitive(bbox, "w");
        if (!cJSON_IsNumber(item)) {
            pose_parse_error_set(error, "Invalid QPP detection bbox.w");
            cJSON_Delete(root);
            return QPP_DETECTION_INVALID;
        }
        pose->person.width = item->valueint;

        item = cJSON_GetObjectItemCaseSensitive(bbox, "h");
        if (!cJSON_IsNumber(item)) {
            pose_parse_error_set(error, "Invalid QPP detection bbox.h");
            cJSON_Delete(root);
            return QPP_DETECTION_INVALID;
        }
        pose->person.height = item->valueint;
    }

    joint_count = cJSON_GetArraySize(keypoints);
    if (joint_count < 0) {
        pose_parse_error_set(error, "Invalid QPP detection keypoint count");
        cJSON_Delete(root);
        return QPP_DETECTION_INVALID;
    }

    stored_joint_count = MIN(joint_count, MAX_JOINTS_PER_POSE);
    if (joint_count > MAX_JOINTS_PER_POSE) {
        char warning[128];

        snprintf(warning, sizeof(warning),
                 "QPP detection has %d keypoints; rendering the first %d and ignoring %d",
                 joint_count, MAX_JOINTS_PER_POSE,
                 joint_count - MAX_JOINTS_PER_POSE);
        pose_log_qpp_input_warning(warning);
    }
    pose->joint_count = (size_t)stored_joint_count;

    cJSON_ArrayForEach(joint_json, keypoints) {
        cJSON *conf_item = NULL;
        cJSON *x_item = NULL;
        cJSON *y_item = NULL;

        if (i >= stored_joint_count) {
            break;
        }
        if (!cJSON_IsObject(joint_json)) {
            pose_parse_error_set(error, "Invalid QPP detection keypoint entry");
            cJSON_Delete(root);
            return QPP_DETECTION_INVALID;
        }

        x_item = cJSON_GetObjectItemCaseSensitive(joint_json, "x");
        y_item = cJSON_GetObjectItemCaseSensitive(joint_json, "y");
        conf_item = cJSON_GetObjectItemCaseSensitive(joint_json, "conf");
        if (!cJSON_IsNumber(x_item) || !cJSON_IsNumber(y_item) ||
            !cJSON_IsNumber(conf_item)) {
            pose_parse_error_set(error, "Invalid QPP detection keypoint values");
            cJSON_Delete(root);
            return QPP_DETECTION_INVALID;
        }

        set_joint_name_for_index(&pose->joints[i], (size_t)i);
        pose->joints[i].x = x_item->valueint;
        pose->joints[i].y = y_item->valueint;
        pose->joints[i].confidence = (float)conf_item->valuedouble;
        if (!has_bbox && pose->joints[i].confidence > 0.0f) {
            if (!has_confident_keypoint) {
                min_x = max_x = pose->joints[i].x;
                min_y = max_y = pose->joints[i].y;
                has_confident_keypoint = true;
            } else {
                min_x = MIN(min_x, pose->joints[i].x);
                max_x = MAX(max_x, pose->joints[i].x);
                min_y = MIN(min_y, pose->joints[i].y);
                max_y = MAX(max_y, pose->joints[i].y);
            }
        }
        i++;
    }

    if (!has_bbox && has_confident_keypoint) {
        pose->person.confidence = 1.0f;
        pose->person.x = min_x;
        pose->person.y = min_y;
        pose->person.width = max_x - min_x;
        pose->person.height = max_y - min_y;
    }

    item = cJSON_GetObjectItemCaseSensitive(root, "pose_detected");
    if (parse_pose_detected(item, pose, error) != 0) {
        cJSON_Delete(root);
        return QPP_DETECTION_INVALID;
    }

    if ((size_t)(pose_id + 1) > info->pose_count) {
        info->pose_count = (size_t)(pose_id + 1);
    }

    cJSON_Delete(root);
    return QPP_DETECTION_APPENDED;
}

/* Validate the hand-selection state published by QPP. */
static bool is_valid_physical_hand_selection_state(const char *state)
{
    return state != NULL &&
           (strcmp(state, "noone") == 0 ||
            strcmp(state, "selecting") == 0 ||
            strcmp(state, "selected") == 0);
}

/* Read and validate the physical robot ROI from QPP. */
static bool read_physical_robot_roi_from_qpp(roi_info_t *roi)
{
    char buffer[64];
    char extra;
    int end_x;
    int end_y;
    int x;
    int y;

    if (roi == NULL ||
        !read_text_snapshot_once(PHYSICAL_QPP_ROBOT_ROI_PATH, buffer,
                                 sizeof(buffer)) ||
        sscanf(buffer, "%d,%d,%d,%d %c", &x, &y, &end_x, &end_y,
               &extra) != 4 ||
        x < 0 || x > 100 || y < 0 || y > 100 ||
        end_x < x || end_x > 100 || end_y < y || end_y > 100) {
        return false;
    }

    roi->x = x;
    roi->y = y;
    roi->end_x = end_x;
    roi->end_y = end_y;
    return true;
}

/* Refresh the physical robot ROI from QPP. */
static void refresh_physical_robot_roi_from_qpp(pose_data_t *pose_data)
{
    resource_info_t info;
    roi_info_t roi;

    if (pose_data == NULL || !read_physical_robot_roi_from_qpp(&roi)) {
        return;
    }

    pthread_mutex_lock(&pose_data->mutex);
    info = pose_data->info;
    pthread_mutex_unlock(&pose_data->mutex);

    if (info.roi.x == roi.x && info.roi.y == roi.y &&
        info.roi.end_x == roi.end_x && info.roi.end_y == roi.end_y) {
        return;
    }

    info.roi = roi;
    pose_data_publish_info(pose_data, &info);
}

/* Refresh physical IK joint selection from QPP. */
static void refresh_physical_ik_joint_overlay_from_qpp(pose_data_t *pose_data)
{
    char left_buffer[MAX_JSON_SIZE];
    char right_buffer[MAX_JSON_SIZE];
    char selected_pose[MAX_JOINT_NAME_LEN];
    char selected_pose_state[MAX_SELECTION_STATE_LEN];
    pose_parse_error_t parse_error;
    qpp_detection_result_t detection_result;
    resource_info_t info;
    bool has_left = false;
    bool has_right = false;
    bool has_selected_pose = false;
    int left_pose_id = -1;
    int next_pose_id = 0;
    int right_pose_id = -1;

    if (pose_data == NULL) {
        return;
    }

    init_default_resource_info(&info);
    strlcpy(info.robot_mode, "ik_joint", sizeof(info.robot_mode));
    info.width = pose_data->hand_info_width;
    info.height = pose_data->hand_info_height;
    (void)read_physical_robot_roi_from_qpp(&info.roi);
    info.keypoint_edge_count = pose_data->hand_info_edge_count;
    memcpy(info.keypoint_edges,
           pose_data->hand_info_edges,
           sizeof(keypoint_edge_t) * pose_data->hand_info_edge_count);

    has_left = read_text_snapshot_once(PHYSICAL_QPP_LEFT_HAND_DETECTION_PATH,
                                       left_buffer,
                                       sizeof(left_buffer));
    has_right = read_text_snapshot_once(PHYSICAL_QPP_RIGHT_HAND_DETECTION_PATH,
                                        right_buffer,
                                        sizeof(right_buffer));

    pose_parse_error_clear(&parse_error);
    if (has_left) {
        detection_result = append_qpp_detection_json(
            left_buffer, pose_data, &info, next_pose_id, &parse_error);
        if (detection_result == QPP_DETECTION_INVALID) {
            pose_log_qpp_input_warning(
                parse_error.message[0] != '\0' ? parse_error.message :
                "could not parse left hand detection JSON");
        } else if (detection_result == QPP_DETECTION_APPENDED) {
            left_pose_id = next_pose_id;
            next_pose_id++;
        }
    }

    pose_parse_error_clear(&parse_error);
    if (has_right) {
        detection_result = append_qpp_detection_json(
            right_buffer, pose_data, &info, next_pose_id, &parse_error);
        if (detection_result == QPP_DETECTION_INVALID) {
            pose_log_qpp_input_warning(
                parse_error.message[0] != '\0' ? parse_error.message :
                "could not parse right hand detection JSON");
        } else if (detection_result == QPP_DETECTION_APPENDED) {
            right_pose_id = next_pose_id;
            next_pose_id++;
        }
    }

    if (read_text_snapshot_once(PHYSICAL_QPP_HAND_SELECTED_POSE_STATE_PATH,
                                selected_pose_state,
                                sizeof(selected_pose_state))) {
        strlcpy(info.qpp_selected_pose_state, selected_pose_state,
                sizeof(info.qpp_selected_pose_state));
        if (is_valid_physical_hand_selection_state(selected_pose_state)) {
            strlcpy(info.selection_state, selected_pose_state,
                    sizeof(info.selection_state));
        } else {
            pose_log_qpp_input_warning(
                "selected_pose_state must be noone, selecting, or selected");
        }
    }

    has_selected_pose = read_text_snapshot_once(
        PHYSICAL_QPP_HAND_SELECTED_POSE_PATH, selected_pose, sizeof(selected_pose));
    if (has_selected_pose) {
        strlcpy(info.qpp_selected_pose, selected_pose,
                sizeof(info.qpp_selected_pose));
    }

    if (strcmp(info.selection_state, "noone") != 0 && !has_selected_pose) {
        pose_log_qpp_input_warning(
            "selected_pose_state requires selected_pose to name a detected hand");
    } else if (strcmp(info.selection_state, "noone") != 0) {
        if (strcmp(selected_pose, "left_hand1") == 0) {
            info.selected_pose_id = left_pose_id;
        } else if (strcmp(selected_pose, "right_hand1") == 0) {
            info.selected_pose_id = right_pose_id;
        } else {
            pose_log_qpp_input_warning(
                "selected_pose must be left_hand1 or right_hand1");
        }

        if (info.selected_pose_id < 0) {
            pose_log_qpp_input_warning(
                "selected_pose does not match a currently detected hand");
        }
    }

    pose_data_publish_info(pose_data, &info);
}

/* Refresh the physical robot mode from QPP. */
static void refresh_physical_robot_mode_from_qpp(pose_data_t *pose_data)
{
    resource_info_t info;
    char robot_mode[MAX_ROBOT_MODE_LEN];

    if (pose_data == NULL) {
        return;
    }

    if (!read_text_snapshot_once(PHYSICAL_QPP_ROBOT_MODE_PATH,
                                 robot_mode,
                                 sizeof(robot_mode))) {
        return;
    }

    pthread_mutex_lock(&pose_data->mutex);
    info = pose_data->info;
    pthread_mutex_unlock(&pose_data->mutex);

    if (strcmp(info.robot_mode, robot_mode) == 0) {
        if (strcmp(robot_mode, "ik_joint") == 0 &&
            physical_hand_overlay_available(pose_data)) {
            refresh_physical_ik_joint_overlay_from_qpp(pose_data);
        }
        return;
    }

    strlcpy(info.robot_mode, robot_mode, sizeof(info.robot_mode));
    pose_data_publish_info(pose_data, &info);

    if (strcmp(robot_mode, "ik_joint") == 0 &&
        physical_hand_overlay_available(pose_data)) {
        refresh_physical_ik_joint_overlay_from_qpp(pose_data);
    }
}

/* Refresh the physical screensaver mode from QPP. */
static void refresh_physical_screensaver_mode_from_qpp(pose_data_t *pose_data)
{
    resource_info_t info;
    char screensaver_mode[MAX_SCREENSAVER_MODE_LEN];

    if (pose_data == NULL) {
        return;
    }

    if (!read_text_snapshot_once(PHYSICAL_QPP_SCREENSAVER_MODE_PATH,
                                 screensaver_mode,
                                 sizeof(screensaver_mode))) {
        return;
    }

    pthread_mutex_lock(&pose_data->mutex);
    info = pose_data->info;
    pthread_mutex_unlock(&pose_data->mutex);

    if (strcmp(info.screensaver_mode, screensaver_mode) == 0) {
        return;
    }

    strlcpy(info.screensaver_mode, screensaver_mode,
            sizeof(info.screensaver_mode));
    pose_data_publish_info(pose_data, &info);
}

/* Block until a physical-AI QPP value changes or shutdown is requested. */
static bool wait_for_qpp_update(pose_data_t *pose_data)
{
    struct pollfd pollfds[MAX_QPP_TEXT_SNAPSHOTS + 1];
    qpp_text_snapshot_t *snapshots[MAX_QPP_TEXT_SNAPSHOTS + 1] = { 0 };
    int fd_count = 1;
    int i;
    int poll_rc;

    memset(pollfds, 0, sizeof(pollfds));
    pollfds[0].fd = pose_data->wake_pipe[0];
    pollfds[0].events = POLLIN;

    pthread_mutex_lock(&g_qpp_text_snapshots_mutex);
    for (i = 0; i < MAX_QPP_TEXT_SNAPSHOTS; i++) {
        qpp_text_snapshot_t *snapshot = &g_qpp_text_snapshots[i];

        if (snapshot->path[0] == '\0' || snapshot->fd == -1) {
            continue;
        }
        pollfds[fd_count].fd = snapshot->fd;
        pollfds[fd_count].events = POLLIN | POLLERR | POLLHUP;
        snapshots[fd_count++] = snapshot;
    }
    pthread_mutex_unlock(&g_qpp_text_snapshots_mutex);

    poll_rc = poll(pollfds, (nfds_t)fd_count, -1);
    if (poll_rc < 0) {
        if (errno != EINTR) {
            printf("ERROR: poll() failed for physical AI QPP input: %s\n",
                   strerror(errno));
        }
        return pose_data_is_stopping(pose_data);
    }

    if ((pollfds[0].revents & POLLIN) != 0) {
        char wake_buf[16];
        while (read(pose_data->wake_pipe[0], wake_buf, sizeof(wake_buf)) > 0) {
        }
    }

    for (i = 1; i < fd_count; i++) {
        qpp_text_snapshot_t *snapshot = snapshots[i];

        pthread_mutex_lock(&g_qpp_text_snapshots_mutex);
        if ((pollfds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            close(snapshot->fd);
            snapshot->fd = -1;
        } else if ((pollfds[i].revents & POLLIN) != 0 &&
                   read_qpp_text_value(snapshot->path, snapshot->value,
                                       sizeof(snapshot->value))) {
            snapshot->has_value = true;
        }
        pthread_mutex_unlock(&g_qpp_text_snapshots_mutex);
    }

    return pose_data_is_stopping(pose_data);
}

/* Wait for QPP to notify us that one of the selected inputs changed. */
static void *pose_data_thread_main(void *arg)
{
    pose_data_t *pose_data = arg;

    printf("INFO: Starting physical AI QPP pose reader\n");
    prepare_physical_qpp_text_snapshots();

    while (!pose_data_is_stopping(pose_data)) {
        if (wait_for_qpp_update(pose_data)) {
            break;
        }
        refresh_physical_robot_mode_from_qpp(pose_data);
        refresh_physical_screensaver_mode_from_qpp(pose_data);
        refresh_physical_robot_roi_from_qpp(pose_data);
    }

    close_qpp_text_snapshots();
    return NULL;
}

/* Initialize the pose reader state without starting background work yet. */
void pose_data_init(pose_data_t *pose_data)
{
    memset(pose_data, 0, sizeof(*pose_data));
    init_default_resource_info(&pose_data->info);
    pose_data->info.width = CAMERA_REQUESTED_WIDTH;
    pose_data->info.height = CAMERA_REQUESTED_HEIGHT;
    pose_data->wake_pipe[0] = -1;
    pose_data->wake_pipe[1] = -1;
    pthread_mutex_init(&pose_data->mutex, NULL);
}

/* Start the reader thread for a file or live pose signal source. */
bool pose_data_start(pose_data_t *pose_data, GtkWidget *overlay)
{
    int create_rc;

    if (DEBUG_DISABLE_POSE_PIPELINE) {
        return true;
    }

    pose_data->overlay = overlay;
    pose_data->stopping = false;
    pose_data->redraw_pending = false;

    if (pipe(pose_data->wake_pipe) != 0) {
        printf("Failed to create wake pipe for pose data: %s\n",
               strerror(errno));
        pose_data->wake_pipe[0] = -1;
        pose_data->wake_pipe[1] = -1;
        return false;
    }

    if (fcntl(pose_data->wake_pipe[0], F_SETFL, O_NONBLOCK) == -1 ||
        fcntl(pose_data->wake_pipe[1], F_SETFL, O_NONBLOCK) == -1) {
        printf("Failed to configure wake pipe for pose data: %s\n",
               strerror(errno));
        close(pose_data->wake_pipe[0]);
        close(pose_data->wake_pipe[1]);
        pose_data->wake_pipe[0] = -1;
        pose_data->wake_pipe[1] = -1;
        return false;
    }

    create_rc = pthread_create(&pose_data->thread, NULL,
                               pose_data_thread_main, pose_data);
    if (create_rc != 0) {
        printf("Failed to create pose data thread\n");
        close(pose_data->wake_pipe[0]);
        close(pose_data->wake_pipe[1]);
        pose_data->wake_pipe[0] = -1;
        pose_data->wake_pipe[1] = -1;
        return false;
    }

    pose_data->running = true;
    pthread_setname_np(pose_data->thread, "pose_data_thread");
    return true;
}

/* Stop the reader thread and release its wake pipe and file handle. */
void pose_data_stop(pose_data_t *pose_data)
{
    pthread_mutex_lock(&pose_data->mutex);
    pose_data->stopping = true;
    pthread_mutex_unlock(&pose_data->mutex);

    if (pose_data->wake_pipe[1] != -1) {
        ssize_t rc;
        char wake = 'x';

        do {
            rc = write(pose_data->wake_pipe[1], &wake, sizeof(wake));
        } while (rc < 0 && errno == EINTR);
    }

    if (pose_data->running) {
        pthread_join(pose_data->thread, NULL);
        pose_data->running = false;
    }

    /* GLib removes only one matching source per call. */
    while (g_source_remove_by_user_data(pose_data)) {
    }

    if (pose_data->wake_pipe[0] != -1) {
        close(pose_data->wake_pipe[0]);
        pose_data->wake_pipe[0] = -1;
    }
    if (pose_data->wake_pipe[1] != -1) {
        close(pose_data->wake_pipe[1]);
        pose_data->wake_pipe[1] = -1;
    }

    pose_data->overlay = NULL;
    pthread_mutex_destroy(&pose_data->mutex);
}

/* Copy the latest pose snapshot while holding the reader mutex. */
void pose_data_get_info(pose_data_t *pose_data, resource_info_t *info_out)
{
    if (info_out == NULL) {
        return;
    }

    pthread_mutex_lock(&pose_data->mutex);
    *info_out = pose_data->info;
    pthread_mutex_unlock(&pose_data->mutex);
}
