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

#ifndef FRAME_PIPELINE_H_
#define FRAME_PIPELINE_H_

#include <gtk/gtk.h>
#include <pthread.h>

#include "camera_runtime.h"

typedef struct frame {
    camera_buffer_t cbuf;
    uint8_t *csc_buffer;
    uint64_t ready_time_ns;
    GdkTexture *tex;
    struct frame *next;
    struct frame_pipeline *pipeline;
} frame_t;

typedef struct frame_pipeline {
    camera_runtime_t *camera;
    GtkWidget *picture;
    GtkWidget *overlay;
    pthread_t thread;
    bool thread_started;
    bool copy_frames;
    void (*failed)(gpointer user_data);
    gpointer failed_user_data;
    pthread_mutex_t mutex;
    frame_t *free_frames;
    frame_t *next_frame;
    uint64_t drop_count_period_ns;
    uint64_t stage_stats_period_ns;
    uint64_t camera_get_total_ns;
    uint64_t csc_total_ns;
    uint64_t texture_total_ns;
    uint64_t present_latency_total_ns;
    uint64_t picture_set_total_ns;
    uint64_t presented_frame_count;
    uint64_t stage_sample_count;
} frame_pipeline_t;

/* Bind the pipeline to the camera runtime and presentation widgets. */
void frame_pipeline_init(frame_pipeline_t *pipeline,
                         camera_runtime_t *camera,
                         GtkWidget *picture,
                         GtkWidget *overlay);

/* Start the worker thread that converts and presents camera frames. */
bool frame_pipeline_start(frame_pipeline_t *pipeline);

/* Stop the worker thread and wake it if it is blocked on pulses. */
void frame_pipeline_stop(frame_pipeline_t *pipeline);

/* Free any cached frame wrappers and synchronization primitives. */
void frame_pipeline_cleanup(frame_pipeline_t *pipeline);

#endif
