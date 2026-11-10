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

#include "frame_pipeline.h"

#include "app_config.h"

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Return a camera buffer to the runtime and recycle the frame wrapper. */
static void free_frame(gpointer data)
{
    frame_t *frame = data;
    frame_pipeline_t *pipeline = frame->pipeline;
    camera_return_buffer(pipeline->camera->handle, &frame->cbuf);

    pthread_mutex_lock(&pipeline->mutex);
    frame->next = pipeline->free_frames;
    pipeline->free_frames = frame;
    pthread_mutex_unlock(&pipeline->mutex);
}

/* Textures own their pixels so GTK can retain them after a camera frame releases. */
static void release_texture(frame_pipeline_t *pipeline, frame_t *frame)
{
    g_object_unref(frame->tex);
    if (pipeline->copy_frames) {
        pthread_mutex_lock(&pipeline->mutex);
        frame->next = pipeline->free_frames;
        pipeline->free_frames = frame;
        pthread_mutex_unlock(&pipeline->mutex);
    }
}

/* Deliver a pipeline failure callback on the GTK main loop. */
static gboolean idle_pipeline_failed(gpointer user_data)
{
    frame_pipeline_t *pipeline = user_data;
    if (pipeline->failed != NULL) {
        pipeline->failed(pipeline->failed_user_data);
    }
    return G_SOURCE_REMOVE;
}

/* Present the newest frame on the GTK thread and update timing stats. */
static gboolean idle_update_frame(gpointer user_data)
{
    frame_pipeline_t *pipeline = user_data;
    frame_t *frame;
    uint64_t after_picture_set_time;
    struct timespec ts;
    uint64_t cur_time;

    pthread_mutex_lock(&pipeline->mutex);
    frame = pipeline->next_frame;
    pipeline->next_frame = NULL;
    pthread_mutex_unlock(&pipeline->mutex);

    if (frame != NULL) {
        clock_gettime(CLOCK_MONOTONIC, &ts);
        cur_time = timespec2nsec(&ts);

        pthread_mutex_lock(&pipeline->mutex);
        if (frame->ready_time_ns != 0) {
            pipeline->present_latency_total_ns +=
                cur_time - frame->ready_time_ns;
            pipeline->presented_frame_count++;
        }
        pthread_mutex_unlock(&pipeline->mutex);

        gtk_picture_set_paintable(GTK_PICTURE(pipeline->picture),
                                  GDK_PAINTABLE(frame->tex));
        clock_gettime(CLOCK_MONOTONIC, &ts);
        after_picture_set_time = timespec2nsec(&ts);

        pthread_mutex_lock(&pipeline->mutex);
        pipeline->picture_set_total_ns += after_picture_set_time - cur_time;
        pthread_mutex_unlock(&pipeline->mutex);

        release_texture(pipeline, frame);
        if (!DEBUG_DISABLE_OVERLAY_QUEUE_DRAW) {
            gtk_widget_queue_draw(pipeline->overlay);
        }
    }

    return G_SOURCE_REMOVE;
}

/* Reuse an existing frame wrapper when possible to avoid churn. */
static frame_t *acquire_frame(frame_pipeline_t *pipeline)
{
    frame_t *frame = NULL;

    pthread_mutex_lock(&pipeline->mutex);
    if (pipeline->free_frames != NULL) {
        frame = pipeline->free_frames;
        pipeline->free_frames = frame->next;
    }
    pthread_mutex_unlock(&pipeline->mutex);

    if (frame == NULL) {
        frame = calloc(1, sizeof(*frame));
    }

    return frame;
}

/* Push a frame wrapper back onto the free list. */
static void recycle_frame(frame_pipeline_t *pipeline, frame_t *frame)
{
    pthread_mutex_lock(&pipeline->mutex);
    frame->next = pipeline->free_frames;
    pipeline->free_frames = frame;
    pthread_mutex_unlock(&pipeline->mutex);
}

/* Consume camera pulses, build textures, and queue them for presentation. */
static void *frame_event_thread(void *arg)
{
    frame_pipeline_t *pipeline = arg;
    bool frame_has_buffer = false;
    int cerr;
    uint64_t cur_time;
    uint64_t camera_get_end_time;
    uint64_t camera_get_start_time;
    uint64_t csc_end_time;
    uint64_t csc_start_time;
    size_t csc_size;
    int drop_count = 0;
    frame_t *frame = NULL;
    int height;
    uint64_t last_time;
    uint64_t last_stage_log_time;
    struct _pulse pulse;
    uint8_t *pixels;
    int stride;
    uint64_t start_time;
    gsize size;
    struct timespec ts;
    int total_drop_count = 0;
    uint64_t texture_end_time;
    uint64_t texture_start_time;
    GBytes *bytes;
    frame_t *drop_frame;
    GdkMemoryFormat format;
    int width;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    start_time = timespec2nsec(&ts);
    last_time = start_time;
    last_stage_log_time = start_time;

    while (true) {
        if (MsgReceivePulse(pipeline->camera->chid, &pulse, sizeof(pulse),
                            NULL) != 0) {
            printf("MsgReceivePulse failed. Error=%s\n", strerror(errno));
            goto exit;
        }

        if (pulse.code == VIDEO_PULSE_STOP) {
            return NULL;
        }
        if (pulse.code != VIDEO_PULSE_FRAME_AVAILABLE) {
            continue;
        }

        frame = acquire_frame(pipeline);
        if (frame == NULL) {
            printf("Unable to allocate frame wrapper.\n");
            goto exit;
        }

        clock_gettime(CLOCK_MONOTONIC, &ts);
        camera_get_start_time = timespec2nsec(&ts);
        cerr = camera_get_viewfinder_buffers(pipeline->camera->handle,
                                             pipeline->camera->frame_event_key,
                                             &frame->cbuf, NULL);
        clock_gettime(CLOCK_MONOTONIC, &ts);
        camera_get_end_time = timespec2nsec(&ts);
        if (cerr != EOK) {
            printf("Unable to get viewfinder buffer. Error=%s\n",
                   strerror(cerr));
            goto exit;
        }
        frame_has_buffer = true;

        get_camera_frame_properties(&frame->cbuf, &width, &height, &stride);
        if (width <= 0 || height <= 0 || stride <= 0 || width > INT_MAX / 4 ||
            (size_t)width > SIZE_MAX / 4U / (size_t)height) {
            printf("Invalid camera frame dimensions: %dx%d stride=%d\n",
                   width, height, stride);
            goto exit;
        }

        if (pipeline->camera->csc != NULL) {
            if (frame->csc_buffer == NULL) {
                csc_size = (size_t)width * (size_t)height * 4U;
                frame->csc_buffer = malloc(csc_size);
                if (frame->csc_buffer == NULL) {
                    printf("Unable to allocate csc buffer\n");
                    goto exit;
                }
            }

            clock_gettime(CLOCK_MONOTONIC, &ts);
            csc_start_time = timespec2nsec(&ts);
            if (!pipeline->camera->csc->do_conversion(&frame->cbuf,
                                                      frame->csc_buffer,
                                                      width * 4)) {
                goto exit;
            }
            clock_gettime(CLOCK_MONOTONIC, &ts);
            csc_end_time = timespec2nsec(&ts);

            format = GDK_MEMORY_B8G8R8A8;
            pixels = frame->csc_buffer;
            stride = width * 4;
            size = (gsize)height * (gsize)stride;
        } else if (frame->cbuf.frametype == CAMERA_FRAMETYPE_RGB8888) {
            csc_start_time = 0;
            csc_end_time = 0;
            format = GDK_MEMORY_B8G8R8A8;
            pixels = frame->cbuf.framebuf;
            size = (gsize)height * (gsize)stride;
        } else if (frame->cbuf.frametype == CAMERA_FRAMETYPE_BGR8888) {
            csc_start_time = 0;
            csc_end_time = 0;
            format = GDK_MEMORY_B8G8R8A8;
            pixels = frame->cbuf.framebuf;
            size = (gsize)height * (gsize)stride;
        } else if (frame->cbuf.frametype == CAMERA_FRAMETYPE_RGB888) {
            csc_start_time = 0;
            csc_end_time = 0;
            format = GDK_MEMORY_B8G8R8;
            pixels = frame->cbuf.framebuf;
            size = (gsize)height * (gsize)stride;
        } else {
            printf("Unsupported camera frame type: %d\n",
                   frame->cbuf.frametype);
            goto exit;
        }

        bytes = pipeline->copy_frames ? g_bytes_new(pixels, size) :
            g_bytes_new_with_free_func(pixels, size, free_frame, frame);
        if (pipeline->copy_frames) {
            camera_return_buffer(pipeline->camera->handle, &frame->cbuf);
            frame_has_buffer = false;
        }
        if (bytes == NULL) {
            printf("Unable to wrap frame.\n");
            goto exit;
        }

        frame->pipeline = pipeline;
        frame_has_buffer = false;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        texture_start_time = timespec2nsec(&ts);
        frame->tex = gdk_memory_texture_new(width, height, format, bytes,
                                            stride);
        clock_gettime(CLOCK_MONOTONIC, &ts);
        texture_end_time = timespec2nsec(&ts);
        g_bytes_unref(bytes);
        if (frame->tex == NULL) {
            printf("Unable to make texture.\n");
            if (pipeline->copy_frames) {
                recycle_frame(pipeline, frame);
            }
            frame = NULL;
            goto exit;
        }
        frame->ready_time_ns = texture_end_time;

        pthread_mutex_lock(&pipeline->mutex);
        pipeline->camera_get_total_ns += camera_get_end_time -
                                         camera_get_start_time;
        if (csc_end_time > csc_start_time) {
            pipeline->csc_total_ns += csc_end_time - csc_start_time;
        }
        pipeline->texture_total_ns += texture_end_time - texture_start_time;
        pipeline->stage_sample_count++;
        pthread_mutex_unlock(&pipeline->mutex);

        pthread_mutex_lock(&pipeline->mutex);
        drop_frame = pipeline->next_frame;
        pipeline->next_frame = frame;
        pthread_mutex_unlock(&pipeline->mutex);

        if (drop_frame != NULL) {
            drop_count++;
            release_texture(pipeline, drop_frame);
        } else {
            g_idle_add_full(G_PRIORITY_HIGH_IDLE,
                            G_SOURCE_FUNC(idle_update_frame),
                            pipeline, NULL);
        }

        clock_gettime(CLOCK_MONOTONIC, &ts);
        cur_time = timespec2nsec(&ts);
        if (pipeline->drop_count_period_ns != 0 &&
            (cur_time - last_time) >= pipeline->drop_count_period_ns &&
            drop_count != 0) {
            total_drop_count += drop_count;
            printf("WARNING: Dropped %d frame(s) in last %.3fsecs "
                   "(total %d dropped frame(s) in %.3fsecs == "
                   "%.3f frames dropped/sec)\n",
                   drop_count,
                   (double)(cur_time - last_time) / (1000 * 1000 * 1000),
                   total_drop_count,
                   (double)(cur_time - start_time) / (1000 * 1000 * 1000),
                   cur_time != start_time ?
                   total_drop_count / ((double)(cur_time - start_time) /
                                       (1000 * 1000 * 1000)) : 0);
            last_time = cur_time;
            drop_count = 0;
        }

        if (pipeline->stage_stats_period_ns != 0 &&
            (cur_time - last_stage_log_time) >= pipeline->stage_stats_period_ns) {
            uint64_t camera_get_total_ns;
            uint64_t csc_total_ns;
            uint64_t texture_total_ns;
            uint64_t present_latency_total_ns;
            uint64_t picture_set_total_ns;
            uint64_t presented_frame_count;
            uint64_t stage_sample_count;
            double avg_camera_get_ms;
            double avg_csc_ms;
            double avg_picture_set_ms;
            double avg_texture_ms;
            double avg_present_latency_ms;

            pthread_mutex_lock(&pipeline->mutex);
            camera_get_total_ns = pipeline->camera_get_total_ns;
            csc_total_ns = pipeline->csc_total_ns;
            texture_total_ns = pipeline->texture_total_ns;
            present_latency_total_ns = pipeline->present_latency_total_ns;
            picture_set_total_ns = pipeline->picture_set_total_ns;
            presented_frame_count = pipeline->presented_frame_count;
            stage_sample_count = pipeline->stage_sample_count;
            pipeline->camera_get_total_ns = 0;
            pipeline->csc_total_ns = 0;
            pipeline->texture_total_ns = 0;
            pipeline->present_latency_total_ns = 0;
            pipeline->picture_set_total_ns = 0;
            pipeline->presented_frame_count = 0;
            pipeline->stage_sample_count = 0;
            pthread_mutex_unlock(&pipeline->mutex);

            avg_camera_get_ms = stage_sample_count != 0 ?
                (double)camera_get_total_ns / (double)stage_sample_count /
                (1000.0 * 1000.0) : 0.0;
            avg_csc_ms = stage_sample_count != 0 ?
                (double)csc_total_ns / (double)stage_sample_count /
                (1000.0 * 1000.0) : 0.0;
            avg_texture_ms = stage_sample_count != 0 ?
                (double)texture_total_ns / (double)stage_sample_count /
                (1000.0 * 1000.0) : 0.0;
            avg_picture_set_ms = presented_frame_count != 0 ?
                (double)picture_set_total_ns / (double)presented_frame_count /
                (1000.0 * 1000.0) : 0.0;
            avg_present_latency_ms = presented_frame_count != 0 ?
                (double)present_latency_total_ns /
                (double)presented_frame_count / (1000.0 * 1000.0) : 0.0;

            printf("INFO: Frame stages over %.3fsecs: samples=%llu "
                   "presented=%llu avg camera_get=%.3fms avg csc=%.3fms "
                   "avg texture=%.3fms avg present_latency=%.3fms "
                   "avg picture_set=%.3fms\n",
                   (double)(cur_time - last_stage_log_time) /
                   (1000.0 * 1000.0 * 1000.0),
                   (unsigned long long)stage_sample_count,
                   (unsigned long long)presented_frame_count,
                   avg_camera_get_ms,
                   avg_csc_ms,
                   avg_texture_ms,
                   avg_present_latency_ms,
                   avg_picture_set_ms);
            last_stage_log_time = cur_time;
        }

        frame = NULL;
    }

exit:
    if (pipeline->failed != NULL) {
        g_idle_add_full(G_PRIORITY_DEFAULT, idle_pipeline_failed, pipeline, NULL);
    }
    if (frame != NULL) {
        if (frame_has_buffer) {
            camera_return_buffer(pipeline->camera->handle, &frame->cbuf);
        }
        recycle_frame(pipeline, frame);
    }

    return NULL;
}

/* Reset the pipeline and bind it to the active runtime widgets. */
void frame_pipeline_init(frame_pipeline_t *pipeline,
                         camera_runtime_t *camera,
                         GtkWidget *picture,
                         GtkWidget *overlay)
{
    memset(pipeline, 0, sizeof(*pipeline));
    pipeline->camera = camera;
    pipeline->picture = picture;
    pipeline->overlay = overlay;
    pipeline->drop_count_period_ns = DEBUG_ENABLE_FRAME_PIPELINE_LOGS ?
        DROP_COUNT_PERIOD_NS : 0;
    pipeline->stage_stats_period_ns = DEBUG_ENABLE_FRAME_PIPELINE_LOGS ?
        FRAME_STAGE_STATS_PERIOD_NS : 0;
    pthread_mutex_init(&pipeline->mutex, NULL);
}

/* Launch the worker thread that listens for viewfinder pulses. */
bool frame_pipeline_start(frame_pipeline_t *pipeline)
{
    if (pthread_create(&pipeline->thread, NULL,
                       frame_event_thread, pipeline) != 0) {
        printf("Failed to create frame event thread\n");
        return false;
    }

    pipeline->thread_started = true;
    pthread_setname_np(pipeline->thread, "frame_event_thread");
    return true;
}

/* Ask the worker thread to stop and wait for it to exit. */
void frame_pipeline_stop(frame_pipeline_t *pipeline)
{
    if (pipeline->thread_started) {
        MsgSendPulse(pipeline->camera->coid, -1, VIDEO_PULSE_STOP, 0);
        pthread_join(pipeline->thread, NULL);
        pipeline->thread_started = false;
    }
    while (g_source_remove_by_user_data(pipeline)) {
    }
}

/* Free cached frames and release the pipeline mutex. */
void frame_pipeline_cleanup(frame_pipeline_t *pipeline)
{
    frame_t *frame;

    if (pipeline->next_frame != NULL) {
        release_texture(pipeline, pipeline->next_frame);
        pipeline->next_frame = NULL;
    }

    while (pipeline->free_frames != NULL) {
        frame = pipeline->free_frames;
        pipeline->free_frames = frame->next;
        free(frame->csc_buffer);
        free(frame);
    }

    pthread_mutex_destroy(&pipeline->mutex);
}
