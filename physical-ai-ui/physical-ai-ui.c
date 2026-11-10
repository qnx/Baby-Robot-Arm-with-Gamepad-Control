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

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <gtk/gtk.h>

#include "app_config.h"
#include "app_ui.h"
#include "camera_runtime.h"
#include "frame_pipeline.h"

typedef struct {
    camera_runtime_t camera;
    frame_pipeline_t frame_pipeline;
    app_ui_t ui;
    gint64 app_start_us;
    bool show_video_flip;
    bool pipeline_initialized;
} app_context_t;

/* Parse a camera access policy command-line value. */
static bool parse_camera_access_policy_arg(const char *value,
                                           camera_access_policy_t *policy_out)
{
    if (value == NULL || policy_out == NULL) {
        return false;
    }

    if (strcmp(value, "auto") == 0) {
        *policy_out = CAMERA_ACCESS_POLICY_AUTO;
        return true;
    }

    if (strcmp(value, "ro") == 0) {
        *policy_out = CAMERA_ACCESS_POLICY_FORCE_RO;
        return true;
    }

    return false;
}

/* Parse a camera unit command-line value. */
static bool parse_camera_unit_arg(const char *value, camera_unit_t *unit_out)
{
    char *end = NULL;
    unsigned long parsed;

    if (value == NULL || *value < '0' || *value > '9' || unit_out == NULL) {
        return false;
    }

    errno = 0;
    parsed = strtoul(value, &end, 10);
    if (errno == ERANGE || end == value || *end != '\0' ||
        parsed > (unsigned long)INT_MAX) {
        return false;
    }

    *unit_out = (camera_unit_t)parsed;
    return true;
}

/* Print command-line usage information. */
static void print_usage(const char *program_name)
{
    fprintf(stdout,
            "Usage: %s [OPTIONS]\n"
            "\n"
            "Options:\n"
            "  -h, --help                 Show this help message and exit\n"
            "  --show-video-flip          Show the Flip Video control\n"
            "  --camera-access=auto|ro    Camera access policy (default: auto)\n"
            "  --camera-unit=<n>          Use camera unit n (default: first viable unit)\n",
            program_name != NULL ? program_name : "physical-ai-ui");
}

/* Report whether the command line requests usage help. */
static bool has_help_arg(int argc, char **argv)
{
    int index;

    for (index = 1; index < argc; ++index) {
        if (strcmp(argv[index], "-h") == 0 ||
            strcmp(argv[index], "--help") == 0) {
            return true;
        }
    }

    return false;
}

/* Strip the app-specific flags before GTK processes argv. */
static bool parse_app_args(int *argc, char ***argv,
                           camera_access_policy_t *camera_access_policy_out,
                           camera_unit_t *camera_unit_out,
                           bool *camera_unit_requested_out,
                           bool *show_video_flip_out)
{
    int read_index;
    int write_index;
    char **args;

    if (argc == NULL || argv == NULL || *argv == NULL ||
        camera_access_policy_out == NULL || camera_unit_out == NULL ||
        camera_unit_requested_out == NULL || show_video_flip_out == NULL) {
        return false;
    }

    args = *argv;
    *camera_access_policy_out = CAMERA_ACCESS_POLICY_AUTO;
    *camera_unit_requested_out = false;
    *show_video_flip_out = false;
    write_index = 1;

    for (read_index = 1; read_index < *argc; ++read_index) {
        camera_access_policy_t parsed_policy;

        if (strcmp(args[read_index], "--show-video-flip") == 0) {
            *show_video_flip_out = true;
            continue;
        }

        if (g_str_has_prefix(args[read_index], "--camera-access=")) {
            if (!parse_camera_access_policy_arg(
                    args[read_index] + strlen("--camera-access="),
                    &parsed_policy)) {
                fprintf(stderr,
                        "Unknown camera access '%s'. Expected "
                        "--camera-access=auto or --camera-access=ro.\n",
                        args[read_index] + strlen("--camera-access="));
                return false;
            }
            *camera_access_policy_out = parsed_policy;
            continue;
        }

        if (g_str_has_prefix(args[read_index], "--camera-unit=")) {
            if (!parse_camera_unit_arg(
                    args[read_index] + strlen("--camera-unit="),
                    camera_unit_out)) {
                fprintf(stderr,
                        "Invalid camera unit '%s'. Expected a non-negative integer.\n",
                        args[read_index] + strlen("--camera-unit="));
                return false;
            }
            *camera_unit_requested_out = true;
            continue;
        }

        args[write_index++] = args[read_index];
    }

    args[write_index] = NULL;
    *argc = write_index;
    return true;
}

/* Display a camera failure and stop active processing. */
static void show_camera_failure(app_context_t *ctx, const char *reason)
{
    char message[192];
    if (ctx->camera.camera_unit_requested) {
        snprintf(message, sizeof(message), "Camera unit %u unavailable\n%s",
                 (unsigned)ctx->camera.unit, reason);
    } else {
        snprintf(message, sizeof(message), "Camera unavailable\n%s", reason);
    }
    gtk_picture_set_paintable(GTK_PICTURE(ctx->ui.picture), NULL);
    gtk_label_set_text(GTK_LABEL(ctx->ui.camera_error), message);
    gtk_widget_set_visible(ctx->ui.camera_error, TRUE);
    gtk_widget_set_visible(ctx->ui.overlay, FALSE);
}

/* Present asynchronous frame-pipeline failures to the user. */
static void on_pipeline_failed(gpointer user_data)
{
    app_context_t *ctx = user_data;
    frame_pipeline_stop(&ctx->frame_pipeline);
    frame_pipeline_cleanup(&ctx->frame_pipeline);
    ctx->pipeline_initialized = false;
    camera_runtime_stop_viewfinder(&ctx->camera);
    camera_runtime_cleanup(&ctx->camera);
    show_camera_failure(ctx, "Unable to receive camera frames");
}

/* Start the UI and frame pipeline once GTK activates the app. */
static void gtk_activate(GtkApplication *app, gpointer user_data)
{
    app_context_t *ctx = user_data;

    if (!app_ui_init(&ctx->ui, app,
                     ctx->show_video_flip,
                     &ctx->camera.fullscreen,
                     ctx->app_start_us)) {
        g_application_quit(G_APPLICATION(app));
        return;
    }

    ctx->pipeline_initialized = true;
    frame_pipeline_init(&ctx->frame_pipeline, &ctx->camera,
                        app_ui_get_picture(&ctx->ui),
                        app_ui_get_overlay(&ctx->ui));
    ctx->frame_pipeline.failed = on_pipeline_failed;
    ctx->frame_pipeline.failed_user_data = ctx;

    if (!camera_runtime_start_viewfinder(&ctx->camera)) {
        g_application_quit(G_APPLICATION(app));
        return;
    }

    if (!frame_pipeline_start(&ctx->frame_pipeline)) {
        camera_runtime_stop_viewfinder(&ctx->camera);
        g_application_quit(G_APPLICATION(app));
        return;
    }

    app_ui_present(&ctx->ui);
}

/* Shut down runtime services in the reverse order they were started. */
static void gtk_shutdown(GtkApplication *app, gpointer user_data)
{
    app_context_t *ctx = user_data;

    (void)app;

    if (ctx->pipeline_initialized) {
        frame_pipeline_stop(&ctx->frame_pipeline);
        frame_pipeline_cleanup(&ctx->frame_pipeline);
    }
    camera_runtime_stop_viewfinder(&ctx->camera);
    app_ui_shutdown(&ctx->ui);
}

/* Initialize the runtime and hand control to the GTK application loop. */
int main(int argc, char **argv)
{
    GtkApplication *app;
    app_context_t ctx;
    int status;

    memset(&ctx, 0, sizeof(ctx));
    ctx.app_start_us = g_get_monotonic_time();
    ctx.camera.access_policy = CAMERA_ACCESS_POLICY_AUTO;

    if (has_help_arg(argc, argv)) {
        print_usage(argv[0]);
        return 0;
    }

    if (!parse_app_args(&argc, &argv, &ctx.camera.access_policy,
                        &ctx.camera.unit, &ctx.camera.camera_unit_requested,
                        &ctx.show_video_flip)) {
        return 1;
    }

    printf("Version %s\n", APP_VERSION);

    if (!camera_runtime_init(&ctx.camera)) {
        camera_runtime_cleanup(&ctx.camera);
        return 1;
    }

    app = gtk_application_new("com.qnx.physical-ai-ui",
                              G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(app, "activate", G_CALLBACK(gtk_activate), &ctx);
    g_signal_connect(app, "shutdown", G_CALLBACK(gtk_shutdown), &ctx);
    status = g_application_run(G_APPLICATION(app), argc, argv);
    g_object_unref(app);

    camera_runtime_cleanup(&ctx.camera);
    return status == 0 ? 0 : 1;
}
