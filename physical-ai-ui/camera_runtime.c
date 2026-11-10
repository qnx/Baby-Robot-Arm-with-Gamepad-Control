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

#include "camera_runtime.h"

#include "app_config.h"

#include <assert.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern const csc_api_t CscLibYUV;

static const char *const frame_type_names[CAMERA_FRAMETYPE_NUMFRAMETYPES] = {
    [CAMERA_FRAMETYPE_UNSPECIFIED] = "unspecified",
    [CAMERA_FRAMETYPE_NV12] = "nv12",
    [CAMERA_FRAMETYPE_RGB8888] = "rgb8888",
    [CAMERA_FRAMETYPE_RGB888] = "rgb888",
    [CAMERA_FRAMETYPE_GRAY8] = "gray8",
    [CAMERA_FRAMETYPE_METADATA] = "metadata",
    [CAMERA_FRAMETYPE_BAYER] = "bayer",
    [CAMERA_FRAMETYPE_CBYCRY] = "cbycry",
    [CAMERA_FRAMETYPE_COMPRESSEDVIDEO] = "compressed_video",
    [CAMERA_FRAMETYPE_COMPRESSEDAUDIO] = "compressed_audio",
    [CAMERA_FRAMETYPE_RGB565] = "rgb565",
    [CAMERA_FRAMETYPE_YCBCR420P] = "ycbcr420p",
    [CAMERA_FRAMETYPE_YCBYCR] = "ycbycr",
    [CAMERA_FRAMETYPE_YCRYCB] = "ycrycb",
    [CAMERA_FRAMETYPE_CRYCBY] = "crycby",
    [CAMERA_FRAMETYPE_ROI] = "roi",
    [CAMERA_FRAMETYPE_BAYER14_RGGB_PADLO16] = "bayer14_rggb_pad16",
    [CAMERA_FRAMETYPE_NV16] = "nv16",
    [CAMERA_FRAMETYPE_NV24_Y12] = "nv24_y12",
    [CAMERA_FRAMETYPE_BGR8888] = "bgr8888",
    [CAMERA_FRAMETYPE_GRAY10P] = "gray10p",
    [CAMERA_FRAMETYPE_GRAY12P] = "gray12p",
    [CAMERA_FRAMETYPE_RGB_IR_8BIT] = "rgb_ir_8bit",
    [CAMERA_FRAMETYPE_RGB_IR_10BIT] = "rgb_ir_10bit",
    [CAMERA_FRAMETYPE_NV21] = "nv21",
    [CAMERA_FRAMETYPE_NV61] = "nv61",
    [CAMERA_FRAMETYPE_YCRCB420P] = "ycrcb420p",
    [CAMERA_FRAMETYPE_CBYCRY_10BIT] = "cbycry_10bit",
    [CAMERA_FRAMETYPE_BAYER8_RGGB] = "bayer8_rggb",
    [CAMERA_FRAMETYPE_BAYER12_GRBG] = "bayer12_grbg",
    [CAMERA_FRAMETYPE_BAYER10_BGGR_PACKED] = "bayer10_bggr_packed",
    [CAMERA_FRAMETYPE_GRAY8I] = "gray8i",
    [CAMERA_FRAMETYPE_GRAY12I] = "gray12i",
    [CAMERA_FRAMETYPE_Z16] = "z16"
};

#define LOG_COLOR_IMPORTANT "\033[1;36m"
#define LOG_COLOR_ERROR "\033[1;31m"
#define LOG_COLOR_RESET "\033[0m"

/* Return a bounds-safe printable name for a frame type. */
static const char *frame_type_to_string(camera_frametype_t frame_type)
{
    size_t index = (size_t)frame_type;

    if (index >= ARRAYSZ(frame_type_names) || frame_type_names[index] == NULL) {
        return "unknown";
    }

    return frame_type_names[index];
}

/* Print an error message using the runtime's highlighted log style. */
static void log_error(const char *fmt, ...)
{
    va_list args;

    fputs(LOG_COLOR_ERROR, stdout);
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fputs(LOG_COLOR_RESET, stdout);
}

/* Return a printable name for a viewfinder mode. */
static const char *vfmode_to_string(camera_vfmode_t mode)
{
    switch (mode) {
    case CAMERA_VFMODE_VIDEO:
        return "video";
    default:
        return "unknown";
    }
}

/* Return a printable name for a camera operating mode. */
static const char *camera_mode_to_string(uint32_t mode)
{
    switch (mode) {
    case CAMERA_MODE_RO:
        return "RO";
    case CAMERA_MODE_RW:
        return "RW";
    default:
        return "custom";
    }
}

/* Return a printable name for a camera access policy. */
static const char *camera_access_policy_to_string(camera_access_policy_t policy)
{
    switch (policy) {
    case CAMERA_ACCESS_POLICY_AUTO:
        return "auto";
    case CAMERA_ACCESS_POLICY_FORCE_RO:
        return "ro";
    default:
        return "unknown";
    }
}

/* Return the ordered camera units considered during discovery. */
static camera_unit_t *get_camera_units(unsigned int *num)
{
    camera_error_t cerr;
    camera_unit_t *cunits;

    cerr = camera_get_supported_cameras(0, num, NULL);
    if (cerr != CAMERA_EOK) {
        printf("Failed to get # of supported cameras: %s\n", strerror(cerr));
        return NULL;
    }

    cunits = calloc(*num, sizeof(*cunits));
    cerr = camera_get_supported_cameras(*num, num, cunits);
    if (cerr != CAMERA_EOK) {
        printf("Failed to get supported cameras: %s\n", strerror(cerr));
        free(cunits);
        return NULL;
    }

    return cunits;
}

/* Read the frame type currently configured for the viewfinder. */
static bool query_current_viewfinder_frametype(camera_handle_t handle,
                                               camera_frametype_t *ftype_out)
{
    camera_error_t cerr;

    if (ftype_out == NULL) {
        return false;
    }

    cerr = camera_get_vf_property(handle, CAMERA_IMGPROP_FORMAT, ftype_out);
    if (cerr != CAMERA_EOK) {
        printf("Failed to read current VF format: %s\n", strerror(cerr));
        return false;
    }

    return true;
}

/* Open a camera unit using the requested access policy. */
static bool open_camera(camera_unit_t unit,
                        camera_access_policy_t access_policy,
                        camera_handle_t *camera_handle,
                        uint32_t *access_mode_out)
{
    camera_error_t cerr;
    uint32_t access_mode;

    access_mode = access_policy == CAMERA_ACCESS_POLICY_FORCE_RO ?
        CAMERA_MODE_RO : CAMERA_MODE_RW;
    cerr = camera_open(unit, access_mode, camera_handle);
    if (cerr == CAMERA_EBUSY && access_policy == CAMERA_ACCESS_POLICY_AUTO) {
        printf("Camera unit %u is busy for RW access, retrying RO\n", unit);
        access_mode = CAMERA_MODE_RO;
        cerr = camera_open(unit, access_mode, camera_handle);
    }

    if (cerr != EOK) {
        printf("Failed to open camera unit %u: %s\n", unit, strerror(cerr));
        return false;
    }

    if (!camera_has_feature(*camera_handle, CAMERA_FEATURE_VFWINDOW)) {
        printf("Camera unit %u does not have a viewfinder.\n", unit);
        camera_close(*camera_handle);
        *camera_handle = CAMERA_HANDLE_INVALID;
        return false;
    }

    if (access_mode_out != NULL) {
        *access_mode_out = access_mode;
    }

    return true;
}

/* Report whether the application can process a frame type. */
static bool frame_type_supported(camera_frametype_t ftype)
{
    if (ftype == CAMERA_FRAMETYPE_RGB8888 ||
        ftype == CAMERA_FRAMETYPE_RGB888 ||
        ftype == CAMERA_FRAMETYPE_BGR8888) {
        return true;
    }

    return CscLibYUV.is_supported(ftype);
}

/* Rank a supported frame type for format selection. */
static int frame_type_preference(camera_frametype_t ftype)
{
    switch (ftype) {
    case CAMERA_FRAMETYPE_NV12:
        return 6;
    case CAMERA_FRAMETYPE_YCBYCR:
        return 1;
    case CAMERA_FRAMETYPE_CBYCRY:
        return 2;
    case CAMERA_FRAMETYPE_BGR8888:
        return 3;
    case CAMERA_FRAMETYPE_RGB8888:
        return 4;
    case CAMERA_FRAMETYPE_RGB888:
        return 5;
    default:
        return 100;
    }
}

/* Log the frame types supported by the active camera. */
static void log_supported_viewfinder_frame_types(camera_runtime_t *runtime)
{
    camera_error_t cerr;
    camera_frametype_t *frametypes = NULL;
    uint32_t numsupported = 0;
    uint32_t i;

    cerr = camera_get_supported_vf_frame_types(runtime->handle,
                                               0,
                                               &numsupported,
                                               NULL);
    if (cerr != CAMERA_EOK || numsupported == 0) {
        printf("Unable to query supported VF frame types: %s\n",
               strerror(cerr));
        return;
    }

    frametypes = calloc(numsupported, sizeof(*frametypes));
    if (frametypes == NULL) {
        printf("Unable to allocate VF frame types array\n");
        return;
    }

    cerr = camera_get_supported_vf_frame_types(runtime->handle,
                                               numsupported,
                                               &numsupported,
                                               frametypes);
    if (cerr != CAMERA_EOK) {
        printf("Unable to read supported VF frame types: %s\n",
               strerror(cerr));
        free(frametypes);
        return;
    }

    printf("Supported VF frame types:");
    for (i = 0; i < numsupported; ++i) {
        camera_frametype_t ftype = frametypes[i];
        printf(" %s", frame_type_to_string(ftype));
    }
    fputc('\n', stdout);
    free(frametypes);
}

/* Log the capabilities reported for one frame type. */
static void log_frametype_capabilities(camera_runtime_t *runtime,
                                       camera_frametype_t ftype)
{
    bool maxmin = false;
    camera_error_t cerr;
    camera_res_t *resolutions = NULL;
    double *framerates = NULL;
    uint32_t i;
    uint32_t numsupported = 0;

    printf("VF capabilities for %s:\n", frame_type_to_string(ftype));

    cerr = camera_get_supported_vf_resolutions(runtime->handle,
                                               0,
                                               &numsupported,
                                               NULL);
    if (cerr == CAMERA_EOK && numsupported > 0) {
        resolutions = calloc(numsupported, sizeof(*resolutions));
        if (resolutions != NULL) {
            cerr = camera_get_specified_vf_resolutions(runtime->handle,
                                                       ftype,
                                                       numsupported,
                                                       &numsupported,
                                                       resolutions);
            if (cerr == CAMERA_EOK) {
                printf("  resolutions:");
                for (i = 0; i < numsupported; ++i) {
                    printf(" %dx%d",
                           (int)resolutions[i].width,
                           (int)resolutions[i].height);
                }
                fputc('\n', stdout);
            } else {
                printf("  resolutions: unavailable for %s (%s)\n",
                       frame_type_to_string(ftype), strerror(cerr));
            }
            free(resolutions);
        }
    }

    numsupported = 0;
    cerr = camera_get_supported_vf_framerates(runtime->handle,
                                              ftype,
                                              0,
                                              &numsupported,
                                              NULL,
                                              &maxmin);
    if (cerr == CAMERA_EOK && numsupported > 0) {
        framerates = calloc(numsupported, sizeof(*framerates));
        if (framerates != NULL) {
            cerr = camera_get_supported_vf_framerates(runtime->handle,
                                                      ftype,
                                                      numsupported,
                                                      &numsupported,
                                                      framerates,
                                                      &maxmin);
            if (cerr == CAMERA_EOK) {
                if (maxmin && numsupported >= 2) {
                    double max_rate = framerates[0];
                    double min_rate = framerates[1];

                    if (min_rate > max_rate) {
                        double tmp = min_rate;
                        min_rate = max_rate;
                        max_rate = tmp;
                    }

                    printf("  framerates: %.2f-%.2f fps\n",
                           min_rate, max_rate);
                } else {
                    printf("  framerates:");
                    for (i = 0; i < numsupported; ++i) {
                        printf(" %.2f", framerates[i]);
                    }
                    fputs(" fps\n", stdout);
                }
            } else {
                printf("  framerates: unavailable for %s (%s)\n",
                       frame_type_to_string(ftype), strerror(cerr));
            }
            free(framerates);
        }
    }
}

/* Log detailed capabilities for each supported frame type. */
static void log_supported_viewfinder_frametype_capabilities(camera_runtime_t *runtime)
{
    camera_error_t cerr;
    camera_frametype_t *frametypes = NULL;
    uint32_t numsupported = 0;
    uint32_t i;

    cerr = camera_get_supported_vf_frame_types(runtime->handle,
                                               0,
                                               &numsupported,
                                               NULL);
    if (cerr != CAMERA_EOK || numsupported == 0) {
        return;
    }

    frametypes = calloc(numsupported, sizeof(*frametypes));
    if (frametypes == NULL) {
        return;
    }

    cerr = camera_get_supported_vf_frame_types(runtime->handle,
                                               numsupported,
                                               &numsupported,
                                               frametypes);
    if (cerr != CAMERA_EOK) {
        free(frametypes);
        return;
    }

    for (i = 0; i < numsupported; ++i) {
        log_frametype_capabilities(runtime, frametypes[i]);
    }

    free(frametypes);
}

/* Log the viewfinder modes supported by the active camera. */
static void log_supported_viewfinder_modes(camera_runtime_t *runtime)
{
    camera_error_t cerr;
    camera_vfmode_t current_mode;
    camera_vfmode_t *modes = NULL;
    uint32_t numsupported = 0;
    uint32_t i;

    cerr = camera_get_vf_mode(runtime->handle, &current_mode);
    if (cerr == CAMERA_EOK) {
        printf("Current VF mode: %s\n", vfmode_to_string(current_mode));
    } else {
        printf("Unable to read current VF mode: %s\n", strerror(cerr));
    }

    cerr = camera_get_supported_vf_modes(runtime->handle,
                                         0,
                                         &numsupported,
                                         NULL);
    if (cerr != CAMERA_EOK || numsupported == 0) {
        printf("Unable to query supported VF modes: %s\n",
               strerror(cerr));
        return;
    }

    modes = calloc(numsupported, sizeof(*modes));
    if (modes == NULL) {
        printf("Unable to allocate VF modes array\n");
        return;
    }

    cerr = camera_get_supported_vf_modes(runtime->handle,
                                         numsupported,
                                         &numsupported,
                                         modes);
    if (cerr != CAMERA_EOK) {
        printf("Unable to read supported VF modes: %s\n",
               strerror(cerr));
        free(modes);
        return;
    }

    printf("Supported VF modes:");
    for (i = 0; i < numsupported; ++i) {
        printf(" %s", vfmode_to_string(modes[i]));
    }
    fputc('\n', stdout);
    free(modes);
}

/* Select the highest-ranked frame type supported by the camera. */
static bool select_best_supported_frametype(camera_handle_t handle,
                                            camera_frametype_t *selected_ftype)
{
    camera_error_t cerr;
    camera_frametype_t *frametypes = NULL;
    int best_preference = 1000;
    uint32_t numsupported = 0;
    uint32_t i;

    *selected_ftype = CAMERA_FRAMETYPE_UNSPECIFIED;

    cerr = camera_get_supported_vf_frame_types(handle,
                                               0,
                                               &numsupported,
                                               NULL);
    if (cerr != CAMERA_EOK || numsupported == 0) {
        return false;
    }

    frametypes = calloc(numsupported, sizeof(*frametypes));
    if (frametypes == NULL) {
        return false;
    }

    cerr = camera_get_supported_vf_frame_types(handle,
                                               numsupported,
                                               &numsupported,
                                               frametypes);
    if (cerr != CAMERA_EOK) {
        free(frametypes);
        return false;
    }

    for (i = 0; i < numsupported; ++i) {
        camera_frametype_t ftype = frametypes[i];

        if (frame_type_supported(ftype)) {
            int preference = frame_type_preference(ftype);

            if (preference < best_preference) {
                best_preference = preference;
                *selected_ftype = ftype;
            }
        }
    }

    free(frametypes);
    return *selected_ftype != CAMERA_FRAMETYPE_UNSPECIFIED;
}

/* Keep the current frame type when the application supports it. */
static bool select_current_supported_frametype(camera_handle_t handle,
                                               camera_frametype_t *selected_ftype)
{
    const char *frametype_name = "unknown";

    if (!query_current_viewfinder_frametype(handle, selected_ftype)) {
        return false;
    }

    frametype_name = frame_type_to_string(*selected_ftype);

    if (!frame_type_supported(*selected_ftype)) {
        printf("Current VF format %s is not supported by the UI pipeline\n",
               frametype_name);
        return false;
    }

    return true;
}

/* Pick the requested resolution or the closest supported fallback. */
static bool select_viewfinder_resolution(camera_runtime_t *runtime,
                                         int requested_width,
                                         int requested_height,
                                         int *selected_width,
                                         int *selected_height)
{
    camera_error_t cerr;
    camera_res_t *resolutions = NULL;
    unsigned int best_index = 0;
    uint64_t best_score = UINT64_MAX;
    unsigned int i;
    unsigned int numsupported = 0;

    *selected_width = 0;
    *selected_height = 0;

    cerr = camera_get_supported_vf_resolutions(runtime->handle,
                                               0,
                                               &numsupported,
                                               NULL);
    if (cerr != CAMERA_EOK || numsupported == 0) {
        printf("Unable to query supported VF resolutions: %s\n",
               strerror(cerr));
        return false;
    }

    resolutions = calloc(numsupported, sizeof(*resolutions));
    if (resolutions == NULL) {
        printf("Unable to allocate VF resolutions array\n");
        return false;
    }

    cerr = camera_get_supported_vf_resolutions(runtime->handle,
                                               numsupported,
                                               &numsupported,
                                               resolutions);
    if (cerr != CAMERA_EOK) {
        printf("Unable to read supported VF resolutions: %s\n",
               strerror(cerr));
        free(resolutions);
        return false;
    }

    printf("Supported VF resolutions for %s:",
           frame_type_to_string(runtime->ftype));
    for (i = 0; i < numsupported; ++i) {
        int width = (int)resolutions[i].width;
        int height = (int)resolutions[i].height;
        uint64_t score;
        int width_diff = width - requested_width;
        int height_diff = height - requested_height;

        printf(" %dx%d", width, height);

        if (width_diff < 0) {
            width_diff = -width_diff;
        }
        if (height_diff < 0) {
            height_diff = -height_diff;
        }

        score = (uint64_t)width_diff * (uint64_t)width_diff +
                (uint64_t)height_diff * (uint64_t)height_diff;
        if (width <= requested_width &&
            height <= requested_height) {
            score /= 2;
        }

        if (score < best_score) {
            best_score = score;
            best_index = i;
        }
    }
    fputc('\n', stdout);

    *selected_width = (int)resolutions[best_index].width;
    *selected_height = (int)resolutions[best_index].height;
    free(resolutions);

    return true;
}

/* Push the chosen format and resolution back into the camera stack. */
static bool apply_viewfinder_configuration(camera_runtime_t *runtime,
                                           int selected_width,
                                           int selected_height)
{
    camera_error_t cerr;

    cerr = camera_set_vf_property(runtime->handle,
                                  CAMERA_IMGPROP_FORMAT, runtime->ftype,
                                  CAMERA_IMGPROP_WIDTH, selected_width,
                                  CAMERA_IMGPROP_HEIGHT, selected_height,
                                  CAMERA_IMGPROP_CREATEWINDOW, 0);
    if (cerr == CAMERA_EOK) {
        printf("Applied VF configuration using combined format+size request\n");
        return true;
    }

    printf("Combined format+size VF request was rejected for %dx%d: %s; "
           "trying alternate configuration order\n",
           selected_width, selected_height, strerror(cerr));

    cerr = camera_set_vf_property(runtime->handle,
                                  CAMERA_IMGPROP_WIDTH, selected_width,
                                  CAMERA_IMGPROP_HEIGHT, selected_height);
    if (cerr == CAMERA_EOK) {
        cerr = camera_set_vf_property(runtime->handle,
                                      CAMERA_IMGPROP_FORMAT, runtime->ftype,
                                      CAMERA_IMGPROP_CREATEWINDOW, 0);
        if (cerr == CAMERA_EOK) {
            printf("Applied VF configuration using size-first request\n");
            return true;
        }

        printf("Size-first VF request applied size but format/createwindow "
               "was rejected: %s; trying alternate configuration order\n",
               strerror(cerr));
    } else {
        printf("Size-first VF request was rejected for %dx%d: %s; "
               "trying alternate configuration order\n",
               selected_width, selected_height, strerror(cerr));
    }

    cerr = camera_set_vf_property(runtime->handle,
                                  CAMERA_IMGPROP_FORMAT, runtime->ftype,
                                  CAMERA_IMGPROP_CREATEWINDOW, 0);
    if (cerr != CAMERA_EOK) {
        log_error("Baseline VF format/createwindow request failed: %s\n",
                  strerror(cerr));
        return false;
    }

    cerr = camera_set_vf_property(runtime->handle,
                                  CAMERA_IMGPROP_WIDTH, selected_width,
                                  CAMERA_IMGPROP_HEIGHT, selected_height);
    if (cerr == CAMERA_EOK) {
        printf("Applied VF configuration using format-first request\n");
        return true;
    }

    printf("Format-first VF size request was rejected for %dx%d: %s\n",
           selected_width, selected_height, strerror(cerr));
    return false;
}

/* Pick the nearest supported frame rate for the configured viewfinder mode. */
static bool select_viewfinder_framerate(camera_runtime_t *runtime,
                                        double requested_framerate,
                                        double *selected_framerate)
{
    bool maxmin = false;
    camera_error_t cerr;
    double *framerates = NULL;
    double best_rate = 0.0;
    double best_score = 0.0;
    uint32_t numsupported = 0;
    uint32_t i;

    *selected_framerate = 0.0;

    cerr = camera_get_supported_vf_framerates(runtime->handle,
                                              runtime->ftype,
                                              0,
                                              &numsupported,
                                              NULL,
                                              &maxmin);
    if (cerr != CAMERA_EOK || numsupported == 0) {
        printf("Unable to query supported VF framerates for %s: %s\n",
               frame_type_to_string(runtime->ftype), strerror(cerr));
        return false;
    }

    framerates = calloc(numsupported, sizeof(*framerates));
    if (framerates == NULL) {
        printf("Unable to allocate VF framerates array\n");
        return false;
    }

    cerr = camera_get_supported_vf_framerates(runtime->handle,
                                              runtime->ftype,
                                              numsupported,
                                              &numsupported,
                                              framerates,
                                              &maxmin);
    if (cerr != CAMERA_EOK) {
        printf("Unable to read supported VF framerates for %s: %s\n",
               frame_type_to_string(runtime->ftype), strerror(cerr));
        free(framerates);
        return false;
    }

    if (maxmin && numsupported >= 2) {
        double max_rate = framerates[0];
        double min_rate = framerates[1];

        if (min_rate > max_rate) {
            double tmp = min_rate;
            min_rate = max_rate;
            max_rate = tmp;
        }

        printf("Supported VF framerates for %s: range %.2f-%.2f fps\n",
               frame_type_to_string(runtime->ftype), min_rate, max_rate);

        if (requested_framerate < min_rate) {
            *selected_framerate = min_rate;
        } else if (requested_framerate > max_rate) {
            *selected_framerate = max_rate;
        } else {
            *selected_framerate = requested_framerate;
        }

        free(framerates);
        return true;
    }

    printf("Supported VF framerates for %s:",
           frame_type_to_string(runtime->ftype));
    for (i = 0; i < numsupported; ++i) {
        double framerate = framerates[i];
        double score = framerate - requested_framerate;

        printf(" %.2f", framerate);

        if (score < 0.0) {
            score = -score;
        }

        if (i == 0 || score < best_score) {
            best_score = score;
            best_rate = framerate;
        }
    }
    fputs(" fps\n", stdout);

    *selected_framerate = best_rate;
    free(framerates);
    return true;
}

/* Apply the negotiated frame rate to the active viewfinder configuration. */
static bool apply_viewfinder_framerate(camera_runtime_t *runtime,
                                       double selected_framerate)
{
    camera_error_t cerr;

    cerr = camera_set_vf_property(runtime->handle,
                                  CAMERA_IMGPROP_FRAMERATE,
                                  selected_framerate);
    if (cerr != CAMERA_EOK) {
        printf("Failed to apply VF framerate %.2f fps: %s\n",
               selected_framerate, strerror(cerr));
        return false;
    }

    printf("Applied VF framerate %.2f fps\n", selected_framerate);
    return true;
}

/* Open the first camera that can support this application's viewfinder path. */
static bool find_default_camera(camera_runtime_t *runtime)
{
    camera_unit_t *cunits = NULL;
    camera_frametype_t best_ftype = CAMERA_FRAMETYPE_UNSPECIFIED;
    uint32_t best_access_mode = 0;
    camera_unit_t best_unit = 0;
    bool found = false;
    unsigned int i;
    unsigned int num_units = 0;

    cunits = get_camera_units(&num_units);
    if (cunits == NULL) {
        return false;
    }

    for (i = 0; i < num_units; ++i) {
        uint32_t access_mode = 0;
        bool suitable = false;

        if (runtime->camera_unit_requested && cunits[i] != runtime->unit) {
            continue;
        }

        if (!open_camera(cunits[i], runtime->access_policy,
                         &runtime->handle, &access_mode)) {
            continue;
        }

        if (access_mode == CAMERA_MODE_RW) {
            suitable =
                select_best_supported_frametype(runtime->handle, &best_ftype);
        } else {
            suitable =
                select_current_supported_frametype(runtime->handle, &best_ftype);
        }

        if (suitable) {
            best_unit = cunits[i];
            best_access_mode = access_mode;
            found = true;
            camera_close(runtime->handle);
            runtime->handle = CAMERA_HANDLE_INVALID;
            break;
        }

        camera_close(runtime->handle);
        runtime->handle = CAMERA_HANDLE_INVALID;
    }

    if (!found && runtime->camera_unit_requested) {
        printf("Requested camera unit %u is unavailable or cannot support "
               "the libyuv pipeline\n", runtime->unit);
    }

    if (found) {
        runtime->unit = best_unit;
        runtime->ftype = best_ftype;
        runtime->access_mode = best_access_mode;
        if (!open_camera(runtime->unit, runtime->access_policy,
                         &runtime->handle, &runtime->access_mode)) {
            free(cunits);
            return false;
        }
        if (runtime->access_mode == CAMERA_MODE_RO &&
            !select_current_supported_frametype(runtime->handle,
                                                &runtime->ftype)) {
            camera_close(runtime->handle);
            runtime->handle = CAMERA_HANDLE_INVALID;
            free(cunits);
            return false;
        }
        free(cunits);
        return true;
    }

    printf("Unable to find a viable camera for the libyuv pipeline\n");
    free(cunits);
    return false;
}

/* Initialize camera access, CSC selection, and pulse-based frame delivery. */
bool camera_runtime_init(camera_runtime_t *runtime)
{
    camera_access_policy_t access_policy;
    camera_unit_t requested_camera_unit;
    camera_error_t cerr;
    const char *fps_status;
    const char *size_status;
    double current_framerate = 0.0;
    double selected_framerate = 0.0;
    bool requested_framerate_applied = false;
    bool requested_size_applied = false;
    int selected_height = 0;
    int selected_width = 0;
    bool camera_unit_requested;

    access_policy = runtime->access_policy;
    requested_camera_unit = runtime->unit;
    camera_unit_requested = runtime->camera_unit_requested;
    memset(runtime, 0, sizeof(*runtime));
    runtime->handle = CAMERA_HANDLE_INVALID;
    runtime->frame_event_key = -1;
    runtime->access_policy = access_policy;
    runtime->unit = requested_camera_unit;
    runtime->camera_unit_requested = camera_unit_requested;
    runtime->access_mode = 0;
    runtime->chid = -1;
    runtime->coid = -1;

    if (!find_default_camera(runtime)) {
        return false;
    }

    log_supported_viewfinder_frame_types(runtime);
    log_supported_viewfinder_frametype_capabilities(runtime);
    log_supported_viewfinder_modes(runtime);

    if (runtime->access_mode == CAMERA_MODE_RW) {
        cerr = camera_set_vf_mode(runtime->handle, CAMERA_VFMODE_VIDEO);
        if (cerr != CAMERA_EOK) {
            printf("Failed to set VF mode for unit %d: %s\n",
                   runtime->unit, strerror(cerr));
            return false;
        }

        if (select_viewfinder_resolution(runtime,
                                         CAMERA_REQUESTED_WIDTH,
                                         CAMERA_REQUESTED_HEIGHT,
                                         &selected_width,
                                         &selected_height)) {
            if (DEBUG_DISABLE_LOWER_FRAME_SIZE_SELECTION) {
                printf("Debug flag enabled; using the camera's default "
                       "viewfinder resolution\n");
            } else if (apply_viewfinder_configuration(runtime,
                                                      selected_width,
                                                      selected_height)) {
                requested_size_applied = true;
            } else {
                printf("Camera unit %d could not apply supported wxh=%dx%d; "
                       "using the camera default size\n",
                       runtime->unit,
                       selected_width,
                       selected_height);
            }
        } else {
            printf("Falling back to default VF resolution for unit %d\n",
                   runtime->unit);
        }

        if (!requested_size_applied) {
            cerr = camera_set_vf_property(runtime->handle,
                                          CAMERA_IMGPROP_FORMAT, runtime->ftype,
                                          CAMERA_IMGPROP_CREATEWINDOW, 0);
            if (cerr != CAMERA_EOK) {
                printf("Failed to set fallback vf properties for unit %d: %s\n",
                       runtime->unit, strerror(cerr));
                return false;
            }
        }
    } else {
        printf("Camera unit %d opened %s; preserving existing VF configuration\n",
               runtime->unit, camera_mode_to_string(runtime->access_mode));
    }

    cerr = camera_get_vf_property(runtime->handle,
                                  CAMERA_IMGPROP_FRAMERATE,
                                  &current_framerate);
    if (cerr == CAMERA_EOK) {
        printf("Current VF framerate for %s: %.2f fps\n",
               frame_type_to_string(runtime->ftype), current_framerate);
    } else {
        printf("Unable to read current VF framerate for %s: %s\n",
               frame_type_to_string(runtime->ftype), strerror(cerr));
    }

    if (runtime->access_mode == CAMERA_MODE_RO) {
        selected_framerate = current_framerate;
    }

    if (runtime->access_mode == CAMERA_MODE_RW) {
        if (select_viewfinder_framerate(runtime,
                                        CAMERA_REQUESTED_FRAMERATE,
                                        &selected_framerate)) {
            if (apply_viewfinder_framerate(runtime, selected_framerate)) {
                requested_framerate_applied = true;
            }
        }
    }

    cerr = camera_get_vf_property(runtime->handle,
                                  CAMERA_IMGPROP_WIDTH, &runtime->width,
                                  CAMERA_IMGPROP_HEIGHT, &runtime->height);
    if (cerr != CAMERA_EOK) {
        printf("Failed to get default frame width/height for unit %d: %s\n",
               runtime->unit, strerror(cerr));
        return false;
    }

    if (runtime->access_mode == CAMERA_MODE_RO) {
        selected_width = runtime->width;
        selected_height = runtime->height;
    }

    size_status = runtime->access_mode == CAMERA_MODE_RO ?
        "read-only current" :
        (requested_size_applied ? "applied" : "fallback");
    fps_status = runtime->access_mode == CAMERA_MODE_RO ?
        "read-only current" :
        (requested_framerate_applied ? "applied" : "fallback");

    if (runtime->ftype != CAMERA_FRAMETYPE_RGB8888 &&
        runtime->ftype != CAMERA_FRAMETYPE_RGB888 &&
        runtime->ftype != CAMERA_FRAMETYPE_BGR8888) {
        if (!CscLibYUV.is_supported(runtime->ftype)) {
            printf("libyuv does not support conversion from %s\n",
                   frame_type_to_string(runtime->ftype));
            return false;
        }

        if (!CscLibYUV.init(runtime->ftype, runtime->width, runtime->height)) {
            return false;
        }

        runtime->csc = &CscLibYUV;
    }

    printf(LOG_COLOR_IMPORTANT
           "Using camera unit %d (%s) with frametype %s, "
           "target wxh=%dx%d, active wxh=%dx%d (%s), "
           "target fps=%.2f, active fps=%.2f (%s), "
           "actual frame wxh=%dx%d"
           LOG_COLOR_RESET,
           runtime->unit, camera_mode_to_string(runtime->access_mode),
           frame_type_to_string(runtime->ftype),
           CAMERA_REQUESTED_WIDTH, CAMERA_REQUESTED_HEIGHT,
           selected_width, selected_height,
           size_status,
           CAMERA_REQUESTED_FRAMERATE,
           selected_framerate,
           fps_status,
           runtime->width, runtime->height);
    if (runtime->csc != NULL) {
        printf(" (CSC by %s to rgb8888)", runtime->csc->name);
    }
    fputc('\n', stdout);

    printf("Camera access policy=%s requested=%s actual=%s\n",
           camera_access_policy_to_string(runtime->access_policy),
           runtime->access_policy == CAMERA_ACCESS_POLICY_FORCE_RO ? "RO" : "RW",
           camera_mode_to_string(runtime->access_mode));

    runtime->chid = ChannelCreate(_NTO_CHF_PRIVATE);
    if (runtime->chid == -1) {
        printf("Unable to create channel to receive events. Error=%s\n",
               strerror(errno));
        return false;
    }

    runtime->coid = ConnectAttach(0, 0, runtime->chid, _NTO_SIDE_CHANNEL, 0);
    if (runtime->coid == -1) {
        printf("Unable to connect to event channel. Error=%s\n",
               strerror(errno));
        return false;
    }

    SIGEV_PULSE_INT_INIT(&runtime->frame_event, runtime->coid,
                         SIGEV_PULSE_PRIO_INHERIT,
                         VIDEO_PULSE_FRAME_AVAILABLE, runtime->handle);
    cerr = camera_enable_viewfinder_event(runtime->handle,
                                          CAMERA_EVENTMODE_READONLY,
                                          &runtime->frame_event_key,
                                          &runtime->frame_event);
    if (cerr != CAMERA_EOK) {
        printf("Failed to enable frame events for unit %d: %s\n",
               runtime->unit, strerror(cerr));
        return false;
    }

    return true;
}

/* Start the viewfinder stream once initialization is complete. */
bool camera_runtime_start_viewfinder(camera_runtime_t *runtime)
{
    camera_error_t cerr;

    cerr = camera_start_viewfinder(runtime->handle, NULL, NULL, NULL);
    if (cerr != CAMERA_EOK) {
        printf("Failed to start viewfinder: %s\n", strerror(cerr));
        return false;
    }

    runtime->running = true;
    return true;
}

/* Stop the viewfinder stream if the runtime currently owns one. */
void camera_runtime_stop_viewfinder(camera_runtime_t *runtime)
{
    if (runtime->running) {
        camera_stop_viewfinder(runtime->handle);
        runtime->running = false;
    }
}

/* Release camera handles and channels. */
void camera_runtime_cleanup(camera_runtime_t *runtime)
{
    if (runtime->frame_event_key != -1) {
        camera_disable_event(runtime->handle, runtime->frame_event_key);
        runtime->frame_event_key = -1;
    }
    if (runtime->csc != NULL) {
        runtime->csc->deinit();
        runtime->csc = NULL;
    }
    if (runtime->coid != -1) {
        ConnectDetach(runtime->coid);
        runtime->coid = -1;
    }
    if (runtime->chid != -1) {
        ChannelDestroy(runtime->chid);
        runtime->chid = -1;
    }
    if (runtime->handle != CAMERA_HANDLE_INVALID) {
        camera_close(runtime->handle);
        runtime->handle = CAMERA_HANDLE_INVALID;
    }
}

/* Decode frame metadata into plain width, height, and stride outputs. */
void get_camera_frame_properties(const camera_buffer_t *cbuf,
                                 int *width, int *height, int *stride)
{
    int h;
    int s;
    int w;

    assert(cbuf != NULL);

    switch (cbuf->frametype) {
    case CAMERA_FRAMETYPE_NV12:
        w = cbuf->framedesc.nv12.width;
        h = cbuf->framedesc.nv12.height;
        s = cbuf->framedesc.nv12.stride;
        break;
    case CAMERA_FRAMETYPE_NV16:
        w = cbuf->framedesc.nv16.width;
        h = cbuf->framedesc.nv16.height;
        s = cbuf->framedesc.nv16.stride;
        break;
    case CAMERA_FRAMETYPE_NV24_Y12:
        w = cbuf->framedesc.nv24.width;
        h = cbuf->framedesc.nv24.height;
        s = cbuf->framedesc.nv24.stride;
        break;
    case CAMERA_FRAMETYPE_RGB8888:
        w = cbuf->framedesc.rgb8888.width;
        h = cbuf->framedesc.rgb8888.height;
        s = cbuf->framedesc.rgb8888.stride;
        break;
    case CAMERA_FRAMETYPE_BGR8888:
        w = cbuf->framedesc.bgr8888.width;
        h = cbuf->framedesc.bgr8888.height;
        s = cbuf->framedesc.bgr8888.stride;
        break;
    case CAMERA_FRAMETYPE_RGB888:
        w = cbuf->framedesc.rgb888.width;
        h = cbuf->framedesc.rgb888.height;
        s = cbuf->framedesc.rgb888.stride;
        break;
    case CAMERA_FRAMETYPE_GRAY8:
        w = cbuf->framedesc.gray8.width;
        h = cbuf->framedesc.gray8.height;
        s = cbuf->framedesc.gray8.stride;
        break;
    case CAMERA_FRAMETYPE_CBYCRY:
        w = cbuf->framedesc.cbycry.width;
        h = cbuf->framedesc.cbycry.height;
        s = cbuf->framedesc.cbycry.stride;
        break;
    case CAMERA_FRAMETYPE_RGB565:
        w = cbuf->framedesc.rgb565.width;
        h = cbuf->framedesc.rgb565.height;
        s = cbuf->framedesc.rgb565.stride;
        break;
    case CAMERA_FRAMETYPE_YCBCR420P:
        w = cbuf->framedesc.ycbcr420p.width;
        h = cbuf->framedesc.ycbcr420p.height;
        s = cbuf->framedesc.ycbcr420p.y_stride;
        break;
    case CAMERA_FRAMETYPE_YCBYCR:
        w = cbuf->framedesc.ycbycr.width;
        h = cbuf->framedesc.ycbycr.height;
        s = cbuf->framedesc.ycbycr.stride;
        break;
    case CAMERA_FRAMETYPE_YCRYCB:
        w = cbuf->framedesc.ycrycb.width;
        h = cbuf->framedesc.ycrycb.height;
        s = cbuf->framedesc.ycrycb.stride;
        break;
    case CAMERA_FRAMETYPE_GRAY10P:
        w = cbuf->framedesc.gray10p.width;
        h = cbuf->framedesc.gray10p.height;
        s = cbuf->framedesc.gray10p.stride;
        break;
    case CAMERA_FRAMETYPE_GRAY12P:
        w = cbuf->framedesc.gray12p.width;
        h = cbuf->framedesc.gray12p.height;
        s = cbuf->framedesc.gray12p.stride;
        break;
    case CAMERA_FRAMETYPE_NV21:
        w = cbuf->framedesc.nv21.width;
        h = cbuf->framedesc.nv21.height;
        s = cbuf->framedesc.nv21.stride;
        break;
    case CAMERA_FRAMETYPE_YCRCB420P:
        w = cbuf->framedesc.ycrcb420p.width;
        h = cbuf->framedesc.ycrcb420p.height;
        s = cbuf->framedesc.ycrcb420p.y_stride;
        break;
    case CAMERA_FRAMETYPE_BAYER8_RGGB:
        w = cbuf->framedesc.bayer.width;
        h = cbuf->framedesc.bayer.height;
        s = cbuf->framedesc.bayer.stride;
        break;
    default:
        assert(false && "Unexpected frametype");
        w = 0;
        h = 0;
        s = 0;
        break;
    }

    if (width != NULL) {
        *width = w;
    }
    if (height != NULL) {
        *height = h;
    }
    if (stride != NULL) {
        *stride = s;
    }
}
