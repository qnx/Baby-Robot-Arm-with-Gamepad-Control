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

#include "user_settings.h"

#include <cjson/cJSON.h>

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USER_SETTINGS_FILENAME ".physical-ai-ui-settings.json"
#define USER_SETTINGS_MAX_SIZE (64U * 1024U)
#define USER_SETTINGS_VIDEO_FLIP_KEY "video_flip"

/* Build the absolute path to the per-user settings file. */
static bool get_user_settings_path(char *path, size_t path_size)
{
    const char *home;
    int written;

    home = getenv("HOME");
    if (home == NULL || home[0] == '\0') {
        return false;
    }

    written = snprintf(path, path_size, "%s/%s", home,
                       USER_SETTINGS_FILENAME);
    if (written < 0 || (size_t)written >= path_size) {
        return false;
    }

    return true;
}

/* Load and parse the settings JSON document when it exists. */
static cJSON *load_user_settings_json(char *path, size_t path_size)
{
    cJSON *root;
    FILE *file;
    long file_size;
    size_t read_size;
    char *buffer;

    if (!get_user_settings_path(path, path_size)) {
        return NULL;
    }

    file = fopen(path, "r");
    if (file == NULL) {
        if (errno != ENOENT) {
            printf("Unable to open user settings file %s: %s\n",
                   path, strerror(errno));
        }
        return NULL;
    }

    if (fseek(file, 0, SEEK_END) != 0) {
        printf("Unable to inspect user settings file %s: %s\n",
               path, strerror(errno));
        fclose(file);
        return NULL;
    }
    file_size = ftell(file);
    if (file_size < 0 || (unsigned long)file_size > USER_SETTINGS_MAX_SIZE) {
        printf("User settings file %s is invalid or too large\n", path);
        fclose(file);
        return NULL;
    }
    if (fseek(file, 0, SEEK_SET) != 0) {
        printf("Unable to rewind user settings file %s: %s\n",
               path, strerror(errno));
        fclose(file);
        return NULL;
    }

    buffer = calloc((size_t)file_size + 1, sizeof(*buffer));
    if (buffer == NULL) {
        fclose(file);
        return NULL;
    }

    read_size = fread(buffer, 1, (size_t)file_size, file);
    fclose(file);
    if (read_size != (size_t)file_size) {
        printf("Unable to read user settings file %s\n", path);
        free(buffer);
        return NULL;
    }

    root = cJSON_Parse(buffer);
    if (root == NULL) {
        printf("Unable to parse user settings file %s\n", path);
    }
    free(buffer);

    return root;
}

/* Read the saved video flip state. */
bool user_settings_load_video_flip(bool *enabled)
{
    cJSON *item;
    cJSON *root;
    char path[PATH_MAX];

    if (enabled == NULL) {
        return false;
    }

    root = load_user_settings_json(path, sizeof(path));
    if (root == NULL) {
        return false;
    }

    item = cJSON_GetObjectItemCaseSensitive(root,
                                            USER_SETTINGS_VIDEO_FLIP_KEY);
    if (!cJSON_IsBool(item)) {
        cJSON_Delete(root);
        return false;
    }

    *enabled = cJSON_IsTrue(item);
    cJSON_Delete(root);
    return true;
}

/* Write the video flip state while preserving the other user settings. */
bool user_settings_save_video_flip(bool enabled)
{
    cJSON *root;
    char *json_text;
    char path[PATH_MAX];
    FILE *file;
    bool ok = false;

    if (!get_user_settings_path(path, sizeof(path))) {
        printf("User settings path unavailable; skipping save\n");
        return false;
    }

    root = load_user_settings_json(path, sizeof(path));
    if (root == NULL || !cJSON_IsObject(root)) {
        if (root != NULL) {
            cJSON_Delete(root);
        }
        root = cJSON_CreateObject();
        if (root == NULL) {
            return false;
        }
    }

    cJSON_DeleteItemFromObject(root, USER_SETTINGS_VIDEO_FLIP_KEY);
    if (cJSON_AddBoolToObject(root, USER_SETTINGS_VIDEO_FLIP_KEY,
                             enabled) == NULL) {
        cJSON_Delete(root);
        return false;
    }

    json_text = cJSON_Print(root);
    cJSON_Delete(root);
    if (json_text == NULL) {
        return false;
    }

    file = fopen(path, "w");
    if (file == NULL) {
        printf("Unable to write user settings file %s: %s\n",
               path, strerror(errno));
        cJSON_free(json_text);
        return false;
    }

    if (fputs(json_text, file) >= 0 && fputc('\n', file) != EOF) {
        ok = true;
    } else {
        printf("Unable to write user settings file %s: %s\n",
               path, strerror(errno));
    }

    if (fclose(file) != 0) {
        printf("Unable to close user settings file %s: %s\n",
               path, strerror(errno));
        ok = false;
    }
    cJSON_free(json_text);
    return ok;
}
