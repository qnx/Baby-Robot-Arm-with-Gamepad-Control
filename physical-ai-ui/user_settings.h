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

#ifndef USER_SETTINGS_H_
#define USER_SETTINGS_H_

#include <stdbool.h>

/* Load the last saved video flip state if a settings file exists. */
bool user_settings_load_video_flip(bool *enabled);

/* Persist whether the camera view is horizontally flipped. */
bool user_settings_save_video_flip(bool enabled);

#endif
