/**
 * Copyright (c) 2025, BlackBerry Limited. All rights reserved.
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
 *
 * @file parser.h
 * @brief Defines the public interface for controller-specific HID data parsers.
 *
 * This header provides the necessary definitions and function prototypes
 * to allow the main application to dynamically select the correct parser
 * for a given joystick based on its Vendor ID (VID) and Product ID (PID).
 */
 
#ifndef PARSER_H_
#define PARSER_H_
 
// These guards allow this C header to be safely included in C++ code.
#ifdef __cplusplus
extern "C" {
#endif
 
#include <stdint.h>
#include <screen/screen.h> // For SCREEN_*_GAME_BUTTON definitions
 
// Defines the type of data to be parsed from the HID report.
#define PARSER_MODE_BUTTON   0 // Parse button data into a bitmask.
#define PARSER_MODE_ANALOG1x 1 // Parse the first joystick's X-axis.
#define PARSER_MODE_ANALOG1y 2 // Parse the first joystick's Y-axis.
#define PARSER_MODE_ANALOG2x 3 // Parse the second joystick's X-axis.
#define PARSER_MODE_ANALOG2y 4 // Parse the second joystick's Y-axis.
 
/**
 * @brief Typedef for a generic parser function pointer.
 *
 * @param mode The type of data to parse (e.g., PARSER_MODE_BUTTON).
 * @param data_len The length of the raw data buffer.
 * @param data A pointer to the raw HID report data.
 * @return The parsed integer value (a bitmask for buttons, or an axis value).
 */
typedef int (*parser_func_t)(int mode, int data_len, const uint8_t *data);
 
/**
 * @brief Retrieves the correct parser function for a given device.
 *
 * @param vid The Vendor ID of the joystick.
 * @param pid The Product ID of the joystick.
 * @return A function pointer to the correct parser, or `prs_generic` if not found.
 */
parser_func_t get_parser(int vid, int pid);

/**
 * @brief Gets the exact raw report length required by a device profile.
 *
 * Fixed-offset parsing is safe only for the tested, exact wire layout. The HID
 * node uses this value to reject shorter, longer, or composite report layouts.
 *
 * @return The exact length in bytes, or -1 for an unsupported device.
 */
int get_parser_report_length(int vid, int pid);

/**
 * @brief Validates a report against its VID/PID-specific wire profile.
 *
 * This validates the exact report size and, where the protocol defines them,
 * the report ID and size header before any fixed-offset field is consumed.
 *
 * @return 1 if the report matches the supported profile, 0 otherwise.
 */
int validate_parser_report(int vid, int pid, int data_len, const uint8_t *data);
 
/**
 * @brief Checks if a given VID/PID is in the list of supported controllers.
 *
 * @param vid The Vendor ID of the joystick.
 * @param pid The Product ID of the joystick.
 * @return 1 if allowed, -1 otherwise.
 */
int check_allowed(int vid, int pid);
 
/* --- Function Prototypes for Specific Parsers --- */
 
// A generic parser that does nothing, used as a default.
int prs_generic(int mode, int data_len, const uint8_t *data);
 
// Parser for Logitech F310/F710 in X-Input mode.
int prs_v046d_pc21d(int mode, int data_len, const uint8_t *data);
 
// Parser for Logitech F310 in D-Input mode.
int prs_v046d_pc216(int mode, int data_len, const uint8_t *data);
 
// Parser for Logitech F710 in D-Input mode.
int prs_v046d_pc219(int mode, int data_len, const uint8_t *data);
 
#ifdef __cplusplus
} // extern "C"
#endif
 
#endif /* PARSER_H_ */
