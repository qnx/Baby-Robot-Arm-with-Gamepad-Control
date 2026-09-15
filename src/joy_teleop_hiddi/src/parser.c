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
 * @file parser.c
 * @brief Implements HID data parsers for specific game controllers.
 *
 * This file contains the implementations of parser functions for
 * the Logitech controllers (F310, F710) in both X-Input and D-Input modes.
 * Each function is responsible for interpreting the controller-specific raw
 * byte array from a HID report and extracting button and analog stick data.
 */
 
#include "parser.h"
#include <stdio.h>
#include <screen/screen.h>
 
// External variable defined in joy_teleop_node.cpp, used for verbose logging if needed.
extern int verbose;
 
// Defines the total number of supported controller models in the lookup table.
#define CONTROLLER_COUNT 4
#define XINPUT_REPORT_LENGTH 20
#define F310_DINPUT_REPORT_LENGTH 8
#define F710_DINPUT_REPORT_LENGTH 7

enum report_format {
    REPORT_FORMAT_XINPUT,
    REPORT_FORMAT_DIRECTINPUT
};

struct device_parser_profile {
    int vid;
    int pid;
    int report_len;
    enum report_format format;
    parser_func_t parser;
};
 
/**
 * @brief A lookup table mapping controller Vendor IDs (VID) and Product IDs (PID)
 * to their corresponding parser functions.
 *
 * This array allows the system to dynamically select the correct parser at
 * runtime when a new controller is connected.
 */
static const struct device_parser_profile device_lookup[CONTROLLER_COUNT] = {
    /* XUSB default input report 0x00 is 20 bytes. The DirectInput HIDDI
     * payloads have no report-ID prefix and retain their profile-specific
     * trailing bytes, hence exact lengths 8 (F310) and 7 (F710). */
    {0x046d, 0xc21d, XINPUT_REPORT_LENGTH, REPORT_FORMAT_XINPUT, prs_v046d_pc21d},
    {0x046d, 0xc216, F310_DINPUT_REPORT_LENGTH, REPORT_FORMAT_DIRECTINPUT, prs_v046d_pc216},
    {0x046d, 0xc21f, XINPUT_REPORT_LENGTH, REPORT_FORMAT_XINPUT, prs_v046d_pc21d},
    {0x046d, 0xc219, F710_DINPUT_REPORT_LENGTH, REPORT_FORMAT_DIRECTINPUT, prs_v046d_pc219}
};

static const struct device_parser_profile *find_profile(int vid, int pid) {
    for (int i = 0; i < CONTROLLER_COUNT; i++) {
        if (device_lookup[i].vid == vid && device_lookup[i].pid == pid) {
            return &device_lookup[i];
        }
    }
    return NULL;
}

static int validate_xinput_report(int data_len, const uint8_t *data) {
    /* MS-XUSBI report 0x00 is exactly 20 bytes and declares size 0x14.
     * Bytes 14..19 cannot affect commands and are deliberately ignored until
     * their values are confirmed on the deployed Logitech/QNX combination. */
    if (data == NULL || data_len != XINPUT_REPORT_LENGTH ||
        data[0] != 0x00u || data[1] != XINPUT_REPORT_LENGTH) {
        return 0;
    }

    /* Adjacent directions are valid diagonals; opposing directions indicate
     * a malformed frame and must not be resolved into a motion command. */
    const uint8_t dpad = data[2] & 0x0fu;
    return (dpad & 0x03u) != 0x03u && (dpad & 0x0cu) != 0x0cu;
}

static int validate_dinput_report(int expected_length, int data_len, const uint8_t *data) {
    /* These no-ID profiles define hat values 0..7 and neutral 8. Treating an
     * out-of-range nibble as neutral could turn a malformed frame into input. */
    return data != NULL && data_len == expected_length &&
        (data[4] & 0x0fu) <= 8u;
}
 
/**
 * @brief Retrieves the correct parser function for a given device VID and PID.
 *
 * @param vid The Vendor ID of the connected controller.
 * @param pid The Product ID of the connected controller.
 * @return A function pointer to the specific parser. If no match is found,
 * it returns a pointer to a generic, non-functional parser.
 */
parser_func_t get_parser(int vid, int pid) {
    const struct device_parser_profile *profile = find_profile(vid, pid);
    return profile != NULL ? profile->parser : prs_generic;
}

int get_parser_report_length(int vid, int pid) {
    const struct device_parser_profile *profile = find_profile(vid, pid);
    return profile != NULL ? profile->report_len : -1;
}

int validate_parser_report(int vid, int pid, int data_len, const uint8_t *data) {
    const struct device_parser_profile *profile = find_profile(vid, pid);
    if (profile == NULL || data == NULL || data_len != profile->report_len) {
        return 0;
    }

    /* DirectInput has no leading report ID in these tested profiles: byte 0
     * is left-stick X. Exact length and the hat's logical range identify the
     * layouts without misclassifying axis data as a report header. */
    return profile->format == REPORT_FORMAT_XINPUT
        ? validate_xinput_report(data_len, data)
        : validate_dinput_report(profile->report_len, data_len, data);
}

/**
 * @brief Checks if a controller with the given VID and PID is supported.
 *
 * @param vid The Vendor ID of the connected controller.
 * @param pid The Product ID of the connected controller.
 * @return 1 if the controller is supported, -1 otherwise.
 */
int check_allowed(int vid, int pid) {
    return find_profile(vid, pid) != NULL ? 1 : -1;
}
 
/**
 * @brief A generic, default parser that does nothing.
 * @return Always returns 0.
 */
int prs_generic(int mode, int data_len, const uint8_t *data) {
    (void)mode;
    (void)data_len;
    (void)data;
    return 0;
}

/**
 * @brief Decodes an XInput signed 16-bit value from its little-endian bytes.
 *
 * Convert through int rather than relying on an implementation-defined
 * unsigned-to-signed narrowing conversion for values above INT16_MAX.
 */
static int _parse_le_i16(const uint8_t *data) {
    uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (raw & 0x8000u) ? (int)raw - 0x10000 : (int)raw;
}

static int _parse_inverted_le_i16(const uint8_t *data) {
    const int value = _parse_le_i16(data);
    /* XInput defines positive Y as up; DirectInput reports up at the low end.
     * Saturation avoids producing an out-of-profile +32768 for -32768. */
    return value == -32768 ? 32767 : -value;
}
 
/**
 * @brief Parser for Logitech controllers in X-Input mode (VID 0x046d, PID 0xc21d/0xc21f).
 *
 * This function decodes the 20-byte default controller input report (ID 0x00)
 * sent by the controller in X-Input mode.
 *
 * @param mode The type of data to extract (e.g., PARSER_MODE_BUTTON).
 * @param data_len The length of the raw data buffer.
 * @param data Pointer to the raw HID data buffer.
 * @return The parsed integer value (button bitmask or analog axis value).
 */
int prs_v046d_pc21d(int mode, int data_len, const uint8_t *data) {
    if (!validate_xinput_report(data_len, data)) return 0;
 
    switch (mode) {
        case PARSER_MODE_BUTTON: {
            uint32_t button = 0;
            // Bytes 2 and 3 contain the primary button states as a bitmask.
            button |= (SCREEN_DPAD_UP_GAME_BUTTON * ((data[2] & 0x01) ? 1 : 0));
            button |= (SCREEN_DPAD_DOWN_GAME_BUTTON * ((data[2] & 0x02) ? 1 : 0));
            button |= (SCREEN_DPAD_LEFT_GAME_BUTTON * ((data[2] & 0x04) ? 1 : 0));
            button |= (SCREEN_DPAD_RIGHT_GAME_BUTTON * ((data[2] & 0x08) ? 1 : 0));
            button |= (SCREEN_MENU2_GAME_BUTTON * ((data[2] & 0x10) ? 1 : 0)); // START button
            button |= (SCREEN_MENU1_GAME_BUTTON * ((data[2] & 0x20) ? 1 : 0)); // BACK button
            button |= (SCREEN_L3_GAME_BUTTON * ((data[2] & 0x40) ? 1 : 0)); // Left stick click
            button |= (SCREEN_R3_GAME_BUTTON * ((data[2] & 0x80) ? 1 : 0)); // Right stick click
            button |= (SCREEN_L1_GAME_BUTTON * ((data[3] & 0x01) ? 1 : 0)); // LB
            button |= (SCREEN_R1_GAME_BUTTON * ((data[3] & 0x02) ? 1 : 0)); // RB
            button |= (SCREEN_A_GAME_BUTTON * ((data[3] & 0x10) ? 1 : 0));
            button |= (SCREEN_B_GAME_BUTTON * ((data[3] & 0x20) ? 1 : 0));
            button |= (SCREEN_X_GAME_BUTTON * ((data[3] & 0x40) ? 1 : 0));
            button |= (SCREEN_Y_GAME_BUTTON * ((data[3] & 0x80) ? 1 : 0));
 
            // Bytes 4 and 5 represent analog triggers (LT/RT), treated as buttons here.
            button |= (data[4] > 20 ? SCREEN_L2_GAME_BUTTON : 0);
            button |= (data[5] > 20 ? SCREEN_R2_GAME_BUTTON : 0);
            return button;
        }
        // XInput axes are signed little-endian; normalize Y to the DInput physical direction.
        case PARSER_MODE_ANALOG1x: return _parse_le_i16(&data[6]);
        case PARSER_MODE_ANALOG1y: return _parse_inverted_le_i16(&data[8]);
        case PARSER_MODE_ANALOG2x: return _parse_le_i16(&data[10]);
        case PARSER_MODE_ANALOG2y: return _parse_inverted_le_i16(&data[12]);
    }
    return 0;
}
 
/**
 * @brief Internal helper function containing the shared D-Input mode parsing logic.
 *
 * This function is marked 'static' as it's only intended for use within this file.
 * It centralizes the parsing logic for both F310 and F710 D-Input modes.
 */
static int _parse_d_mode_data(int mode, const uint8_t *data) {
    switch (mode) {
        case PARSER_MODE_BUTTON: {
            uint32_t buttons = 0;
            // Byte 4 contains the face buttons and D-Pad state.
            if (data[4] & 0x10) buttons |= SCREEN_X_GAME_BUTTON; // Button 1
            if (data[4] & 0x20) buttons |= SCREEN_A_GAME_BUTTON; // Button 2
            if (data[4] & 0x40) buttons |= SCREEN_B_GAME_BUTTON; // Button 3
            if (data[4] & 0x80) buttons |= SCREEN_Y_GAME_BUTTON; // Button 4
 
            // The lower 4 bits of byte 4 represent the D-Pad as a hat switch (0-7 for directions, 8 for neutral).
            uint8_t dpad = data[4] & 0x0F;
            if (dpad == 0) buttons |= SCREEN_DPAD_UP_GAME_BUTTON;
            else if (dpad == 1) buttons |= SCREEN_DPAD_UP_GAME_BUTTON | SCREEN_DPAD_RIGHT_GAME_BUTTON;
            else if (dpad == 2) buttons |= SCREEN_DPAD_RIGHT_GAME_BUTTON;
            else if (dpad == 3) buttons |= SCREEN_DPAD_DOWN_GAME_BUTTON | SCREEN_DPAD_RIGHT_GAME_BUTTON;
            else if (dpad == 4) buttons |= SCREEN_DPAD_DOWN_GAME_BUTTON;
            else if (dpad == 5) buttons |= SCREEN_DPAD_DOWN_GAME_BUTTON | SCREEN_DPAD_LEFT_GAME_BUTTON;
            else if (dpad == 6) buttons |= SCREEN_DPAD_LEFT_GAME_BUTTON;
            else if (dpad == 7) buttons |= SCREEN_DPAD_UP_GAME_BUTTON | SCREEN_DPAD_LEFT_GAME_BUTTON;
            
            // Byte 5 contains shoulder and menu buttons.
            if (data[5] & 0x01) buttons |= SCREEN_L1_GAME_BUTTON;
            if (data[5] & 0x02) buttons |= SCREEN_R1_GAME_BUTTON;
            if (data[5] & 0x04) buttons |= SCREEN_L2_GAME_BUTTON;
            if (data[5] & 0x08) buttons |= SCREEN_R2_GAME_BUTTON;
            if (data[5] & 0x10) buttons |= SCREEN_MENU1_GAME_BUTTON; // BACK
            if (data[5] & 0x20) buttons |= SCREEN_MENU2_GAME_BUTTON; // START
            if (data[5] & 0x40) buttons |= SCREEN_L3_GAME_BUTTON; // Left stick click
            if (data[5] & 0x80) buttons |= SCREEN_R3_GAME_BUTTON; // Right stick click
            return buttons;
        }
        // Analog sticks are 8-bit values centered around 128.
        case PARSER_MODE_ANALOG1x: return data[0] - 128;
        case PARSER_MODE_ANALOG1y: return data[1] - 128;
        case PARSER_MODE_ANALOG2x: return data[2] - 128;
        case PARSER_MODE_ANALOG2y: return data[3] - 128;
    }
    return 0;
}
 
/**
 * @brief Parser for the Logitech F310 controller in D-Input mode (8-byte report).
 */
int prs_v046d_pc216(int mode, int data_len, const uint8_t *data) {
    if (!validate_dinput_report(F310_DINPUT_REPORT_LENGTH, data_len, data)) return 0;
    // Exact length prevents a composite report prefix from being parsed.
    return _parse_d_mode_data(mode, data);
}
 
/**
 * @brief Parser for the Logitech F710 controller in D-Input mode (7-byte report).
 */
int prs_v046d_pc219(int mode, int data_len, const uint8_t *data) {
    if (!validate_dinput_report(F710_DINPUT_REPORT_LENGTH, data_len, data)) return 0;
    // Exact length prevents a composite report prefix from being parsed.
    return _parse_d_mode_data(mode, data);
}
