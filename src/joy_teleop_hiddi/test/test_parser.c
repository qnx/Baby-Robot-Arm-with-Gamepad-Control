#include "parser.h"
#include "joy_teleop_contract.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define XINPUT_REPORT_LENGTH 20
#define F310_DINPUT_REPORT_LENGTH 8
#define F710_DINPUT_REPORT_LENGTH 7

int verbose = 0;

static int failures = 0;

#define EXPECT_EQ(label, expected, actual) do { \
    const int expected_value = (expected); \
    const int actual_value = (actual); \
    if (expected_value != actual_value) { \
        fprintf(stderr, "%s: expected %d, got %d\n", (label), expected_value, actual_value); \
        ++failures; \
    } \
} while (0)

static void init_xinput_report(uint8_t report[XINPUT_REPORT_LENGTH]) {
    memset(report, 0, XINPUT_REPORT_LENGTH);
    report[0] = 0x00u;
    report[1] = XINPUT_REPORT_LENGTH;
}

static void set_le_i16(uint8_t *field, int value) {
    const uint16_t raw = (uint16_t)value;
    field[0] = (uint8_t)(raw & 0xffu);
    field[1] = (uint8_t)(raw >> 8);
}

static void test_xinput_axis_endianness(void) {
    uint8_t report[XINPUT_REPORT_LENGTH];
    init_xinput_report(report);

    set_le_i16(&report[6], 0);
    set_le_i16(&report[8], 32767);
    set_le_i16(&report[10], -32768);
    set_le_i16(&report[12], -1);

    EXPECT_EQ("left x neutral", 0,
        prs_v046d_pc21d(PARSER_MODE_ANALOG1x, XINPUT_REPORT_LENGTH, report));
    EXPECT_EQ("left y up normalized negative", -32767,
        prs_v046d_pc21d(PARSER_MODE_ANALOG1y, XINPUT_REPORT_LENGTH, report));
    EXPECT_EQ("right x minimum", -32768,
        prs_v046d_pc21d(PARSER_MODE_ANALOG2x, XINPUT_REPORT_LENGTH, report));
    EXPECT_EQ("right y inverted minus one", 1,
        prs_v046d_pc21d(PARSER_MODE_ANALOG2y, XINPUT_REPORT_LENGTH, report));
}

static int direction_sign(int value) {
    return (value > 0) - (value < 0);
}

static void test_cross_profile_y_direction_equivalence(void) {
    uint8_t f310_xinput[XINPUT_REPORT_LENGTH];
    uint8_t f710_xinput[XINPUT_REPORT_LENGTH];
    uint8_t f310_dinput[F310_DINPUT_REPORT_LENGTH] = {0};
    uint8_t f710_dinput[F710_DINPUT_REPORT_LENGTH] = {0};
    parser_func_t x_f310 = get_parser(0x046d, 0xc21d);
    parser_func_t x_f710 = get_parser(0x046d, 0xc21f);
    parser_func_t d_f310 = get_parser(0x046d, 0xc216);
    parser_func_t d_f710 = get_parser(0x046d, 0xc219);
    init_xinput_report(f310_xinput);
    init_xinput_report(f710_xinput);
    f310_dinput[4] = 8u;
    f710_dinput[4] = 8u;

    /* MS-XUSBI uses positive Y for up; these Logitech DInput profiles use
     * their low byte value for up. Parsers must expose one physical sign. */
    set_le_i16(&f310_xinput[8], 32767);
    set_le_i16(&f310_xinput[12], 32767);
    set_le_i16(&f710_xinput[8], 32767);
    set_le_i16(&f710_xinput[12], 32767);
    f310_dinput[1] = 0u;
    f310_dinput[3] = 0u;
    f710_dinput[1] = 0u;
    f710_dinput[3] = 0u;

    EXPECT_EQ("F310 XInput left-Y up", -1, direction_sign(
        x_f310(PARSER_MODE_ANALOG1y, XINPUT_REPORT_LENGTH, f310_xinput)));
    EXPECT_EQ("F710 XInput left-Y up", -1, direction_sign(
        x_f710(PARSER_MODE_ANALOG1y, XINPUT_REPORT_LENGTH, f710_xinput)));
    EXPECT_EQ("F310 DirectInput left-Y up", -1, direction_sign(
        d_f310(PARSER_MODE_ANALOG1y, F310_DINPUT_REPORT_LENGTH, f310_dinput)));
    EXPECT_EQ("F710 DirectInput left-Y up", -1, direction_sign(
        d_f710(PARSER_MODE_ANALOG1y, F710_DINPUT_REPORT_LENGTH, f710_dinput)));
    EXPECT_EQ("F310 XInput right-Y up", -1, direction_sign(
        x_f310(PARSER_MODE_ANALOG2y, XINPUT_REPORT_LENGTH, f310_xinput)));
    EXPECT_EQ("F710 XInput right-Y up", -1, direction_sign(
        x_f710(PARSER_MODE_ANALOG2y, XINPUT_REPORT_LENGTH, f710_xinput)));
    EXPECT_EQ("F310 DirectInput right-Y up", -1, direction_sign(
        d_f310(PARSER_MODE_ANALOG2y, F310_DINPUT_REPORT_LENGTH, f310_dinput)));
    EXPECT_EQ("F710 DirectInput right-Y up", -1, direction_sign(
        d_f710(PARSER_MODE_ANALOG2y, F710_DINPUT_REPORT_LENGTH, f710_dinput)));

    set_le_i16(&f310_xinput[8], -32768);
    set_le_i16(&f310_xinput[12], -32768);
    set_le_i16(&f710_xinput[8], -32768);
    set_le_i16(&f710_xinput[12], -32768);
    f310_dinput[1] = 0xffu;
    f310_dinput[3] = 0xffu;
    f710_dinput[1] = 0xffu;
    f710_dinput[3] = 0xffu;

    EXPECT_EQ("XInput -32768 Y saturates safely", 32767,
        x_f310(PARSER_MODE_ANALOG1y, XINPUT_REPORT_LENGTH, f310_xinput));
    EXPECT_EQ("F310 XInput left-Y down", 1, direction_sign(
        x_f310(PARSER_MODE_ANALOG1y, XINPUT_REPORT_LENGTH, f310_xinput)));
    EXPECT_EQ("F710 XInput left-Y down", 1, direction_sign(
        x_f710(PARSER_MODE_ANALOG1y, XINPUT_REPORT_LENGTH, f710_xinput)));
    EXPECT_EQ("F310 DirectInput left-Y down", 1, direction_sign(
        d_f310(PARSER_MODE_ANALOG1y, F310_DINPUT_REPORT_LENGTH, f310_dinput)));
    EXPECT_EQ("F710 DirectInput left-Y down", 1, direction_sign(
        d_f710(PARSER_MODE_ANALOG1y, F710_DINPUT_REPORT_LENGTH, f710_dinput)));
    EXPECT_EQ("F310 XInput right-Y down", 1, direction_sign(
        x_f310(PARSER_MODE_ANALOG2y, XINPUT_REPORT_LENGTH, f310_xinput)));
    EXPECT_EQ("F710 XInput right-Y down", 1, direction_sign(
        x_f710(PARSER_MODE_ANALOG2y, XINPUT_REPORT_LENGTH, f710_xinput)));
    EXPECT_EQ("F310 DirectInput right-Y down", 1, direction_sign(
        d_f310(PARSER_MODE_ANALOG2y, F310_DINPUT_REPORT_LENGTH, f310_dinput)));
    EXPECT_EQ("F710 DirectInput right-Y down", 1, direction_sign(
        d_f710(PARSER_MODE_ANALOG2y, F710_DINPUT_REPORT_LENGTH, f710_dinput)));
}

static void test_parser_exact_bounds_and_nulls(void) {
    uint8_t xinput[XINPUT_REPORT_LENGTH + 1];
    uint8_t dinput_f310[F310_DINPUT_REPORT_LENGTH + 1] = {0};
    uint8_t dinput_f710[F710_DINPUT_REPORT_LENGTH + 1] = {0};
    init_xinput_report(xinput);
    xinput[XINPUT_REPORT_LENGTH] = 0xa5u;
    set_le_i16(&xinput[6], 1234);
    dinput_f310[0] = 0xffu;
    dinput_f710[0] = 0xffu;

    EXPECT_EQ("xinput null", 0,
        prs_v046d_pc21d(PARSER_MODE_ANALOG1x, XINPUT_REPORT_LENGTH, NULL));
    EXPECT_EQ("xinput short", 0,
        prs_v046d_pc21d(PARSER_MODE_ANALOG1x, XINPUT_REPORT_LENGTH - 1, xinput));
    EXPECT_EQ("xinput oversized composite", 0,
        prs_v046d_pc21d(PARSER_MODE_ANALOG1x, XINPUT_REPORT_LENGTH + 1, xinput));
    EXPECT_EQ("f310 null", 0,
        prs_v046d_pc216(PARSER_MODE_ANALOG1x, F310_DINPUT_REPORT_LENGTH, NULL));
    EXPECT_EQ("f310 short", 0,
        prs_v046d_pc216(PARSER_MODE_ANALOG1x, F310_DINPUT_REPORT_LENGTH - 1, dinput_f310));
    EXPECT_EQ("f310 oversized composite", 0,
        prs_v046d_pc216(PARSER_MODE_ANALOG1x, F310_DINPUT_REPORT_LENGTH + 1, dinput_f310));
    EXPECT_EQ("f710 null", 0,
        prs_v046d_pc219(PARSER_MODE_ANALOG1x, F710_DINPUT_REPORT_LENGTH, NULL));
    EXPECT_EQ("f710 short", 0,
        prs_v046d_pc219(PARSER_MODE_ANALOG1x, F710_DINPUT_REPORT_LENGTH - 1, dinput_f710));
    EXPECT_EQ("f710 oversized composite", 0,
        prs_v046d_pc219(PARSER_MODE_ANALOG1x, F710_DINPUT_REPORT_LENGTH + 1, dinput_f710));
}

static void test_profile_lookup_and_framing(void) {
    uint8_t xinput[XINPUT_REPORT_LENGTH + 1];
    uint8_t dinput_f310[F310_DINPUT_REPORT_LENGTH + 1] = {0};
    uint8_t dinput_f710[F710_DINPUT_REPORT_LENGTH + 1] = {0};
    init_xinput_report(xinput);

    EXPECT_EQ("F310 XInput allowed", 1, check_allowed(0x046d, 0xc21d));
    EXPECT_EQ("F710 XInput allowed", 1, check_allowed(0x046d, 0xc21f));
    EXPECT_EQ("F310 DirectInput allowed", 1, check_allowed(0x046d, 0xc216));
    EXPECT_EQ("F710 DirectInput allowed", 1, check_allowed(0x046d, 0xc219));
    EXPECT_EQ("unsupported denied", -1, check_allowed(0x1234, 0x5678));
    EXPECT_EQ("F310 XInput exact length", XINPUT_REPORT_LENGTH,
        get_parser_report_length(0x046d, 0xc21d));
    EXPECT_EQ("F710 XInput exact length", XINPUT_REPORT_LENGTH,
        get_parser_report_length(0x046d, 0xc21f));
    EXPECT_EQ("F310 DirectInput exact length", F310_DINPUT_REPORT_LENGTH,
        get_parser_report_length(0x046d, 0xc216));
    EXPECT_EQ("F710 DirectInput exact length", F710_DINPUT_REPORT_LENGTH,
        get_parser_report_length(0x046d, 0xc219));
    EXPECT_EQ("unsupported length", -1, get_parser_report_length(0x1234, 0x5678));

    EXPECT_EQ("F310 XInput valid framing", 1,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, xinput));
    EXPECT_EQ("F710 XInput valid framing", 1,
        validate_parser_report(0x046d, 0xc21f, XINPUT_REPORT_LENGTH, xinput));
    EXPECT_EQ("XInput short framing rejected", 0,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH - 1, xinput));
    EXPECT_EQ("XInput oversized framing rejected", 0,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH + 1, xinput));
    xinput[0] = 0x01u;
    EXPECT_EQ("XInput non-gamepad report ID rejected", 0,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, xinput));
    xinput[0] = 0x00u;
    xinput[1] = XINPUT_REPORT_LENGTH - 1;
    EXPECT_EQ("XInput inconsistent size header rejected", 0,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, xinput));
    xinput[1] = XINPUT_REPORT_LENGTH;

    EXPECT_EQ("F310 DirectInput valid framing", 1,
        validate_parser_report(0x046d, 0xc216, F310_DINPUT_REPORT_LENGTH, dinput_f310));
    EXPECT_EQ("F310 DirectInput oversized framing rejected", 0,
        validate_parser_report(0x046d, 0xc216, F310_DINPUT_REPORT_LENGTH + 1, dinput_f310));
    dinput_f310[0] = 0xffu;
    dinput_f310[4] = 9u;
    EXPECT_EQ("F310 DirectInput invalid hat rejected", 0,
        validate_parser_report(0x046d, 0xc216, F310_DINPUT_REPORT_LENGTH, dinput_f310));
    EXPECT_EQ("F310 parser rejects invalid hat", 0,
        prs_v046d_pc216(PARSER_MODE_ANALOG1x, F310_DINPUT_REPORT_LENGTH, dinput_f310));
    EXPECT_EQ("F710 DirectInput valid framing", 1,
        validate_parser_report(0x046d, 0xc219, F710_DINPUT_REPORT_LENGTH, dinput_f710));
    EXPECT_EQ("F710 DirectInput oversized framing rejected", 0,
        validate_parser_report(0x046d, 0xc219, F710_DINPUT_REPORT_LENGTH + 1, dinput_f710));
    dinput_f710[0] = 0xffu;
    dinput_f710[4] = 0x0fu;
    EXPECT_EQ("F710 DirectInput invalid hat rejected", 0,
        validate_parser_report(0x046d, 0xc219, F710_DINPUT_REPORT_LENGTH, dinput_f710));
    EXPECT_EQ("F710 parser rejects invalid hat", 0,
        prs_v046d_pc219(PARSER_MODE_ANALOG1x, F710_DINPUT_REPORT_LENGTH, dinput_f710));
    EXPECT_EQ("unsupported profile framing rejected", 0,
        validate_parser_report(0x1234, 0x5678, XINPUT_REPORT_LENGTH, xinput));
    EXPECT_EQ("null report framing rejected", 0,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, NULL));
}

static void test_xinput_dpad_framing(void) {
    static const struct {
        uint8_t bits;
        int expected_buttons;
    } valid_diagonals[] = {
        {0x09u, SCREEN_DPAD_UP_GAME_BUTTON | SCREEN_DPAD_RIGHT_GAME_BUTTON},
        {0x05u, SCREEN_DPAD_UP_GAME_BUTTON | SCREEN_DPAD_LEFT_GAME_BUTTON},
        {0x0au, SCREEN_DPAD_DOWN_GAME_BUTTON | SCREEN_DPAD_RIGHT_GAME_BUTTON},
        {0x06u, SCREEN_DPAD_DOWN_GAME_BUTTON | SCREEN_DPAD_LEFT_GAME_BUTTON}
    };
    const int dpad_mask = SCREEN_DPAD_UP_GAME_BUTTON |
        SCREEN_DPAD_DOWN_GAME_BUTTON |
        SCREEN_DPAD_LEFT_GAME_BUTTON |
        SCREEN_DPAD_RIGHT_GAME_BUTTON;
    uint8_t report[XINPUT_REPORT_LENGTH];
    init_xinput_report(report);
    report[3] = 0x10u; /* A proves invalid frames are rejected before parsing. */

    report[2] = 0x03u;
    EXPECT_EQ("XInput up+down rejected", 0,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, report));
    EXPECT_EQ("F710 XInput up+down rejected", 0,
        validate_parser_report(0x046d, 0xc21f, XINPUT_REPORT_LENGTH, report));
    EXPECT_EQ("parser rejects up+down frame", 0,
        prs_v046d_pc21d(PARSER_MODE_BUTTON, XINPUT_REPORT_LENGTH, report));

    report[2] = 0x0cu;
    EXPECT_EQ("XInput left+right rejected", 0,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, report));
    EXPECT_EQ("F710 XInput left+right rejected", 0,
        validate_parser_report(0x046d, 0xc21f, XINPUT_REPORT_LENGTH, report));
    EXPECT_EQ("parser rejects left+right frame", 0,
        prs_v046d_pc21d(PARSER_MODE_BUTTON, XINPUT_REPORT_LENGTH, report));

    report[2] = 0x0fu;
    EXPECT_EQ("XInput all opposing directions rejected", 0,
        validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, report));

    for (size_t i = 0; i < sizeof(valid_diagonals) / sizeof(valid_diagonals[0]); ++i) {
        report[2] = valid_diagonals[i].bits;
        EXPECT_EQ("F310 XInput adjacent diagonal accepted", 1,
            validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, report));
        EXPECT_EQ("F710 XInput adjacent diagonal accepted", 1,
            validate_parser_report(0x046d, 0xc21f, XINPUT_REPORT_LENGTH, report));
        const int buttons = prs_v046d_pc21d(PARSER_MODE_BUTTON, XINPUT_REPORT_LENGTH, report);
        EXPECT_EQ("adjacent diagonal preserves both directions",
            valid_diagonals[i].expected_buttons, buttons & dpad_mask);
    }
}

static void test_xinput_ignored_padding_is_command_inert(void) {
    static const int modes[] = {
        PARSER_MODE_BUTTON,
        PARSER_MODE_ANALOG1x,
        PARSER_MODE_ANALOG1y,
        PARSER_MODE_ANALOG2x,
        PARSER_MODE_ANALOG2y
    };
    uint8_t report[XINPUT_REPORT_LENGTH];
    int baseline[sizeof(modes) / sizeof(modes[0])];
    init_xinput_report(report);
    report[2] = 0x41u;
    report[3] = 0x21u;
    report[4] = 42u;
    set_le_i16(&report[6], 12345);
    set_le_i16(&report[8], -23456);
    set_le_i16(&report[10], -1);
    set_le_i16(&report[12], 32767);

    for (size_t mode_index = 0; mode_index < sizeof(modes) / sizeof(modes[0]); ++mode_index) {
        baseline[mode_index] = prs_v046d_pc21d(
            modes[mode_index], XINPUT_REPORT_LENGTH, report);
    }

    /* The parser never consumes compatibility padding bytes 14..19. Exercise
     * every byte value so future edits cannot accidentally make them commands. */
    for (size_t offset = 14; offset < XINPUT_REPORT_LENGTH; ++offset) {
        for (int value = 0; value <= 0xff; ++value) {
            report[offset] = (uint8_t)value;
            EXPECT_EQ("F310 XInput padding keeps framing valid", 1,
                validate_parser_report(0x046d, 0xc21d, XINPUT_REPORT_LENGTH, report));
            EXPECT_EQ("F710 XInput padding keeps framing valid", 1,
                validate_parser_report(0x046d, 0xc21f, XINPUT_REPORT_LENGTH, report));
            for (size_t mode_index = 0;
                 mode_index < sizeof(modes) / sizeof(modes[0]); ++mode_index) {
                EXPECT_EQ("XInput padding cannot alter parsed commands", baseline[mode_index],
                    prs_v046d_pc21d(modes[mode_index], XINPUT_REPORT_LENGTH, report));
            }
        }
        report[offset] = 0u;
    }
}

static void test_dinput_ignored_trailing_bytes_are_command_inert(void) {
    static const int modes[] = {
        PARSER_MODE_BUTTON,
        PARSER_MODE_ANALOG1x,
        PARSER_MODE_ANALOG1y,
        PARSER_MODE_ANALOG2x,
        PARSER_MODE_ANALOG2y
    };
    uint8_t f310[F310_DINPUT_REPORT_LENGTH] = {1u, 2u, 3u, 4u, 0x18u, 0x41u, 0u, 0u};
    uint8_t f710[F710_DINPUT_REPORT_LENGTH] = {1u, 2u, 3u, 4u, 0x18u, 0x41u, 0u};
    int f310_baseline[sizeof(modes) / sizeof(modes[0])];
    int f710_baseline[sizeof(modes) / sizeof(modes[0])];

    for (size_t mode_index = 0; mode_index < sizeof(modes) / sizeof(modes[0]); ++mode_index) {
        f310_baseline[mode_index] = prs_v046d_pc216(
            modes[mode_index], F310_DINPUT_REPORT_LENGTH, f310);
        f710_baseline[mode_index] = prs_v046d_pc219(
            modes[mode_index], F710_DINPUT_REPORT_LENGTH, f710);
    }

    for (size_t offset = 6; offset < F310_DINPUT_REPORT_LENGTH; ++offset) {
        for (int value = 0; value <= 0xff; ++value) {
            f310[offset] = (uint8_t)value;
            EXPECT_EQ("F310 trailing byte keeps framing valid", 1,
                validate_parser_report(0x046d, 0xc216, F310_DINPUT_REPORT_LENGTH, f310));
            for (size_t mode_index = 0;
                 mode_index < sizeof(modes) / sizeof(modes[0]); ++mode_index) {
                EXPECT_EQ("F310 trailing byte cannot alter commands", f310_baseline[mode_index],
                    prs_v046d_pc216(modes[mode_index], F310_DINPUT_REPORT_LENGTH, f310));
            }
        }
        f310[offset] = 0u;
    }

    for (int value = 0; value <= 0xff; ++value) {
        f710[6] = (uint8_t)value;
        EXPECT_EQ("F710 trailing byte keeps framing valid", 1,
            validate_parser_report(0x046d, 0xc219, F710_DINPUT_REPORT_LENGTH, f710));
        for (size_t mode_index = 0;
             mode_index < sizeof(modes) / sizeof(modes[0]); ++mode_index) {
            EXPECT_EQ("F710 trailing byte cannot alter commands", f710_baseline[mode_index],
                prs_v046d_pc219(modes[mode_index], F710_DINPUT_REPORT_LENGTH, f710));
        }
    }
}

static void expect_stick_click_profile(
    int pid,
    int report_length,
    uint8_t *report,
    size_t button_offset) {
    parser_func_t parser = get_parser(0x046d, pid);

    report[button_offset] = 0x40u;
    EXPECT_EQ("profile valid with L3", 1,
        validate_parser_report(0x046d, pid, report_length, report));
    int buttons = parser(PARSER_MODE_BUTTON, report_length, report);
    EXPECT_EQ("bit 6 is L3", SCREEN_L3_GAME_BUTTON, buttons & SCREEN_L3_GAME_BUTTON);
    EXPECT_EQ("L3 does not assert R3", 0, buttons & SCREEN_R3_GAME_BUTTON);

    report[button_offset] = 0x80u;
    EXPECT_EQ("profile valid with R3", 1,
        validate_parser_report(0x046d, pid, report_length, report));
    buttons = parser(PARSER_MODE_BUTTON, report_length, report);
    EXPECT_EQ("bit 7 leaves L3 released", 0, buttons & SCREEN_L3_GAME_BUTTON);
    EXPECT_EQ("bit 7 is R3", SCREEN_R3_GAME_BUTTON, buttons & SCREEN_R3_GAME_BUTTON);
}

static void test_l3_positive_and_negative_for_every_profile(void) {
    uint8_t f310_xinput[XINPUT_REPORT_LENGTH];
    uint8_t f710_xinput[XINPUT_REPORT_LENGTH];
    uint8_t f310_dinput[F310_DINPUT_REPORT_LENGTH] = {0};
    uint8_t f710_dinput[F710_DINPUT_REPORT_LENGTH] = {0};
    init_xinput_report(f310_xinput);
    init_xinput_report(f710_xinput);
    f310_dinput[4] = 8u;
    f710_dinput[4] = 8u;

    expect_stick_click_profile(0xc21d, XINPUT_REPORT_LENGTH, f310_xinput, 2);
    expect_stick_click_profile(0xc21f, XINPUT_REPORT_LENGTH, f710_xinput, 2);
    expect_stick_click_profile(0xc216, F310_DINPUT_REPORT_LENGTH, f310_dinput, 5);
    expect_stick_click_profile(0xc219, F710_DINPUT_REPORT_LENGTH, f710_dinput, 5);
}

static void test_publisher_contract(void) {
    EXPECT_EQ("published axis count", 6, JOY_TELEOP_AXIS_COUNT);
    EXPECT_EQ("published button count", 12, JOY_TELEOP_BUTTON_COUNT);
    EXPECT_EQ("L3 publisher index", 10, JOY_TELEOP_L3_BUTTON_INDEX);
    EXPECT_EQ("L3 remains actuator dead-man", JOY_TELEOP_L3_BUTTON_INDEX,
        JOY_TELEOP_DEADMAN_BUTTON_INDEX);
}

static void test_button_mapping(void) {
    uint8_t report[XINPUT_REPORT_LENGTH];
    init_xinput_report(report);
    report[2] = 0x09u; /* D-pad up and right. */
    report[3] = 0x11u; /* L1 and A. */
    report[4] = 21u;   /* L2 over threshold. */

    const int buttons = prs_v046d_pc21d(PARSER_MODE_BUTTON, XINPUT_REPORT_LENGTH, report);
    EXPECT_EQ("dpad up", SCREEN_DPAD_UP_GAME_BUTTON, buttons & SCREEN_DPAD_UP_GAME_BUTTON);
    EXPECT_EQ("dpad right", SCREEN_DPAD_RIGHT_GAME_BUTTON, buttons & SCREEN_DPAD_RIGHT_GAME_BUTTON);
    EXPECT_EQ("A", SCREEN_A_GAME_BUTTON, buttons & SCREEN_A_GAME_BUTTON);
    EXPECT_EQ("L1", SCREEN_L1_GAME_BUTTON, buttons & SCREEN_L1_GAME_BUTTON);
    EXPECT_EQ("L2", SCREEN_L2_GAME_BUTTON, buttons & SCREEN_L2_GAME_BUTTON);
    EXPECT_EQ("R2 clear", 0, buttons & SCREEN_R2_GAME_BUTTON);
}

int main(void) {
    test_xinput_axis_endianness();
    test_cross_profile_y_direction_equivalence();
    test_parser_exact_bounds_and_nulls();
    test_profile_lookup_and_framing();
    test_xinput_dpad_framing();
    test_xinput_ignored_padding_is_command_inert();
    test_dinput_ignored_trailing_bytes_are_command_inert();
    test_l3_positive_and_negative_for_every_profile();
    test_publisher_contract();
    test_button_mapping();

    if (failures != 0) {
        fprintf(stderr, "%d parser test(s) failed\n", failures);
        return 1;
    }
    puts("parser tests passed");
    return 0;
}
