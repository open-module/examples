#include "cybergear_protocol.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned int s_failures;

#define CHECK(condition)                                                                    \
    do {                                                                                    \
        if (!(condition)) {                                                                 \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
            s_failures++;                                                                   \
        }                                                                                   \
    } while (0)

#define CHECK_U32(expected, actual)                                                         \
    do {                                                                                    \
        const uint32_t expected_value = (uint32_t)(expected);                               \
        const uint32_t actual_value = (uint32_t)(actual);                                   \
        if (expected_value != actual_value) {                                               \
            fprintf(                                                                        \
                stderr,                                                                     \
                "%s:%d: expected 0x%08x, got 0x%08x\n",                                  \
                __FILE__,                                                                   \
                __LINE__,                                                                   \
                (unsigned int)expected_value,                                               \
                (unsigned int)actual_value);                                                \
            s_failures++;                                                                   \
        }                                                                                   \
    } while (0)

static void check_bytes(const uint8_t expected[8], const uint8_t actual[8])
{
    if (memcmp(expected, actual, 8) != 0) {
        fprintf(stderr, "payload mismatch\n");
        s_failures++;
    }
}

static void test_zero_torque_occupies_control_identifier_data(void)
{
    cybergear_frame_t frame = {0};
    const uint8_t expected[8] = {0x7F, 0xFF, 0x7F, 0xFF, 0, 0, 0, 0};

    CHECK(cybergear_make_motion_control(0x7F, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &frame));
    CHECK_U32(0x017FFF7Fu, frame.identifier);
    CHECK_U32(8, frame.data_length_code);
    check_bytes(expected, frame.data);
}

static void test_motion_control_clamps_every_wire_value(void)
{
    cybergear_frame_t frame = {0};
    const uint8_t expected[8] = {0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0};

    CHECK(cybergear_make_motion_control(0x7F, -100.0f, 100.0f, 100.0f, 1000.0f, -1.0f, &frame));
    CHECK_U32(0x01FFFF7Fu, frame.identifier);
    check_bytes(expected, frame.data);
}

static void test_enable_and_stop_use_host_identifier(void)
{
    cybergear_frame_t frame = {0};
    const uint8_t zeros[8] = {0};
    const uint8_t clear_fault[8] = {1, 0, 0, 0, 0, 0, 0, 0};

    CHECK(cybergear_make_enable(0x7F, 0xFD, &frame));
    CHECK_U32(0x0300FD7Fu, frame.identifier);
    check_bytes(zeros, frame.data);

    CHECK(cybergear_make_stop(0x7F, 0xFD, false, &frame));
    CHECK_U32(0x0400FD7Fu, frame.identifier);
    check_bytes(zeros, frame.data);

    CHECK(cybergear_make_stop(0x7F, 0xFD, true, &frame));
    check_bytes(clear_fault, frame.data);
}

static void test_device_id_frames_keep_the_manual_layout(void)
{
    cybergear_frame_t request = {0};
    cybergear_frame_t response = {
        .identifier = 0x00007FFEu,
        .data = {0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE},
        .data_length_code = 8,
    };
    cybergear_device_info_t device = {0};
    const uint8_t expected_uid[8] = {0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE};

    CHECK(cybergear_make_device_id_request(0x7F, 0xFD, &request));
    CHECK_U32(0x0000FD7Fu, request.identifier);
    CHECK(cybergear_parse_device_id(&response, &device));
    CHECK_U32(0x7F, device.motor_id);
    check_bytes(expected_uid, device.mcu_uid);
}

static void test_feedback_decodes_identifier_and_physical_values(void)
{
    const cybergear_frame_t frame = {
        .identifier = 0x02847FFDu,
        .data = {0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0xFA},
        .data_length_code = 8,
    };
    cybergear_feedback_t feedback = {0};

    CHECK(cybergear_parse_feedback(&frame, 0xFD, &feedback));
    CHECK_U32(0x7F, feedback.motor_id);
    CHECK_U32(0xFD, feedback.host_id);
    CHECK_U32(CYBERGEAR_MODE_MOTOR, feedback.mode);
    CHECK_U32(CYBERGEAR_FAULT_OVER_TEMPERATURE, feedback.faults);
    CHECK(fabsf(feedback.position_rad) < 0.001f);
    CHECK(fabsf(feedback.velocity_rad_s) < 0.001f);
    CHECK(fabsf(feedback.torque_nm) < 0.001f);
    CHECK(fabsf(feedback.temperature_c - 25.0f) < 0.001f);
}

static void test_fault_feedback_preserves_fault_and_warning_bytes(void)
{
    cybergear_frame_t frame = {
        .identifier = 0x1500FD7Fu,
        .data = {0x01, 0x00, 0x80, 0x01, 0x00, 0x00, 0x00, 0x00},
        .data_length_code = 8,
    };
    cybergear_fault_feedback_t fault = {0};
    const uint8_t expected_fault[4] = {0x01, 0x00, 0x80, 0x01};
    const uint8_t no_warning[4] = {0};

    CHECK(cybergear_parse_fault_feedback(&frame, 0xFD, &fault));
    CHECK_U32(0x7F, fault.motor_id);
    CHECK_U32(0xFD, fault.host_id);
    CHECK(fault.has_fault);
    CHECK(!fault.has_warning);
    CHECK(memcmp(expected_fault, fault.fault_data, sizeof(expected_fault)) == 0);
    CHECK(memcmp(no_warning, fault.warning_data, sizeof(no_warning)) == 0);

    memset(frame.data, 0, sizeof(frame.data));
    frame.data[7] = 1;
    CHECK(cybergear_parse_fault_feedback(&frame, 0xFD, &fault));
    CHECK(!fault.has_fault);
    CHECK(fault.has_warning);
}

static void test_fault_feedback_rejects_wrong_host_and_length(void)
{
    cybergear_frame_t frame = {
        .identifier = 0x1500FD7Fu,
        .data_length_code = 8,
    };
    cybergear_fault_feedback_t fault = {0};

    CHECK(!cybergear_parse_fault_feedback(&frame, 0x01, &fault));
    frame.data_length_code = 7;
    CHECK(!cybergear_parse_fault_feedback(&frame, 0xFD, &fault));
}

static void test_parsers_reject_wrong_type_host_and_length(void)
{
    cybergear_frame_t frame = {
        .identifier = 0x02007FFDu,
        .data_length_code = 8,
    };
    cybergear_feedback_t feedback = {0};
    cybergear_device_info_t device = {0};

    CHECK(!cybergear_parse_feedback(&frame, 0x01, &feedback));
    frame.data_length_code = 7;
    CHECK(!cybergear_parse_feedback(&frame, 0xFD, &feedback));
    frame.data_length_code = 8;
    frame.identifier = 0x01007FFEu;
    CHECK(!cybergear_parse_device_id(&frame, &device));
    CHECK(!cybergear_make_enable(0x7F, 0xFD, NULL));
}

static void test_motion_control_rejects_non_finite_inputs(void)
{
    cybergear_frame_t frame = {0};

    CHECK(!cybergear_make_motion_control(0x7F, NAN, 0.0f, 0.0f, 0.0f, 0.0f, &frame));
    CHECK(!cybergear_make_motion_control(0x7F, 0.0f, INFINITY, 0.0f, 0.0f, 0.0f, &frame));
    CHECK(!cybergear_make_motion_control(0x7F, 0.0f, 0.0f, -INFINITY, 0.0f, 0.0f, &frame));
}

int main(void)
{
    test_zero_torque_occupies_control_identifier_data();
    test_motion_control_clamps_every_wire_value();
    test_enable_and_stop_use_host_identifier();
    test_device_id_frames_keep_the_manual_layout();
    test_feedback_decodes_identifier_and_physical_values();
    test_fault_feedback_preserves_fault_and_warning_bytes();
    test_fault_feedback_rejects_wrong_host_and_length();
    test_parsers_reject_wrong_type_host_and_length();
    test_motion_control_rejects_non_finite_inputs();

    if (s_failures != 0) {
        fprintf(stderr, "%u protocol test assertion(s) failed\n", s_failures);
        return EXIT_FAILURE;
    }

    puts("all CyberGear protocol tests passed");
    return EXIT_SUCCESS;
}
