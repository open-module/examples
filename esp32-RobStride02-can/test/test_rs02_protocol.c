#include "rs02_protocol.h"

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

#define CHECK_U32(expected, actual)                                                        \
    do {                                                                                   \
        const uint32_t expected_value = (uint32_t)(expected);                              \
        const uint32_t actual_value = (uint32_t)(actual);                                  \
        if (expected_value != actual_value) {                                              \
            fprintf(                                                                       \
                stderr,                                                                    \
                "%s:%d: expected 0x%08x, got 0x%08x\n",                                 \
                __FILE__,                                                                  \
                __LINE__,                                                                  \
                (unsigned int)expected_value,                                              \
                (unsigned int)actual_value);                                               \
            s_failures++;                                                                  \
        }                                                                                  \
    } while (0)

static void check_bytes(const uint8_t expected[8], const uint8_t actual[8])
{
    if (memcmp(expected, actual, 8) != 0) {
        fprintf(stderr, "payload mismatch\n");
        s_failures++;
    }
}

static void test_zero_effort_wire_values(void)
{
    rs02_frame_t frame = {0};
    const uint8_t expected[8] = {0x7F, 0xFF, 0x7F, 0xFF, 0, 0, 0, 0};

    CHECK(rs02_make_operation_control(0x7F, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &frame));
    CHECK_U32(0x017FFF7FU, frame.identifier);
    CHECK_U32(8, frame.data_length_code);
    check_bytes(expected, frame.data);
}

static void test_operation_control_clamps_wire_values(void)
{
    rs02_frame_t frame = {0};
    const uint8_t expected[8] = {0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0};

    CHECK(rs02_make_operation_control(
        0x7F, -100.0f, 100.0f, 100.0f, 1000.0f, -1.0f, &frame));
    CHECK_U32(0x01FFFF7FU, frame.identifier);
    check_bytes(expected, frame.data);
}

static void test_enable_and_stop_frames(void)
{
    rs02_frame_t frame = {0};
    const uint8_t zeros[8] = {0};
    const uint8_t clear_fault[8] = {1, 0, 0, 0, 0, 0, 0, 0};

    CHECK(rs02_make_enable(0x7F, 0xFD, &frame));
    CHECK_U32(0x0300FD7FU, frame.identifier);
    check_bytes(zeros, frame.data);

    CHECK(rs02_make_stop(0x7F, 0xFD, false, &frame));
    CHECK_U32(0x0400FD7FU, frame.identifier);
    check_bytes(zeros, frame.data);

    CHECK(rs02_make_stop(0x7F, 0xFD, true, &frame));
    check_bytes(clear_fault, frame.data);
}

static void test_device_id_frames(void)
{
    rs02_frame_t request = {0};
    rs02_frame_t response = {
        .identifier = 0x00007FFEU,
        .data = {0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE},
        .data_length_code = 8,
    };
    rs02_device_info_t device = {0};
    const uint8_t expected_uid[8] = {0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE};

    CHECK(rs02_make_device_id_request(0x7F, 0xFD, &request));
    CHECK_U32(0x0000FD7FU, request.identifier);
    CHECK(rs02_parse_device_id(&response, 0x7F, &device));
    CHECK_U32(0x7F, device.motor_id);
    check_bytes(expected_uid, device.mcu_uid);
    CHECK(!rs02_parse_device_id(&response, 0x01, &device));
}

static void test_feedback_values_and_identity(void)
{
    rs02_frame_t frame = {
        .identifier = 0x02847FFDU,
        .data = {0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x00, 0xFA},
        .data_length_code = 8,
    };
    rs02_feedback_t feedback = {0};

    CHECK(rs02_parse_feedback(&frame, 0x7F, 0xFD, &feedback));
    CHECK_U32(0x7F, feedback.motor_id);
    CHECK_U32(0xFD, feedback.host_id);
    CHECK_U32(RS02_MODE_MOTOR, feedback.mode);
    CHECK_U32(RS02_FAULT_OVER_TEMPERATURE, feedback.faults);
    CHECK(fabsf(feedback.position_rad) < 0.001f);
    CHECK(fabsf(feedback.velocity_rad_s) < 0.001f);
    CHECK(fabsf(feedback.torque_nm) < 0.001f);
    CHECK(fabsf(feedback.temperature_c - 25.0f) < 0.001f);

    CHECK(!rs02_parse_feedback(&frame, 0x01, 0xFD, &feedback));
    CHECK(!rs02_parse_feedback(&frame, 0x7F, 0x01, &feedback));
    frame.data_length_code = 7;
    CHECK(!rs02_parse_feedback(&frame, 0x7F, 0xFD, &feedback));
}

static void test_fault_feedback_accepts_both_documented_layouts(void)
{
    rs02_frame_t frame = {
        .identifier = 0x15007FFDU,
        .data = {0x01, 0x00, 0x80, 0x01, 0x04, 0x00, 0x00, 0x80},
        .data_length_code = 8,
    };
    rs02_fault_feedback_t fault = {0};

    CHECK(rs02_parse_fault_feedback(&frame, 0x7F, 0xFD, &fault));
    CHECK_U32(0x01800001U, fault.faults);
    CHECK_U32(0x80000004U, fault.warnings);

    frame.identifier = 0x1500FD7FU;
    CHECK(rs02_parse_fault_feedback(&frame, 0x7F, 0xFD, &fault));
    CHECK(!rs02_parse_fault_feedback(&frame, 0x01, 0xFD, &fault));
    CHECK(!rs02_parse_fault_feedback(&frame, 0x7F, 0x01, &fault));
    frame.data_length_code = 7;
    CHECK(!rs02_parse_fault_feedback(&frame, 0x7F, 0xFD, &fault));
}

static void test_parameter_read_and_float_response(void)
{
    rs02_frame_t request = {0};
    rs02_frame_t response = {
        .identifier = 0x11007FFDU,
        .data = {0x1C, 0x70, 0, 0, 0, 0, 0x40, 0x42},
        .data_length_code = 8,
    };
    float value = 0.0f;

    CHECK(rs02_make_parameter_read(0x7F, 0xFD, RS02_PARAMETER_BUS_VOLTAGE, &request));
    CHECK_U32(0x1100FD7FU, request.identifier);
    const uint8_t expected[8] = {0x1C, 0x70, 0, 0, 0, 0, 0, 0};
    check_bytes(expected, request.data);

    CHECK(rs02_parse_parameter_float(
        &response, 0x7F, 0xFD, RS02_PARAMETER_BUS_VOLTAGE, &value));
    CHECK(fabsf(value - 48.0f) < 0.001f);
    CHECK(!rs02_parse_parameter_float(
        &response, 0x7F, 0xFD, RS02_PARAMETER_MECHANICAL_POSITION, &value));

    response.identifier = 0x11017FFDU;
    CHECK(!rs02_parse_parameter_float(
        &response, 0x7F, 0xFD, RS02_PARAMETER_BUS_VOLTAGE, &value));
    response.identifier = 0x11007FFDU;
    response.data[4] = 0;
    response.data[5] = 0;
    response.data[6] = 0xC0;
    response.data[7] = 0x7F;
    CHECK(!rs02_parse_parameter_float(
        &response, 0x7F, 0xFD, RS02_PARAMETER_BUS_VOLTAGE, &value));
}

static void test_parameter_writes_and_u8_response(void)
{
    rs02_frame_t frame = {0};
    const uint8_t expected_mode[8] = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
    const uint8_t expected_limit[8] = {0x0B, 0x70, 0, 0, 0, 0, 0x80, 0x3F};

    CHECK(rs02_make_parameter_write_u8(
        0x7F, 0xFD, RS02_PARAMETER_RUN_MODE, 0U, &frame));
    CHECK_U32(0x1200FD7FU, frame.identifier);
    check_bytes(expected_mode, frame.data);

    CHECK(rs02_make_parameter_write_float(
        0x7F, 0xFD, RS02_PARAMETER_TORQUE_LIMIT, 1.0f, &frame));
    CHECK_U32(0x1200FD7FU, frame.identifier);
    check_bytes(expected_limit, frame.data);
    CHECK(!rs02_make_parameter_write_float(
        0x7F, 0xFD, RS02_PARAMETER_TORQUE_LIMIT, NAN, &frame));

    rs02_frame_t response = {
        .identifier = 0x11007FFDU,
        .data = {0x05, 0x70, 0, 0, 0, 0, 0, 0},
        .data_length_code = 8,
    };
    uint8_t run_mode = 0xFFU;
    CHECK(rs02_parse_parameter_u8(
        &response, 0x7F, 0xFD, RS02_PARAMETER_RUN_MODE, &run_mode));
    CHECK_U32(0U, run_mode);
    response.data[7] = 1U;
    CHECK(!rs02_parse_parameter_u8(
        &response, 0x7F, 0xFD, RS02_PARAMETER_RUN_MODE, &run_mode));
}

static void test_null_and_nonfinite_inputs(void)
{
    rs02_frame_t frame = {0};

    CHECK(!rs02_make_enable(0x7F, 0xFD, NULL));
    CHECK(!rs02_make_operation_control(0x7F, NAN, 0.0f, 0.0f, 0.0f, 0.0f, &frame));
    CHECK(!rs02_make_operation_control(0x7F, 0.0f, INFINITY, 0.0f, 0.0f, 0.0f, &frame));
    CHECK(!rs02_make_operation_control(0x7F, 0.0f, 0.0f, -INFINITY, 0.0f, 0.0f, &frame));
}

int main(void)
{
    test_zero_effort_wire_values();
    test_operation_control_clamps_wire_values();
    test_enable_and_stop_frames();
    test_device_id_frames();
    test_feedback_values_and_identity();
    test_fault_feedback_accepts_both_documented_layouts();
    test_parameter_read_and_float_response();
    test_parameter_writes_and_u8_response();
    test_null_and_nonfinite_inputs();

    if (s_failures != 0U) {
        fprintf(stderr, "%u protocol test assertion(s) failed\n", s_failures);
        return EXIT_FAILURE;
    }

    puts("all RobStride RS02 protocol tests passed");
    return EXIT_SUCCESS;
}
