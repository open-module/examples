#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS02_CAN_PAYLOAD_SIZE 8U
#define RS02_CAN_ID_MAX 0x1FFFFFFFU

// RS02 private-protocol wire ranges. Position follows the official STM32
// SampleProgram implementation; the remaining ranges match the 2026-07-13 manual.
#define RS02_POSITION_MIN_RAD (-12.5f)
#define RS02_POSITION_MAX_RAD 12.5f
#define RS02_VELOCITY_MIN_RAD_S (-44.0f)
#define RS02_VELOCITY_MAX_RAD_S 44.0f
#define RS02_TORQUE_MIN_NM (-17.0f)
#define RS02_TORQUE_MAX_NM 17.0f
#define RS02_KP_MIN 0.0f
#define RS02_KP_MAX 500.0f
#define RS02_KD_MIN 0.0f
#define RS02_KD_MAX 5.0f

typedef enum {
    RS02_COMM_DEVICE_ID = 0,
    RS02_COMM_OPERATION_CONTROL = 1,
    RS02_COMM_FEEDBACK = 2,
    RS02_COMM_ENABLE = 3,
    RS02_COMM_STOP = 4,
    RS02_COMM_PARAMETER_READ = 17,
    RS02_COMM_PARAMETER_WRITE = 18,
    RS02_COMM_FAULT_FEEDBACK = 21,
} rs02_communication_type_t;

typedef enum {
    RS02_MODE_RESET = 0,
    RS02_MODE_CALIBRATION = 1,
    RS02_MODE_MOTOR = 2,
} rs02_mode_t;

typedef enum {
    RS02_FAULT_UNDER_VOLTAGE = 1U << 0,
    RS02_FAULT_THREE_PHASE_CURRENT = 1U << 1,
    RS02_FAULT_OVER_TEMPERATURE = 1U << 2,
    RS02_FAULT_MAGNETIC_ENCODER = 1U << 3,
    RS02_FAULT_STALL_OVERLOAD = 1U << 4,
    RS02_FAULT_UNCALIBRATED = 1U << 5,
} rs02_feedback_fault_t;

typedef enum {
    RS02_PARAMETER_RUN_MODE = 0x7005,
    RS02_PARAMETER_TORQUE_LIMIT = 0x700B,
    RS02_PARAMETER_MECHANICAL_POSITION = 0x7019,
    RS02_PARAMETER_BUS_VOLTAGE = 0x701C,
} rs02_parameter_index_t;

typedef struct {
    uint32_t identifier;
    uint8_t data[RS02_CAN_PAYLOAD_SIZE];
    uint8_t data_length_code;
} rs02_frame_t;

typedef struct {
    uint8_t motor_id;
    uint8_t mcu_uid[RS02_CAN_PAYLOAD_SIZE];
} rs02_device_info_t;

typedef struct {
    uint8_t motor_id;
    uint8_t host_id;
    rs02_mode_t mode;
    uint8_t faults;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} rs02_feedback_t;

typedef struct {
    uint8_t motor_id;
    uint8_t host_id;
    uint32_t faults;
    uint32_t warnings;
} rs02_fault_feedback_t;

bool rs02_make_device_id_request(uint8_t motor_id, uint8_t host_id, rs02_frame_t *frame);
bool rs02_make_enable(uint8_t motor_id, uint8_t host_id, rs02_frame_t *frame);
bool rs02_make_stop(uint8_t motor_id, uint8_t host_id, bool clear_fault, rs02_frame_t *frame);

bool rs02_make_operation_control(
    uint8_t motor_id,
    float position_rad,
    float velocity_rad_s,
    float torque_nm,
    float kp,
    float kd,
    rs02_frame_t *frame);

bool rs02_make_parameter_read(
    uint8_t motor_id,
    uint8_t host_id,
    uint16_t parameter_index,
    rs02_frame_t *frame);

bool rs02_make_parameter_write_u8(
    uint8_t motor_id,
    uint8_t host_id,
    uint16_t parameter_index,
    uint8_t value,
    rs02_frame_t *frame);

bool rs02_make_parameter_write_float(
    uint8_t motor_id,
    uint8_t host_id,
    uint16_t parameter_index,
    float value,
    rs02_frame_t *frame);

bool rs02_parse_device_id(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    rs02_device_info_t *device);

bool rs02_parse_feedback(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    uint8_t expected_host_id,
    rs02_feedback_t *feedback);

bool rs02_parse_parameter_float(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    uint8_t expected_host_id,
    uint16_t expected_parameter_index,
    float *value);

bool rs02_parse_parameter_u8(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    uint8_t expected_host_id,
    uint16_t expected_parameter_index,
    uint8_t *value);

// The manual's type-21 prose and identifier example swap the motor/host bytes.
// Accept either layout only when both configured IDs match exactly, so a fault
// frame is not silently ignored because of that documentation inconsistency.
bool rs02_parse_fault_feedback(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    uint8_t expected_host_id,
    rs02_fault_feedback_t *fault_feedback);

#ifdef __cplusplus
}
#endif
