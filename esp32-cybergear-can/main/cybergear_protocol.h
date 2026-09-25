#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CYBERGEAR_CAN_PAYLOAD_SIZE 8U
#define CYBERGEAR_CAN_ID_MAX 0x1FFFFFFFU

#define CYBERGEAR_POSITION_MIN_RAD (-12.566370614359172f)
#define CYBERGEAR_POSITION_MAX_RAD 12.566370614359172f
#define CYBERGEAR_VELOCITY_MIN_RAD_S (-30.0f)
#define CYBERGEAR_VELOCITY_MAX_RAD_S 30.0f
#define CYBERGEAR_TORQUE_MIN_NM (-12.0f)
#define CYBERGEAR_TORQUE_MAX_NM 12.0f
#define CYBERGEAR_KP_MIN 0.0f
#define CYBERGEAR_KP_MAX 500.0f
#define CYBERGEAR_KD_MIN 0.0f
#define CYBERGEAR_KD_MAX 5.0f

typedef enum {
    CYBERGEAR_COMM_DEVICE_ID = 0,
    CYBERGEAR_COMM_MOTION_CONTROL = 1,
    CYBERGEAR_COMM_FEEDBACK = 2,
    CYBERGEAR_COMM_ENABLE = 3,
    CYBERGEAR_COMM_STOP = 4,
    CYBERGEAR_COMM_FAULT_FEEDBACK = 21,
} cybergear_communication_type_t;

typedef enum {
    CYBERGEAR_MODE_RESET = 0,
    CYBERGEAR_MODE_CALIBRATION = 1,
    CYBERGEAR_MODE_MOTOR = 2,
} cybergear_mode_t;

typedef enum {
    CYBERGEAR_FAULT_UNDER_VOLTAGE = 1U << 0,
    CYBERGEAR_FAULT_OVER_CURRENT = 1U << 1,
    CYBERGEAR_FAULT_OVER_TEMPERATURE = 1U << 2,
    CYBERGEAR_FAULT_MAGNETIC_ENCODER = 1U << 3,
    CYBERGEAR_FAULT_HALL_ENCODER = 1U << 4,
    CYBERGEAR_FAULT_UNCALIBRATED = 1U << 5,
} cybergear_fault_t;

typedef struct {
    uint32_t identifier;
    uint8_t data[CYBERGEAR_CAN_PAYLOAD_SIZE];
    uint8_t data_length_code;
} cybergear_frame_t;

typedef struct {
    uint8_t motor_id;
    uint8_t mcu_uid[CYBERGEAR_CAN_PAYLOAD_SIZE];
} cybergear_device_info_t;

typedef struct {
    uint8_t motor_id;
    uint8_t host_id;
    cybergear_mode_t mode;
    uint8_t faults;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} cybergear_feedback_t;

typedef struct {
    uint8_t motor_id;
    uint8_t host_id;
    uint8_t fault_data[4];
    uint8_t warning_data[4];
    bool has_fault;
    bool has_warning;
} cybergear_fault_feedback_t;

bool cybergear_make_device_id_request(
    uint8_t motor_id,
    uint8_t host_id,
    cybergear_frame_t *frame);

bool cybergear_make_enable(uint8_t motor_id, uint8_t host_id, cybergear_frame_t *frame);

bool cybergear_make_stop(
    uint8_t motor_id,
    uint8_t host_id,
    bool clear_fault,
    cybergear_frame_t *frame);

bool cybergear_make_motion_control(
    uint8_t motor_id,
    float position_rad,
    float velocity_rad_s,
    float torque_nm,
    float kp,
    float kd,
    cybergear_frame_t *frame);

bool cybergear_parse_device_id(
    const cybergear_frame_t *frame,
    cybergear_device_info_t *device);

bool cybergear_parse_feedback(
    const cybergear_frame_t *frame,
    uint8_t expected_host_id,
    cybergear_feedback_t *feedback);

bool cybergear_parse_fault_feedback(
    const cybergear_frame_t *frame,
    uint8_t expected_host_id,
    cybergear_fault_feedback_t *fault_feedback);

#ifdef __cplusplus
}
#endif
