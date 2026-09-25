#include "cybergear_protocol.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define CYBERGEAR_COMMUNICATION_TYPE_SHIFT 24U
#define CYBERGEAR_COMMUNICATION_TYPE_MASK 0x1FU
#define CYBERGEAR_DATA_SHIFT 8U
#define CYBERGEAR_FEEDBACK_MODE_SHIFT 22U
#define CYBERGEAR_FEEDBACK_MODE_MASK 0x03U
#define CYBERGEAR_FEEDBACK_FAULT_SHIFT 16U
#define CYBERGEAR_FEEDBACK_FAULT_MASK 0x3FU
#define CYBERGEAR_FEEDBACK_MOTOR_ID_SHIFT 8U
#define CYBERGEAR_DEVICE_ID_RESPONSE_TARGET 0xFEU

static uint16_t get_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void put_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)(value & 0xFFU);
}

static uint16_t float_to_u16(float value, float minimum, float maximum)
{
    if (value < minimum) {
        value = minimum;
    } else if (value > maximum) {
        value = maximum;
    }

    return (uint16_t)((value - minimum) * 65535.0f / (maximum - minimum));
}

static float u16_to_float(uint16_t value, float minimum, float maximum)
{
    return ((float)value * (maximum - minimum) / 65535.0f) + minimum;
}

static uint32_t make_identifier(
    cybergear_communication_type_t communication_type,
    uint16_t data,
    uint8_t motor_id)
{
    return ((uint32_t)communication_type << CYBERGEAR_COMMUNICATION_TYPE_SHIFT)
         | ((uint32_t)data << CYBERGEAR_DATA_SHIFT)
         | motor_id;
}

static bool initialize_frame(
    cybergear_frame_t *frame,
    cybergear_communication_type_t communication_type,
    uint16_t data,
    uint8_t motor_id)
{
    if (frame == NULL) {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->identifier = make_identifier(communication_type, data, motor_id);
    frame->data_length_code = CYBERGEAR_CAN_PAYLOAD_SIZE;
    return true;
}

static bool has_type(const cybergear_frame_t *frame, cybergear_communication_type_t expected)
{
    return frame != NULL &&
           frame->identifier <= CYBERGEAR_CAN_ID_MAX &&
           frame->data_length_code == CYBERGEAR_CAN_PAYLOAD_SIZE &&
           ((frame->identifier >> CYBERGEAR_COMMUNICATION_TYPE_SHIFT) &
            CYBERGEAR_COMMUNICATION_TYPE_MASK) == (uint32_t)expected;
}

bool cybergear_make_device_id_request(
    uint8_t motor_id,
    uint8_t host_id,
    cybergear_frame_t *frame)
{
    return initialize_frame(frame, CYBERGEAR_COMM_DEVICE_ID, host_id, motor_id);
}

bool cybergear_make_enable(uint8_t motor_id, uint8_t host_id, cybergear_frame_t *frame)
{
    return initialize_frame(frame, CYBERGEAR_COMM_ENABLE, host_id, motor_id);
}

bool cybergear_make_stop(
    uint8_t motor_id,
    uint8_t host_id,
    bool clear_fault,
    cybergear_frame_t *frame)
{
    if (!initialize_frame(frame, CYBERGEAR_COMM_STOP, host_id, motor_id)) {
        return false;
    }

    frame->data[0] = clear_fault ? 1U : 0U;
    return true;
}

bool cybergear_make_motion_control(
    uint8_t motor_id,
    float position_rad,
    float velocity_rad_s,
    float torque_nm,
    float kp,
    float kd,
    cybergear_frame_t *frame)
{
    if (!isfinite(position_rad) || !isfinite(velocity_rad_s) || !isfinite(torque_nm) ||
        !isfinite(kp) || !isfinite(kd)) {
        return false;
    }

    const uint16_t torque_wire = float_to_u16(
        torque_nm,
        CYBERGEAR_TORQUE_MIN_NM,
        CYBERGEAR_TORQUE_MAX_NM);
    if (!initialize_frame(frame, CYBERGEAR_COMM_MOTION_CONTROL, torque_wire, motor_id)) {
        return false;
    }

    put_u16_be(
        &frame->data[0],
        float_to_u16(position_rad, CYBERGEAR_POSITION_MIN_RAD, CYBERGEAR_POSITION_MAX_RAD));
    put_u16_be(
        &frame->data[2],
        float_to_u16(
            velocity_rad_s,
            CYBERGEAR_VELOCITY_MIN_RAD_S,
            CYBERGEAR_VELOCITY_MAX_RAD_S));
    put_u16_be(&frame->data[4], float_to_u16(kp, CYBERGEAR_KP_MIN, CYBERGEAR_KP_MAX));
    put_u16_be(&frame->data[6], float_to_u16(kd, CYBERGEAR_KD_MIN, CYBERGEAR_KD_MAX));
    return true;
}

bool cybergear_parse_device_id(
    const cybergear_frame_t *frame,
    cybergear_device_info_t *device)
{
    if (device == NULL || !has_type(frame, CYBERGEAR_COMM_DEVICE_ID) ||
        (uint8_t)frame->identifier != CYBERGEAR_DEVICE_ID_RESPONSE_TARGET) {
        return false;
    }

    device->motor_id = (uint8_t)(frame->identifier >> CYBERGEAR_FEEDBACK_MOTOR_ID_SHIFT);
    memcpy(device->mcu_uid, frame->data, sizeof(device->mcu_uid));
    return true;
}

bool cybergear_parse_feedback(
    const cybergear_frame_t *frame,
    uint8_t expected_host_id,
    cybergear_feedback_t *feedback)
{
    if (feedback == NULL || !has_type(frame, CYBERGEAR_COMM_FEEDBACK) ||
        (uint8_t)frame->identifier != expected_host_id) {
        return false;
    }

    feedback->motor_id = (uint8_t)(frame->identifier >> CYBERGEAR_FEEDBACK_MOTOR_ID_SHIFT);
    feedback->host_id = (uint8_t)frame->identifier;
    feedback->mode = (cybergear_mode_t)(
        (frame->identifier >> CYBERGEAR_FEEDBACK_MODE_SHIFT) & CYBERGEAR_FEEDBACK_MODE_MASK);
    feedback->faults = (uint8_t)(
        (frame->identifier >> CYBERGEAR_FEEDBACK_FAULT_SHIFT) & CYBERGEAR_FEEDBACK_FAULT_MASK);
    feedback->position_rad = u16_to_float(
        get_u16_be(&frame->data[0]),
        CYBERGEAR_POSITION_MIN_RAD,
        CYBERGEAR_POSITION_MAX_RAD);
    feedback->velocity_rad_s = u16_to_float(
        get_u16_be(&frame->data[2]),
        CYBERGEAR_VELOCITY_MIN_RAD_S,
        CYBERGEAR_VELOCITY_MAX_RAD_S);
    feedback->torque_nm = u16_to_float(
        get_u16_be(&frame->data[4]),
        CYBERGEAR_TORQUE_MIN_NM,
        CYBERGEAR_TORQUE_MAX_NM);
    feedback->temperature_c = (float)get_u16_be(&frame->data[6]) / 10.0f;
    return true;
}

bool cybergear_parse_fault_feedback(
    const cybergear_frame_t *frame,
    uint8_t expected_host_id,
    cybergear_fault_feedback_t *fault_feedback)
{
    if (fault_feedback == NULL || !has_type(frame, CYBERGEAR_COMM_FAULT_FEEDBACK) ||
        (uint8_t)(frame->identifier >> CYBERGEAR_DATA_SHIFT) != expected_host_id) {
        return false;
    }

    memset(fault_feedback, 0, sizeof(*fault_feedback));
    fault_feedback->motor_id = (uint8_t)frame->identifier;
    fault_feedback->host_id = (uint8_t)(frame->identifier >> CYBERGEAR_DATA_SHIFT);
    memcpy(fault_feedback->fault_data, &frame->data[0], sizeof(fault_feedback->fault_data));
    memcpy(fault_feedback->warning_data, &frame->data[4], sizeof(fault_feedback->warning_data));

    for (size_t i = 0; i < sizeof(fault_feedback->fault_data); ++i) {
        fault_feedback->has_fault = fault_feedback->has_fault || fault_feedback->fault_data[i] != 0U;
        fault_feedback->has_warning =
            fault_feedback->has_warning || fault_feedback->warning_data[i] != 0U;
    }
    return true;
}
