#include "rs02_protocol.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define RS02_COMMUNICATION_TYPE_SHIFT 24U
#define RS02_COMMUNICATION_TYPE_MASK 0x1FU
#define RS02_DATA_SHIFT 8U
#define RS02_FEEDBACK_MODE_SHIFT 22U
#define RS02_FEEDBACK_MODE_MASK 0x03U
#define RS02_FEEDBACK_FAULT_SHIFT 16U
#define RS02_FEEDBACK_FAULT_MASK 0x3FU
#define RS02_FEEDBACK_MOTOR_ID_SHIFT 8U
#define RS02_PARAMETER_STATUS_SHIFT 16U
#define RS02_PARAMETER_MOTOR_ID_SHIFT 8U
#define RS02_DEVICE_ID_RESPONSE_TARGET 0xFEU

static uint16_t get_u16_be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static uint16_t get_u16_le(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[1] << 8U) | data[0]);
}

static uint32_t get_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static void put_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)(value & 0xFFU);
}

static void put_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static void put_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)(value >> 24U);
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
    rs02_communication_type_t communication_type,
    uint16_t data,
    uint8_t destination_id)
{
    return ((uint32_t)communication_type << RS02_COMMUNICATION_TYPE_SHIFT) |
           ((uint32_t)data << RS02_DATA_SHIFT) |
           destination_id;
}

static bool initialize_frame(
    rs02_frame_t *frame,
    rs02_communication_type_t communication_type,
    uint16_t data,
    uint8_t destination_id)
{
    if (frame == NULL) {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->identifier = make_identifier(communication_type, data, destination_id);
    frame->data_length_code = RS02_CAN_PAYLOAD_SIZE;
    return true;
}

static bool has_type(const rs02_frame_t *frame, rs02_communication_type_t expected)
{
    return frame != NULL &&
           frame->identifier <= RS02_CAN_ID_MAX &&
           frame->data_length_code == RS02_CAN_PAYLOAD_SIZE &&
           ((frame->identifier >> RS02_COMMUNICATION_TYPE_SHIFT) &
            RS02_COMMUNICATION_TYPE_MASK) == (uint32_t)expected;
}

bool rs02_make_device_id_request(uint8_t motor_id, uint8_t host_id, rs02_frame_t *frame)
{
    return initialize_frame(frame, RS02_COMM_DEVICE_ID, host_id, motor_id);
}

bool rs02_make_enable(uint8_t motor_id, uint8_t host_id, rs02_frame_t *frame)
{
    return initialize_frame(frame, RS02_COMM_ENABLE, host_id, motor_id);
}

bool rs02_make_stop(uint8_t motor_id, uint8_t host_id, bool clear_fault, rs02_frame_t *frame)
{
    if (!initialize_frame(frame, RS02_COMM_STOP, host_id, motor_id)) {
        return false;
    }

    frame->data[0] = clear_fault ? 1U : 0U;
    return true;
}

bool rs02_make_operation_control(
    uint8_t motor_id,
    float position_rad,
    float velocity_rad_s,
    float torque_nm,
    float kp,
    float kd,
    rs02_frame_t *frame)
{
    if (!isfinite(position_rad) || !isfinite(velocity_rad_s) || !isfinite(torque_nm) ||
        !isfinite(kp) || !isfinite(kd)) {
        return false;
    }

    const uint16_t torque_wire =
        float_to_u16(torque_nm, RS02_TORQUE_MIN_NM, RS02_TORQUE_MAX_NM);
    if (!initialize_frame(frame, RS02_COMM_OPERATION_CONTROL, torque_wire, motor_id)) {
        return false;
    }

    put_u16_be(
        &frame->data[0],
        float_to_u16(position_rad, RS02_POSITION_MIN_RAD, RS02_POSITION_MAX_RAD));
    put_u16_be(
        &frame->data[2],
        float_to_u16(velocity_rad_s, RS02_VELOCITY_MIN_RAD_S, RS02_VELOCITY_MAX_RAD_S));
    put_u16_be(&frame->data[4], float_to_u16(kp, RS02_KP_MIN, RS02_KP_MAX));
    put_u16_be(&frame->data[6], float_to_u16(kd, RS02_KD_MIN, RS02_KD_MAX));
    return true;
}

bool rs02_make_parameter_read(
    uint8_t motor_id,
    uint8_t host_id,
    uint16_t parameter_index,
    rs02_frame_t *frame)
{
    if (!initialize_frame(frame, RS02_COMM_PARAMETER_READ, host_id, motor_id)) {
        return false;
    }
    put_u16_le(&frame->data[0], parameter_index);
    return true;
}

bool rs02_make_parameter_write_u8(
    uint8_t motor_id,
    uint8_t host_id,
    uint16_t parameter_index,
    uint8_t value,
    rs02_frame_t *frame)
{
    if (!initialize_frame(frame, RS02_COMM_PARAMETER_WRITE, host_id, motor_id)) {
        return false;
    }
    put_u16_le(&frame->data[0], parameter_index);
    frame->data[4] = value;
    return true;
}

bool rs02_make_parameter_write_float(
    uint8_t motor_id,
    uint8_t host_id,
    uint16_t parameter_index,
    float value,
    rs02_frame_t *frame)
{
    if (!isfinite(value) ||
        !initialize_frame(frame, RS02_COMM_PARAMETER_WRITE, host_id, motor_id)) {
        return false;
    }
    put_u16_le(&frame->data[0], parameter_index);
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    put_u32_le(&frame->data[4], bits);
    return true;
}

bool rs02_parse_device_id(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    rs02_device_info_t *device)
{
    if (device == NULL || !has_type(frame, RS02_COMM_DEVICE_ID) ||
        (uint8_t)frame->identifier != RS02_DEVICE_ID_RESPONSE_TARGET ||
        (uint8_t)(frame->identifier >> RS02_FEEDBACK_MOTOR_ID_SHIFT) != expected_motor_id) {
        return false;
    }

    device->motor_id = expected_motor_id;
    memcpy(device->mcu_uid, frame->data, sizeof(device->mcu_uid));
    return true;
}

bool rs02_parse_feedback(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    uint8_t expected_host_id,
    rs02_feedback_t *feedback)
{
    if (feedback == NULL || !has_type(frame, RS02_COMM_FEEDBACK) ||
        (uint8_t)frame->identifier != expected_host_id ||
        (uint8_t)(frame->identifier >> RS02_FEEDBACK_MOTOR_ID_SHIFT) != expected_motor_id) {
        return false;
    }

    feedback->motor_id = expected_motor_id;
    feedback->host_id = expected_host_id;
    feedback->mode = (rs02_mode_t)(
        (frame->identifier >> RS02_FEEDBACK_MODE_SHIFT) & RS02_FEEDBACK_MODE_MASK);
    feedback->faults = (uint8_t)(
        (frame->identifier >> RS02_FEEDBACK_FAULT_SHIFT) & RS02_FEEDBACK_FAULT_MASK);
    feedback->position_rad =
        u16_to_float(get_u16_be(&frame->data[0]), RS02_POSITION_MIN_RAD, RS02_POSITION_MAX_RAD);
    feedback->velocity_rad_s =
        u16_to_float(get_u16_be(&frame->data[2]), RS02_VELOCITY_MIN_RAD_S, RS02_VELOCITY_MAX_RAD_S);
    feedback->torque_nm =
        u16_to_float(get_u16_be(&frame->data[4]), RS02_TORQUE_MIN_NM, RS02_TORQUE_MAX_NM);
    feedback->temperature_c = (float)get_u16_be(&frame->data[6]) / 10.0f;
    return true;
}

bool rs02_parse_parameter_float(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    uint8_t expected_host_id,
    uint16_t expected_parameter_index,
    float *value)
{
    if (value == NULL || !has_type(frame, RS02_COMM_PARAMETER_READ) ||
        (uint8_t)frame->identifier != expected_host_id ||
        (uint8_t)(frame->identifier >> RS02_PARAMETER_MOTOR_ID_SHIFT) != expected_motor_id ||
        (uint8_t)(frame->identifier >> RS02_PARAMETER_STATUS_SHIFT) != 0U ||
        get_u16_le(&frame->data[0]) != expected_parameter_index ||
        frame->data[2] != 0U || frame->data[3] != 0U) {
        return false;
    }

    const uint32_t bits = get_u32_le(&frame->data[4]);
    memcpy(value, &bits, sizeof(*value));
    return isfinite(*value);
}

bool rs02_parse_parameter_u8(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    uint8_t expected_host_id,
    uint16_t expected_parameter_index,
    uint8_t *value)
{
    if (value == NULL || !has_type(frame, RS02_COMM_PARAMETER_READ) ||
        (uint8_t)frame->identifier != expected_host_id ||
        (uint8_t)(frame->identifier >> RS02_PARAMETER_MOTOR_ID_SHIFT) != expected_motor_id ||
        (uint8_t)(frame->identifier >> RS02_PARAMETER_STATUS_SHIFT) != 0U ||
        get_u16_le(&frame->data[0]) != expected_parameter_index ||
        frame->data[2] != 0U || frame->data[3] != 0U ||
        frame->data[5] != 0U || frame->data[6] != 0U || frame->data[7] != 0U) {
        return false;
    }

    *value = frame->data[4];
    return true;
}

bool rs02_parse_fault_feedback(
    const rs02_frame_t *frame,
    uint8_t expected_motor_id,
    uint8_t expected_host_id,
    rs02_fault_feedback_t *fault_feedback)
{
    if (fault_feedback == NULL || !has_type(frame, RS02_COMM_FAULT_FEEDBACK)) {
        return false;
    }

    const uint8_t middle_id = (uint8_t)(frame->identifier >> RS02_DATA_SHIFT);
    const uint8_t low_id = (uint8_t)frame->identifier;
    const bool prose_layout = middle_id == expected_motor_id && low_id == expected_host_id;
    const bool example_layout = middle_id == expected_host_id && low_id == expected_motor_id;
    if (!prose_layout && !example_layout) {
        return false;
    }

    fault_feedback->motor_id = expected_motor_id;
    fault_feedback->host_id = expected_host_id;
    fault_feedback->faults = get_u32_le(&frame->data[0]);
    fault_feedback->warnings = get_u32_le(&frame->data[4]);
    return true;
}
