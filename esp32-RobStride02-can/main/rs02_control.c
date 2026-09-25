#include "rs02_control.h"

#include <math.h>
#include <stddef.h>

static bool position_is_valid(float position_rad)
{
    return isfinite(position_rad) && position_rad >= RS02_POSITION_MIN_RAD &&
           position_rad <= RS02_POSITION_MAX_RAD;
}

bool rs02_zero_effort_profile(bool link_soak_enabled, rs02_zero_effort_profile_t *profile)
{
    if (profile == NULL) {
        return false;
    }

    *profile = (rs02_zero_effort_profile_t) {
        .duration_ms = link_soak_enabled ? 60000U : 500U,
        .desired_velocity_rad_s = 0.0f,
        .feedforward_torque_nm = 0.0f,
        .kp = 0.0f,
        .kd = 0.0f,
    };
    return true;
}

bool rs02_bus_voltage_is_safe(float bus_voltage_v)
{
    return isfinite(bus_voltage_v) && bus_voltage_v >= RS02_BUS_VOLTAGE_MIN_V &&
           bus_voltage_v <= RS02_BUS_VOLTAGE_MAX_V;
}

bool rs02_zero_effort_feedback_is_safe(
    const rs02_feedback_t *feedback,
    float initial_position_rad)
{
    return feedback != NULL && feedback->mode == RS02_MODE_MOTOR &&
           feedback->faults == 0U && position_is_valid(initial_position_rad) &&
           position_is_valid(feedback->position_rad) &&
           fabsf(feedback->position_rad - initial_position_rad) <
               RS02_ZERO_EFFORT_TRACKING_GUARD_RAD &&
           isfinite(feedback->velocity_rad_s) &&
           feedback->velocity_rad_s >= RS02_VELOCITY_MIN_RAD_S &&
           feedback->velocity_rad_s <= RS02_VELOCITY_MAX_RAD_S &&
           fabsf(feedback->velocity_rad_s) < RS02_ZERO_EFFORT_SPEED_GUARD_RAD_S &&
           isfinite(feedback->torque_nm) && feedback->torque_nm >= RS02_TORQUE_MIN_NM &&
           feedback->torque_nm <= RS02_TORQUE_MAX_NM &&
           fabsf(feedback->torque_nm) < RS02_ZERO_EFFORT_TORQUE_GUARD_NM &&
           isfinite(feedback->temperature_c) && feedback->temperature_c >= -20.0f &&
           feedback->temperature_c < RS02_SOFTWARE_TEMPERATURE_GUARD_C;
}

bool rs02_motion_endpoint(float origin_rad, float *endpoint_rad)
{
    if (endpoint_rad == NULL || !position_is_valid(origin_rad)) {
        return false;
    }

    const float positive_endpoint = origin_rad + RS02_MOTION_OFFSET_RAD;
    if (positive_endpoint <= RS02_POSITION_MAX_RAD) {
        *endpoint_rad = positive_endpoint;
        return true;
    }

    const float negative_endpoint = origin_rad - RS02_MOTION_OFFSET_RAD;
    if (negative_endpoint >= RS02_POSITION_MIN_RAD) {
        *endpoint_rad = negative_endpoint;
        return true;
    }
    return false;
}

bool rs02_motion_sample(
    float start_rad,
    float end_rad,
    uint32_t elapsed_ms,
    uint32_t duration_ms,
    float *target_rad,
    float *desired_velocity_rad_s)
{
    if (target_rad == NULL || desired_velocity_rad_s == NULL || duration_ms == 0U ||
        !position_is_valid(start_rad) || !position_is_valid(end_rad)) {
        return false;
    }

    const float progress = elapsed_ms >= duration_ms
                               ? 1.0f
                               : (float)elapsed_ms / (float)duration_ms;
    const float smooth_progress = progress * progress * (3.0f - 2.0f * progress);
    const float derivative = 6.0f * progress * (1.0f - progress);
    const float displacement = end_rad - start_rad;
    const float target = start_rad + displacement * smooth_progress;
    const float velocity = displacement * derivative * 1000.0f / (float)duration_ms;
    if (!position_is_valid(target) || !isfinite(velocity) ||
        velocity < RS02_VELOCITY_MIN_RAD_S || velocity > RS02_VELOCITY_MAX_RAD_S) {
        return false;
    }

    *target_rad = target;
    *desired_velocity_rad_s = velocity;
    return true;
}

bool rs02_motion_feedback_is_safe(const rs02_feedback_t *feedback, float target_rad)
{
    return feedback != NULL && feedback->mode == RS02_MODE_MOTOR &&
           feedback->faults == 0U && position_is_valid(target_rad) &&
           position_is_valid(feedback->position_rad) &&
           fabsf(feedback->position_rad - target_rad) < RS02_MOTION_TRACKING_GUARD_RAD &&
           isfinite(feedback->velocity_rad_s) &&
           feedback->velocity_rad_s >= RS02_VELOCITY_MIN_RAD_S &&
           feedback->velocity_rad_s <= RS02_VELOCITY_MAX_RAD_S &&
           fabsf(feedback->velocity_rad_s) < RS02_MOTION_SPEED_GUARD_RAD_S &&
           isfinite(feedback->torque_nm) && feedback->torque_nm >= RS02_TORQUE_MIN_NM &&
           feedback->torque_nm <= RS02_TORQUE_MAX_NM &&
           fabsf(feedback->torque_nm) < RS02_MOTION_TORQUE_GUARD_NM &&
           isfinite(feedback->temperature_c) && feedback->temperature_c >= -20.0f &&
           feedback->temperature_c < RS02_SOFTWARE_TEMPERATURE_GUARD_C;
}

bool rs02_motion_effort_is_safe(
    const rs02_feedback_t *feedback,
    float target_rad,
    float desired_velocity_rad_s,
    float *estimated_effort_nm)
{
    if (estimated_effort_nm == NULL) {
        return false;
    }
    *estimated_effort_nm = NAN;
    if (feedback == NULL || feedback->mode != RS02_MODE_MOTOR ||
        !position_is_valid(target_rad) || !position_is_valid(feedback->position_rad) ||
        !isfinite(desired_velocity_rad_s) ||
        desired_velocity_rad_s < RS02_VELOCITY_MIN_RAD_S ||
        desired_velocity_rad_s > RS02_VELOCITY_MAX_RAD_S ||
        !isfinite(feedback->velocity_rad_s) ||
        feedback->velocity_rad_s < RS02_VELOCITY_MIN_RAD_S ||
        feedback->velocity_rad_s > RS02_VELOCITY_MAX_RAD_S) {
        return false;
    }

    const float estimate =
        RS02_MOTION_KP * (target_rad - feedback->position_rad) +
        RS02_MOTION_KD * (desired_velocity_rad_s - feedback->velocity_rad_s);
    *estimated_effort_nm = estimate;
    return isfinite(estimate) && fabsf(estimate) < RS02_MOTION_EFFORT_GUARD_NM;
}

bool rs02_motion_endpoint_reached(const rs02_feedback_t *feedback, float endpoint_rad)
{
    return feedback != NULL && position_is_valid(endpoint_rad) &&
           position_is_valid(feedback->position_rad) &&
           fabsf(feedback->position_rad - endpoint_rad) < RS02_MOTION_ENDPOINT_ERROR_RAD;
}
