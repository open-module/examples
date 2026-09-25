#include "cybergear_motion.h"

#include <math.h>
#include <stddef.h>

static bool position_is_valid(float position_rad)
{
    return isfinite(position_rad) && position_rad >= CYBERGEAR_POSITION_MIN_RAD &&
           position_rad <= CYBERGEAR_POSITION_MAX_RAD;
}

bool cybergear_zero_effort_profile(bool link_soak_enabled,
                                   cybergear_zero_effort_profile_t *profile)
{
    if (profile == NULL) {
        return false;
    }
    *profile = (cybergear_zero_effort_profile_t) {
        .duration_ms = link_soak_enabled ? 60000U : 500U,
        .target_offset_rad = 0.0f,
        .desired_velocity_rad_s = 0.0f,
        .feedforward_torque_nm = 0.0f,
        .kp = 0.0f,
        .kd = 0.0f,
        .speed_guard_rad_s = CYBERGEAR_ZERO_EFFORT_SPEED_GUARD_RAD_S,
    };
    return true;
}

bool cybergear_motion_endpoint(float origin_rad, float *endpoint_rad)
{
    if (endpoint_rad == NULL || !position_is_valid(origin_rad)) {
        return false;
    }
    const float positive_endpoint = origin_rad + CYBERGEAR_MOTION_OFFSET_RAD;
    *endpoint_rad = positive_endpoint <= CYBERGEAR_POSITION_MAX_RAD
                        ? positive_endpoint : origin_rad - CYBERGEAR_MOTION_OFFSET_RAD;
    return true;
}

bool cybergear_motion_target(float start_rad, float end_rad, uint32_t elapsed_ms,
                             uint32_t duration_ms, float *target_rad)
{
    if (target_rad == NULL || duration_ms == 0U ||
        !position_is_valid(start_rad) || !position_is_valid(end_rad)) {
        return false;
    }
    if (elapsed_ms == 0U) {
        *target_rad = start_rad;
    } else if (elapsed_ms >= duration_ms) {
        *target_rad = end_rad;
    } else {
        const float fraction = (float)elapsed_ms / (float)duration_ms;
        *target_rad = start_rad + (end_rad - start_rad) * fraction;
    }
    return true;
}

bool cybergear_motion_desired_velocity(float start_rad, float end_rad, uint32_t duration_ms,
                                       float *velocity_rad_s)
{
    if (velocity_rad_s == NULL || duration_ms == 0U ||
        !position_is_valid(start_rad) || !position_is_valid(end_rad)) {
        return false;
    }
    const float velocity = (end_rad - start_rad) * 1000.0f / (float)duration_ms;
    if (!isfinite(velocity) || velocity < CYBERGEAR_VELOCITY_MIN_RAD_S ||
        velocity > CYBERGEAR_VELOCITY_MAX_RAD_S) {
        return false;
    }
    *velocity_rad_s = velocity;
    return true;
}

bool cybergear_motion_command_effort_is_safe(
    const cybergear_feedback_t *feedback,
    float target_rad,
    float desired_velocity_rad_s,
    float feedforward_torque_nm,
    float kp,
    float kd,
    float effort_limit_nm,
    float *estimated_effort_nm)
{
    if (estimated_effort_nm == NULL) {
        return false;
    }
    *estimated_effort_nm = NAN;
    if (feedback == NULL || feedback->mode != CYBERGEAR_MODE_MOTOR ||
        !position_is_valid(target_rad) || !position_is_valid(feedback->position_rad) ||
        !isfinite(desired_velocity_rad_s) ||
        desired_velocity_rad_s < CYBERGEAR_VELOCITY_MIN_RAD_S ||
        desired_velocity_rad_s > CYBERGEAR_VELOCITY_MAX_RAD_S ||
        !isfinite(feedback->velocity_rad_s) ||
        feedback->velocity_rad_s < CYBERGEAR_VELOCITY_MIN_RAD_S ||
        feedback->velocity_rad_s > CYBERGEAR_VELOCITY_MAX_RAD_S ||
        !isfinite(feedforward_torque_nm) ||
        feedforward_torque_nm < CYBERGEAR_TORQUE_MIN_NM ||
        feedforward_torque_nm > CYBERGEAR_TORQUE_MAX_NM ||
        !isfinite(kp) || kp < CYBERGEAR_KP_MIN || kp > CYBERGEAR_KP_MAX ||
        !isfinite(kd) || kd < CYBERGEAR_KD_MIN || kd > CYBERGEAR_KD_MAX ||
        !isfinite(effort_limit_nm) || effort_limit_nm <= 0.0f ||
        effort_limit_nm > CYBERGEAR_TORQUE_MAX_NM) {
        return false;
    }

    const float estimated_effort =
        feedforward_torque_nm + kp * (target_rad - feedback->position_rad) +
        kd * (desired_velocity_rad_s - feedback->velocity_rad_s);
    *estimated_effort_nm = estimated_effort;
    return isfinite(estimated_effort) && fabsf(estimated_effort) < effort_limit_nm;
}

bool cybergear_control_feedback_is_safe(const cybergear_feedback_t *feedback, float target_rad,
                                        float velocity_limit_rad_s, float torque_limit_nm)
{
    return isfinite(velocity_limit_rad_s) && velocity_limit_rad_s > 0.0f &&
           velocity_limit_rad_s <= CYBERGEAR_VELOCITY_MAX_RAD_S &&
           isfinite(torque_limit_nm) && torque_limit_nm > 0.0f &&
           torque_limit_nm <= CYBERGEAR_TORQUE_MAX_NM &&
           feedback != NULL && feedback->mode == CYBERGEAR_MODE_MOTOR &&
           position_is_valid(target_rad) && position_is_valid(feedback->position_rad) &&
           isfinite(feedback->velocity_rad_s) &&
           feedback->velocity_rad_s >= CYBERGEAR_VELOCITY_MIN_RAD_S &&
           feedback->velocity_rad_s <= CYBERGEAR_VELOCITY_MAX_RAD_S &&
           fabsf(feedback->velocity_rad_s) < velocity_limit_rad_s &&
           isfinite(feedback->torque_nm) &&
           feedback->torque_nm >= CYBERGEAR_TORQUE_MIN_NM &&
           feedback->torque_nm <= CYBERGEAR_TORQUE_MAX_NM &&
           fabsf(feedback->torque_nm) < torque_limit_nm &&
           fabsf(feedback->position_rad - target_rad) < 0.10f;
}
