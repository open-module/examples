#include "rs02_motion_config.h"

#include <math.h>
#include <stddef.h>

#include "rs02_protocol.h"

#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)
#define SINGLE_TURN_BOUNDARY_EPSILON_RAD 0.0001f

static void set_error(rs02_motion_config_error_t *error, rs02_motion_config_error_t value)
{
    if (error != NULL) {
        *error = value;
    }
}

rs02_motion_config_t rs02_motion_config_default(void)
{
    return (rs02_motion_config_t) {
        .angle_deg = RS02_WEB_DEFAULT_ANGLE_DEG,
        .planned_delta_deg = RS02_WEB_DEFAULT_ANGLE_DEG,
        .duration_ms = RS02_WEB_DEFAULT_DURATION_MS,
        .torque_limit_nm = RS02_WEB_DEFAULT_TORQUE_LIMIT_NM,
        .speed_limit_rad_s = RS02_WEB_DEFAULT_SPEED_LIMIT_RAD_S,
        .absolute_single_turn_target = false,
        .return_to_origin = true,
    };
}

bool rs02_motion_config_validate(
    const rs02_motion_config_t *config,
    float *peak_speed_rad_s,
    rs02_motion_config_error_t *error)
{
    if (peak_speed_rad_s != NULL) {
        *peak_speed_rad_s = NAN;
    }
    set_error(error, RS02_MOTION_CONFIG_NONFINITE);
    if (config == NULL || !isfinite(config->angle_deg) ||
        !isfinite(config->planned_delta_deg) ||
        !isfinite(config->torque_limit_nm) || !isfinite(config->speed_limit_rad_s)) {
        return false;
    }

    const float absolute_angle_deg = fabsf(config->angle_deg);
    const float absolute_delta_deg = fabsf(config->planned_delta_deg);
    if (config->absolute_single_turn_target) {
        if (config->angle_deg < RS02_WEB_MIN_SINGLE_TURN_ANGLE_DEG ||
            config->angle_deg > RS02_WEB_MAX_SINGLE_TURN_ANGLE_DEG ||
            absolute_delta_deg > RS02_WEB_MAX_SINGLE_TURN_ANGLE_DEG) {
            set_error(error, RS02_MOTION_CONFIG_ANGLE_RANGE);
            return false;
        }
    } else if (absolute_angle_deg < RS02_RELATIVE_MIN_ABS_ANGLE_DEG ||
               absolute_angle_deg > RS02_RELATIVE_MAX_ABS_ANGLE_DEG ||
               fabsf(config->planned_delta_deg - config->angle_deg) > 0.001f) {
        set_error(error, RS02_MOTION_CONFIG_ANGLE_RANGE);
        return false;
    }
    if (config->duration_ms < RS02_WEB_MIN_DURATION_MS ||
        config->duration_ms > RS02_WEB_MAX_DURATION_MS) {
        set_error(error, RS02_MOTION_CONFIG_DURATION_RANGE);
        return false;
    }
    if (config->torque_limit_nm < RS02_WEB_MIN_TORQUE_LIMIT_NM ||
        config->torque_limit_nm > RS02_WEB_MAX_TORQUE_LIMIT_NM) {
        set_error(error, RS02_MOTION_CONFIG_TORQUE_RANGE);
        return false;
    }
    if (config->speed_limit_rad_s < RS02_WEB_MIN_SPEED_LIMIT_RAD_S ||
        config->speed_limit_rad_s > RS02_WEB_MAX_SPEED_LIMIT_RAD_S) {
        set_error(error, RS02_MOTION_CONFIG_SPEED_RANGE);
        return false;
    }

    const float angle_rad = absolute_delta_deg * PI_F / 180.0f;
    const float peak_speed = 1.5f * angle_rad * 1000.0f / (float)config->duration_ms;
    if (!isfinite(peak_speed) ||
        peak_speed + RS02_WEB_PROFILE_SPEED_MARGIN_RAD_S > config->speed_limit_rad_s) {
        set_error(error, RS02_MOTION_CONFIG_PROFILE_SPEED);
        return false;
    }

    if (peak_speed_rad_s != NULL) {
        *peak_speed_rad_s = peak_speed;
    }
    set_error(error, RS02_MOTION_CONFIG_OK);
    return true;
}

bool rs02_motion_config_resolve_endpoint(
    float control_origin_rad,
    float mechanical_position_rad,
    rs02_motion_config_t *config,
    float *current_single_turn_deg,
    float *endpoint_rad,
    rs02_motion_config_error_t *error)
{
    float unused_peak_speed = 0.0f;
    if (current_single_turn_deg != NULL) {
        *current_single_turn_deg = NAN;
    }
    if (endpoint_rad == NULL || !isfinite(control_origin_rad) ||
        control_origin_rad < RS02_POSITION_MIN_RAD || control_origin_rad > RS02_POSITION_MAX_RAD ||
        !isfinite(mechanical_position_rad) ||
        !rs02_motion_config_validate(config, &unused_peak_speed, error)) {
        if (endpoint_rad == NULL || !isfinite(control_origin_rad) ||
            control_origin_rad < RS02_POSITION_MIN_RAD ||
            control_origin_rad > RS02_POSITION_MAX_RAD ||
            !isfinite(mechanical_position_rad)) {
            set_error(error, RS02_MOTION_CONFIG_ENDPOINT_RANGE);
        }
        return false;
    }

    float signed_delta_rad = config->angle_deg * PI_F / 180.0f;
    if (config->absolute_single_turn_target) {
        float current_rad = fmodf(mechanical_position_rad, TWO_PI_F);
        if (current_rad < 0.0f) {
            current_rad += TWO_PI_F;
        }
        if (mechanical_position_rad > 0.0f &&
            mechanical_position_rad >= TWO_PI_F - SINGLE_TURN_BOUNDARY_EPSILON_RAD &&
            current_rad <= SINGLE_TURN_BOUNDARY_EPSILON_RAD) {
            current_rad = TWO_PI_F;
        }
        const float target_rad = config->angle_deg * PI_F / 180.0f;
        signed_delta_rad = target_rad - current_rad;
        config->planned_delta_deg = signed_delta_rad * 180.0f / PI_F;
        const float peak_speed =
            1.5f * fabsf(signed_delta_rad) * 1000.0f / (float)config->duration_ms;
        config->speed_limit_rad_s = fmaxf(
            RS02_WEB_MIN_SPEED_LIMIT_RAD_S,
            peak_speed + RS02_WEB_PROFILE_SPEED_MARGIN_RAD_S +
                RS02_WEB_PROFILE_SPEED_PADDING_RAD_S);
        if (current_single_turn_deg != NULL) {
            *current_single_turn_deg = current_rad * 180.0f / PI_F;
        }
        if (!rs02_motion_config_validate(config, &unused_peak_speed, error)) {
            return false;
        }
    }

    const float endpoint = control_origin_rad + signed_delta_rad;
    if (!isfinite(endpoint) || endpoint < RS02_POSITION_MIN_RAD || endpoint > RS02_POSITION_MAX_RAD) {
        set_error(error, RS02_MOTION_CONFIG_ENDPOINT_RANGE);
        return false;
    }

    *endpoint_rad = endpoint;
    set_error(error, RS02_MOTION_CONFIG_OK);
    return true;
}

const char *rs02_motion_config_error_name(rs02_motion_config_error_t error)
{
    switch (error) {
    case RS02_MOTION_CONFIG_OK: return "ok";
    case RS02_MOTION_CONFIG_NONFINITE:
        return "angle, torque, and speed limit must be finite numbers";
    case RS02_MOTION_CONFIG_ANGLE_RANGE:
        return "angle is outside the selected motion-planner range";
    case RS02_MOTION_CONFIG_DURATION_RANGE:
        return "duration must be from 0.3 to 30.0 seconds";
    case RS02_MOTION_CONFIG_TORQUE_RANGE:
        return "torque limit must be from 0.1 to 14.0 N m";
    case RS02_MOTION_CONFIG_SPEED_RANGE:
        return "speed limit must be from 0.3 to 33.0 rad/s";
    case RS02_MOTION_CONFIG_PROFILE_SPEED:
        return "target peak speed plus the 0.25 rad/s margin exceeds the software speed guard";
    case RS02_MOTION_CONFIG_ENDPOINT_RANGE:
        return "requested endpoint is outside the RS02 position command range";
    default: return "unknown motion configuration error";
    }
}
