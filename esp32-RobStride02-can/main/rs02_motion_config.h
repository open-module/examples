#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RS02_WEB_DEFAULT_ANGLE_DEG 60.0f
#define RS02_WEB_DEFAULT_DURATION_MS 3000U
#define RS02_WEB_DEFAULT_TORQUE_LIMIT_NM 1.0f
#define RS02_WEB_DEFAULT_SPEED_LIMIT_RAD_S 0.8f

#define RS02_WEB_MIN_SINGLE_TURN_ANGLE_DEG 0.0f
#define RS02_WEB_MAX_SINGLE_TURN_ANGLE_DEG 360.0f
#define RS02_RELATIVE_MIN_ABS_ANGLE_DEG 0.1f
#define RS02_RELATIVE_MAX_ABS_ANGLE_DEG 1432.3945f
#define RS02_WEB_MIN_DURATION_MS 300U
#define RS02_WEB_MAX_DURATION_MS 30000U
#define RS02_WEB_MIN_TORQUE_LIMIT_NM 0.1f
#define RS02_WEB_MAX_TORQUE_LIMIT_NM 14.0f
#define RS02_WEB_RATED_TORQUE_NM 6.0f
#define RS02_WEB_MIN_SPEED_LIMIT_RAD_S 0.3f
#define RS02_WEB_MAX_SPEED_LIMIT_RAD_S 33.0f
#define RS02_WEB_PROFILE_SPEED_MARGIN_RAD_S 0.25f
#define RS02_WEB_PROFILE_SPEED_PADDING_RAD_S 0.01f
#define RS02_WEB_VALIDATED_SPEED_LIMIT_RAD_S 0.8f

typedef struct {
    float angle_deg;
    float planned_delta_deg;
    uint32_t duration_ms;
    float torque_limit_nm;
    float speed_limit_rad_s;
    bool absolute_single_turn_target;
    bool return_to_origin;
} rs02_motion_config_t;

typedef enum {
    RS02_MOTION_CONFIG_OK = 0,
    RS02_MOTION_CONFIG_NONFINITE,
    RS02_MOTION_CONFIG_ANGLE_RANGE,
    RS02_MOTION_CONFIG_DURATION_RANGE,
    RS02_MOTION_CONFIG_TORQUE_RANGE,
    RS02_MOTION_CONFIG_SPEED_RANGE,
    RS02_MOTION_CONFIG_PROFILE_SPEED,
    RS02_MOTION_CONFIG_ENDPOINT_RANGE,
} rs02_motion_config_error_t;

rs02_motion_config_t rs02_motion_config_default(void);

bool rs02_motion_config_validate(
    const rs02_motion_config_t *config,
    float *peak_speed_rad_s,
    rs02_motion_config_error_t *error);

bool rs02_motion_config_resolve_endpoint(
    float control_origin_rad,
    float mechanical_position_rad,
    rs02_motion_config_t *config,
    float *current_single_turn_deg,
    float *endpoint_rad,
    rs02_motion_config_error_t *error);

const char *rs02_motion_config_error_name(rs02_motion_config_error_t error);

#ifdef __cplusplus
}
#endif
