#pragma once

#include "rs02_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RS02_ZERO_EFFORT_SPEED_GUARD_RAD_S 0.5f
#define RS02_ZERO_EFFORT_TORQUE_GUARD_NM 1.0f
#define RS02_ZERO_EFFORT_TRACKING_GUARD_RAD 0.10f
#define RS02_SOFTWARE_TEMPERATURE_GUARD_C 80.0f
#define RS02_BUS_VOLTAGE_MIN_V 24.0f
#define RS02_BUS_VOLTAGE_MAX_V 60.0f
#define RS02_MOTION_OFFSET_RAD 1.0471975512f
#define RS02_MOTION_LEG_DURATION_MS 3000U
#define RS02_MOTION_SETTLE_DURATION_MS 700U
#define RS02_MOTION_KP 12.0f
#define RS02_MOTION_KD 1.0f
#define RS02_MOTION_SPEED_GUARD_RAD_S 0.8f
#define RS02_MOTION_TORQUE_GUARD_NM 1.5f
#define RS02_MOTION_MOTOR_TORQUE_LIMIT_NM 1.0f
#define RS02_MOTION_EFFORT_GUARD_NM 1.0f
#define RS02_MOTION_MAX_CONFIGURED_TORQUE_NM 14.0f
#define RS02_MOTION_TORQUE_FEEDBACK_MARGIN_NM 0.5f
#define RS02_MOTION_MIN_CONFIGURED_SPEED_RAD_S 0.3f
#define RS02_MOTION_MAX_CONFIGURED_SPEED_RAD_S 33.0f
#define RS02_MOTION_TRACKING_GUARD_RAD 0.075f
#define RS02_MOTION_ENDPOINT_ERROR_RAD 0.035f

typedef struct {
    uint32_t duration_ms;
    float desired_velocity_rad_s;
    float feedforward_torque_nm;
    float kp;
    float kd;
} rs02_zero_effort_profile_t;

// Normal mode exercises the command/feedback path for 500 ms. Link-soak mode
// extends only the duration to 60 seconds; neither profile requests motion,
// damping, holding stiffness, or feed-forward effort.
bool rs02_zero_effort_profile(bool link_soak_enabled, rs02_zero_effort_profile_t *profile);

// Check the 24-60 V RS02 supply range reported by parameter 0x701C.
bool rs02_bus_voltage_is_safe(float bus_voltage_v);

// Conservative diagnostic guards for the enabled zero-effort phase. These are
// software anomaly trips, not certified motor, torque, speed, or thermal limits.
bool rs02_zero_effort_feedback_is_safe(
    const rs02_feedback_t *feedback,
    float initial_position_rad);

// Select a 60-degree endpoint inside the protocol position range. Prefer the
// positive direction and reverse only near the positive range boundary.
bool rs02_motion_endpoint(float origin_rad, float *endpoint_rad);

// Generate a smoothstep position/velocity pair. The velocity is zero at both
// endpoints, avoiding a step in the derivative when a leg begins or ends.
bool rs02_motion_sample(
    float start_rad,
    float end_rad,
    uint32_t elapsed_ms,
    uint32_t duration_ms,
    float *target_rad,
    float *desired_velocity_rad_s);

// Validate the latest feedback against the active motion target. These are
// provisional software trips and are not hardware torque or speed limits.
bool rs02_motion_feedback_is_safe(const rs02_feedback_t *feedback, float target_rad);

bool rs02_motion_feedback_is_safe_with_torque_limit(
    const rs02_feedback_t *feedback,
    float target_rad,
    float torque_limit_nm);

bool rs02_motion_feedback_is_safe_with_limits(
    const rs02_feedback_t *feedback,
    float target_rad,
    float torque_limit_nm,
    float speed_limit_rad_s);

// Estimate feed-forward + Kp position error + Kd velocity error and require
// its magnitude to remain strictly below the configured software guard.
bool rs02_motion_effort_is_safe(
    const rs02_feedback_t *feedback,
    float target_rad,
    float desired_velocity_rad_s,
    float *estimated_effort_nm);

bool rs02_motion_effort_is_safe_with_limit(
    const rs02_feedback_t *feedback,
    float target_rad,
    float desired_velocity_rad_s,
    float effort_limit_nm,
    float *estimated_effort_nm);

bool rs02_motion_endpoint_reached(const rs02_feedback_t *feedback, float endpoint_rad);

#ifdef __cplusplus
}
#endif
