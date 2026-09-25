#pragma once

#include "cybergear_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// CYBERGEAR_MOTION_OFFSET_RAD is the explicitly opted-in 360-degree excursion.
#define CYBERGEAR_MOTION_OFFSET_RAD 6.28318530717958647692f
// CYBERGEAR_MOTION_DURATION_MS gives a 36-degree/s target slope, not an actual speed limit.
#define CYBERGEAR_MOTION_DURATION_MS 10000U
// CYBERGEAR_ZERO_EFFORT_SPEED_GUARD_RAD_S preserves the default stationary-phase guard.
#define CYBERGEAR_ZERO_EFFORT_SPEED_GUARD_RAD_S 0.5f
// CYBERGEAR_MOTION_SPEED_GUARD_RAD_S allows the 0.6283 rad/s target slope with a provisional trip margin.
#define CYBERGEAR_MOTION_SPEED_GUARD_RAD_S 1.0f
// These are strict software feedback trips, not motor current or torque limits.
#define CYBERGEAR_ZERO_EFFORT_TORQUE_GUARD_NM 1.0f
#define CYBERGEAR_MOTION_TORQUE_GUARD_NM 10.0f
// The variable-load profile raises stiffness while retaining zero feed-forward torque.
#define CYBERGEAR_MOTION_KP 90.0f
#define CYBERGEAR_MOTION_KD 1.0f

typedef struct {
    uint32_t duration_ms;
    float target_offset_rad;
    float desired_velocity_rad_s;
    float feedforward_torque_nm;
    float kp;
    float kd;
    float speed_guard_rad_s;
} cybergear_zero_effort_profile_t;

// Return the stationary zero-effort profile. The normal profile lasts 500 ms;
// link-soak mode changes only the duration to 60 seconds so the same 50 Hz
// command/feedback path can expose intermittent CAN receive errors without
// commanding displacement or effort. Null output is rejected.
bool cybergear_zero_effort_profile(bool link_soak_enabled,
                                   cybergear_zero_effort_profile_t *profile);

// cybergear_motion_endpoint chooses origin +2pi rad, or origin -2pi rad if the positive step exceeds
// the protocol limit. Reject nonfinite/out-of-range origins and null output.
bool cybergear_motion_endpoint(float origin_rad, float *endpoint_rad);

// cybergear_motion_target linearly interpolates valid protocol-range endpoints over a nonzero duration.
// elapsed=0 returns the exact start; elapsed>=duration returns the exact end.
// Reject null output, nonfinite/out-of-range endpoints, and zero duration.
// Targets do not impose hard limits on actual speed, torque, or tracking error.
bool cybergear_motion_target(float start_rad, float end_rad, uint32_t elapsed_ms,
                             uint32_t duration_ms, float *target_rad);

// Derive the signed velocity target for a constant-slope motion leg. Reject
// invalid positions, zero duration, nonfinite/out-of-protocol results, and null output.
bool cybergear_motion_desired_velocity(float start_rad, float end_rad, uint32_t duration_ms,
                                       float *velocity_rad_s);

// Estimate the complete controller effort for the next outgoing command from
// the latest Motor-mode feedback:
//   feed-forward + Kp * (target - measured position)
//                + Kd * (desired - measured velocity)
// Return true only when every input is finite and within its protocol range and
// |estimated effort| is strictly below effort_limit_nm. The valid estimate is
// written even when it reaches the limit so callers can diagnose the abort.
bool cybergear_motion_command_effort_is_safe(
    const cybergear_feedback_t *feedback,
    float target_rad,
    float desired_velocity_rad_s,
    float feedforward_torque_nm,
    float kp,
    float kd,
    float effort_limit_nm,
    float *estimated_effort_nm);

// cybergear_control_feedback_is_safe is a provisional anomaly guard, not a certified safety limit: Motor mode, finite
// protocol-range target/position/velocity/torque, |velocity| <velocity_limit_rad_s,
// |torque| <torque_limit_nm and |position-target| <0.10 rad. Equality is rejected.
// Null feedback is rejected. Limits must be finite, positive, and at most their
// protocol maxima.
// Use the current interpolated target, not the fixed origin, on both sides of transmission.
// The caller retains identity, freshness, fault, temperature, and TWAI checks.
bool cybergear_control_feedback_is_safe(const cybergear_feedback_t *feedback, float target_rad,
                                        float velocity_limit_rad_s, float torque_limit_nm);

#ifdef __cplusplus
}
#endif
