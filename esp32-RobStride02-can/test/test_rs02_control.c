#include "rs02_control.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned int s_failures;

#define CHECK(condition)                                                                    \
    do {                                                                                    \
        if (!(condition)) {                                                                 \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
            s_failures++;                                                                   \
        }                                                                                   \
    } while (0)

static rs02_feedback_t safe_feedback(void)
{
    return (rs02_feedback_t) {
        .motor_id = 0x7F,
        .host_id = 0xFD,
        .mode = RS02_MODE_MOTOR,
        .faults = 0,
        .position_rad = 0.0f,
        .velocity_rad_s = 0.0f,
        .torque_nm = 0.0f,
        .temperature_c = 25.0f,
    };
}

static void test_zero_effort_profiles(void)
{
    rs02_zero_effort_profile_t profile = {0};
    CHECK(rs02_zero_effort_profile(false, &profile));
    CHECK(profile.duration_ms == 500U);
    CHECK(profile.desired_velocity_rad_s == 0.0f);
    CHECK(profile.feedforward_torque_nm == 0.0f);
    CHECK(profile.kp == 0.0f);
    CHECK(profile.kd == 0.0f);

    CHECK(rs02_zero_effort_profile(true, &profile));
    CHECK(profile.duration_ms == 60000U);
    CHECK(profile.desired_velocity_rad_s == 0.0f);
    CHECK(profile.feedforward_torque_nm == 0.0f);
    CHECK(profile.kp == 0.0f);
    CHECK(profile.kd == 0.0f);
    CHECK(!rs02_zero_effort_profile(false, NULL));
}

static void test_bus_voltage_guard(void)
{
    CHECK(rs02_bus_voltage_is_safe(24.0f));
    CHECK(rs02_bus_voltage_is_safe(48.0f));
    CHECK(rs02_bus_voltage_is_safe(60.0f));
    CHECK(!rs02_bus_voltage_is_safe(nextafterf(24.0f, -INFINITY)));
    CHECK(!rs02_bus_voltage_is_safe(nextafterf(60.0f, INFINITY)));
    CHECK(!rs02_bus_voltage_is_safe(NAN));
    CHECK(!rs02_bus_voltage_is_safe(INFINITY));
}

static void test_zero_effort_feedback_guards(void)
{
    rs02_feedback_t feedback = safe_feedback();
    CHECK(rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
    CHECK(!rs02_zero_effort_feedback_is_safe(NULL, 0.0f));
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, NAN));

    feedback.mode = RS02_MODE_RESET;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
    feedback = safe_feedback();
    feedback.faults = RS02_FAULT_STALL_OVERLOAD;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));

    feedback = safe_feedback();
    feedback.position_rad = RS02_ZERO_EFFORT_TRACKING_GUARD_RAD;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
    feedback.position_rad = nextafterf(RS02_ZERO_EFFORT_TRACKING_GUARD_RAD, 0.0f);
    CHECK(rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));

    feedback = safe_feedback();
    feedback.velocity_rad_s = RS02_ZERO_EFFORT_SPEED_GUARD_RAD_S;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
    feedback.velocity_rad_s = -RS02_ZERO_EFFORT_SPEED_GUARD_RAD_S;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));

    feedback = safe_feedback();
    feedback.torque_nm = RS02_ZERO_EFFORT_TORQUE_GUARD_NM;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
    feedback.torque_nm = -RS02_ZERO_EFFORT_TORQUE_GUARD_NM;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));

    feedback = safe_feedback();
    feedback.temperature_c = RS02_SOFTWARE_TEMPERATURE_GUARD_C;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
    feedback.temperature_c = -20.1f;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
    feedback.temperature_c = NAN;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));

    feedback = safe_feedback();
    feedback.position_rad = nextafterf(RS02_POSITION_MAX_RAD, INFINITY);
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, RS02_POSITION_MAX_RAD));
    feedback = safe_feedback();
    feedback.velocity_rad_s = NAN;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
    feedback = safe_feedback();
    feedback.torque_nm = INFINITY;
    CHECK(!rs02_zero_effort_feedback_is_safe(&feedback, 0.0f));
}

static void test_motion_endpoint_and_smoothstep(void)
{
    float endpoint = 0.0f;
    CHECK(rs02_motion_endpoint(0.5f, &endpoint));
    CHECK(fabsf(endpoint - (0.5f + RS02_MOTION_OFFSET_RAD)) < 0.000001f);
    CHECK(rs02_motion_endpoint(RS02_POSITION_MAX_RAD, &endpoint));
    CHECK(fabsf(endpoint - (RS02_POSITION_MAX_RAD - RS02_MOTION_OFFSET_RAD)) < 0.000001f);
    CHECK(!rs02_motion_endpoint(NAN, &endpoint));
    CHECK(!rs02_motion_endpoint(0.0f, NULL));

    float target = 0.0f;
    float velocity = 0.0f;
    CHECK(rs02_motion_sample(0.0f, 1.0f, 0U, 3000U, &target, &velocity));
    CHECK(target == 0.0f);
    CHECK(velocity == 0.0f);
    CHECK(rs02_motion_sample(0.0f, 1.0f, 1500U, 3000U, &target, &velocity));
    CHECK(fabsf(target - 0.5f) < 0.000001f);
    CHECK(fabsf(velocity - 0.5f) < 0.000001f);
    CHECK(rs02_motion_sample(0.0f, 1.0f, 3000U, 3000U, &target, &velocity));
    CHECK(target == 1.0f);
    CHECK(velocity == 0.0f);
    CHECK(!rs02_motion_sample(0.0f, 1.0f, 0U, 0U, &target, &velocity));
    CHECK(!rs02_motion_sample(0.0f, 1.0f, 0U, 3000U, NULL, &velocity));

    CHECK(rs02_motion_sample(
        0.0f,
        RS02_MOTION_OFFSET_RAD,
        RS02_MOTION_LEG_DURATION_MS / 2U,
        RS02_MOTION_LEG_DURATION_MS,
        &target,
        &velocity));
    CHECK(fabsf(target - (RS02_MOTION_OFFSET_RAD / 2.0f)) < 0.000001f);
    CHECK(fabsf(velocity) < RS02_MOTION_SPEED_GUARD_RAD_S);
}

static void test_motion_feedback_and_effort_guards(void)
{
    rs02_feedback_t feedback = safe_feedback();
    float estimate = 0.0f;

    CHECK(rs02_motion_feedback_is_safe(&feedback, 0.0f));
    CHECK(rs02_motion_effort_is_safe(&feedback, 0.05f, 0.0f, &estimate));
    CHECK(fabsf(estimate - 0.6f) < 0.000001f);

    feedback.position_rad = RS02_MOTION_TRACKING_GUARD_RAD;
    CHECK(!rs02_motion_feedback_is_safe(&feedback, 0.0f));
    feedback = safe_feedback();
    feedback.velocity_rad_s = RS02_MOTION_SPEED_GUARD_RAD_S;
    CHECK(!rs02_motion_feedback_is_safe(&feedback, 0.0f));
    feedback.velocity_rad_s = 3.39f;
    CHECK(rs02_motion_feedback_is_safe_with_limits(&feedback, 0.0f, 1.0f, 3.4f));
    feedback.velocity_rad_s = 3.4f;
    CHECK(!rs02_motion_feedback_is_safe_with_limits(&feedback, 0.0f, 1.0f, 3.4f));
    CHECK(!rs02_motion_feedback_is_safe_with_limits(&feedback, 0.0f, 1.0f, 33.01f));
    feedback = safe_feedback();
    feedback.torque_nm = RS02_MOTION_TORQUE_GUARD_NM;
    CHECK(!rs02_motion_feedback_is_safe(&feedback, 0.0f));

    feedback = safe_feedback();
    feedback.torque_nm = 6.49f;
    CHECK(rs02_motion_feedback_is_safe_with_torque_limit(&feedback, 0.0f, 6.0f));
    feedback.torque_nm = 6.5f;
    CHECK(!rs02_motion_feedback_is_safe_with_torque_limit(&feedback, 0.0f, 6.0f));
    CHECK(!rs02_motion_feedback_is_safe_with_torque_limit(&feedback, 0.0f, 14.01f));

    feedback = safe_feedback();
    CHECK(!rs02_motion_effort_is_safe(&feedback, 0.09f, 0.0f, &estimate));
    CHECK(estimate > RS02_MOTION_EFFORT_GUARD_NM);
    CHECK(!rs02_motion_effort_is_safe(&feedback, 0.0f, 0.0f, NULL));

    feedback = safe_feedback();
    CHECK(rs02_motion_effort_is_safe_with_limit(&feedback, 0.04f, 0.0f, 0.5f, &estimate));
    CHECK(!rs02_motion_effort_is_safe_with_limit(&feedback, 0.05f, 0.0f, 0.5f, &estimate));
    CHECK(!rs02_motion_effort_is_safe_with_limit(&feedback, 0.01f, 0.0f, 0.0f, &estimate));
    CHECK(rs02_motion_effort_is_safe_with_limit(&feedback, 0.01f, 0.0f, 6.0f, &estimate));
    CHECK(!rs02_motion_effort_is_safe_with_limit(&feedback, 0.01f, 0.0f, 14.01f, &estimate));

    feedback = safe_feedback();
    feedback.position_rad = 0.03f;
    CHECK(rs02_motion_endpoint_reached(&feedback, 0.0f));
    feedback.position_rad = RS02_MOTION_ENDPOINT_ERROR_RAD;
    CHECK(!rs02_motion_endpoint_reached(&feedback, 0.0f));
}

int main(void)
{
    test_zero_effort_profiles();
    test_bus_voltage_guard();
    test_zero_effort_feedback_guards();
    test_motion_endpoint_and_smoothstep();
    test_motion_feedback_and_effort_guards();

    if (s_failures != 0U) {
        fprintf(stderr, "%u control test assertion(s) failed\n", s_failures);
        return EXIT_FAILURE;
    }

    puts("all RS02 control tests passed");
    return EXIT_SUCCESS;
}
