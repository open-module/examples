#include "cybergear_motion.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned int failures;

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
        failures++; \
    } \
} while (0)

// Catch a stepped command, wrong direction, elapsed-time extrapolation, or skipped endpoint.
static void test_elapsed_targets(void)
{
    const struct { uint32_t elapsed; float expected; } samples[] = {
        {0U, -0.9f}, {20U, -0.8998f}, {137U, -0.89863f},
        {2500U, -0.875f}, {4999U, -0.85001f},
        {5000U, -0.85f}, {6000U, -0.85f}, {UINT32_MAX, -0.85f},
    };
    float target = 0.0f;
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        CHECK(cybergear_motion_target(-0.9f, -0.85f, samples[i].elapsed, 5000U, &target));
        CHECK(fabsf(target - samples[i].expected) < 0.000001f);
    }
    CHECK(cybergear_motion_target(-0.9f, -0.85f, 0U, 5000U, &target));
    CHECK(target == -0.9f);
    CHECK(cybergear_motion_target(-0.9f, -0.85f, 5000U, 5000U, &target));
    CHECK(target == -0.85f);
    CHECK(cybergear_motion_target(-0.85f, -0.9f, 2500U, 5000U, &target));
    CHECK(fabsf(target - -0.875f) < 0.000001f);
    CHECK(cybergear_motion_target(-0.85f, -0.9f, 6000U, 5000U, &target));
    CHECK(target == -0.9f);
    CHECK(cybergear_motion_target(-0.9f, -0.9f, 500U, 500U, &target));
    CHECK(target == -0.9f);
    CHECK(cybergear_motion_target(CYBERGEAR_POSITION_MIN_RAD, CYBERGEAR_POSITION_MAX_RAD,
                                 2500U, 5000U, &target));
    CHECK(target == 0.0f);
}

// Catch missing range checks and accidental outward motion at either protocol boundary.
static void test_endpoint_selection(void)
{
    float endpoint = 0.0f;
    CHECK(cybergear_motion_endpoint(-0.9f, &endpoint));
    CHECK(fabsf(endpoint - 5.38318531f) < 0.000001f);
    CHECK(cybergear_motion_endpoint(CYBERGEAR_POSITION_MIN_RAD, &endpoint));
    CHECK(fabsf(endpoint - -6.28318531f) < 0.000001f);
    CHECK(cybergear_motion_endpoint(CYBERGEAR_POSITION_MAX_RAD, &endpoint));
    CHECK(fabsf(endpoint - 6.28318531f) < 0.000001f);
    CHECK(cybergear_motion_endpoint(12.55f, &endpoint));
    CHECK(fabsf(endpoint - 6.26681469f) < 0.000001f);
    CHECK(cybergear_motion_endpoint(6.2f, &endpoint));
    CHECK(fabsf(endpoint - 12.48318531f) < 0.000001f);
    CHECK(cybergear_motion_endpoint(6.4f, &endpoint));
    CHECK(fabsf(endpoint - 0.11681469f) < 0.000001f);
}

// Catch retaining the half-turn offset or using a duration other than ten seconds.
static void test_full_turn_in_ten_seconds(void)
{
    float endpoint = 0.0f;
    float target = 0.0f;
    CHECK(cybergear_motion_endpoint(0.0f, &endpoint));
    CHECK(fabsf(endpoint - 6.28318531f) < 0.000001f);
    CHECK(cybergear_motion_target(0.0f, endpoint, 20U, CYBERGEAR_MOTION_DURATION_MS, &target));
    CHECK(fabsf(target - 0.012566371f) < 0.00000001f);
    CHECK(cybergear_motion_target(0.0f, endpoint, 5000U, CYBERGEAR_MOTION_DURATION_MS, &target));
    CHECK(fabsf(target - 3.14159265f) < 0.000001f);
    cybergear_feedback_t feedback = {
        .mode = CYBERGEAR_MODE_MOTOR,
        .position_rad = 3.14f,
        .velocity_rad_s = 0.01f,
    };
    CHECK(cybergear_control_feedback_is_safe(&feedback, target,
                                             CYBERGEAR_MOTION_SPEED_GUARD_RAD_S,
                                             1.0f));
    feedback.position_rad = 0.0f;
    CHECK(!cybergear_control_feedback_is_safe(&feedback, target,
                                              CYBERGEAR_MOTION_SPEED_GUARD_RAD_S,
                                              1.0f));
    CHECK(cybergear_motion_target(0.0f, endpoint, 180U, CYBERGEAR_MOTION_DURATION_MS, &target));
    CHECK(!cybergear_control_feedback_is_safe(&feedback, target,
                                              CYBERGEAR_MOTION_SPEED_GUARD_RAD_S,
                                              1.0f));
    CHECK(cybergear_motion_target(0.0f, endpoint, 10000U, CYBERGEAR_MOTION_DURATION_MS, &target));
    CHECK(target == endpoint);
    CHECK(cybergear_motion_target(endpoint, 0.0f, 5000U, CYBERGEAR_MOTION_DURATION_MS, &target));
    CHECK(fabsf(target - 3.14159265f) < 0.000001f);
    CHECK(cybergear_motion_target(endpoint, 0.0f, 10000U, CYBERGEAR_MOTION_DURATION_MS, &target));
    CHECK(target == 0.0f);
}

// Keep the higher-force profile inside the documented rated-load envelope and
// make the velocity term follow, rather than oppose, each trajectory leg.
static void test_rated_force_profile(void)
{
    CHECK(CYBERGEAR_MOTION_KP == 90.0f);
    CHECK(CYBERGEAR_MOTION_KD == 1.0f);
    CHECK(CYBERGEAR_ZERO_EFFORT_TORQUE_GUARD_NM == 1.0f);
    CHECK(CYBERGEAR_MOTION_TORQUE_GUARD_NM == 10.0f);

    float velocity_rad_s = 0.0f;
    CHECK(cybergear_motion_desired_velocity(
        0.0f, CYBERGEAR_MOTION_OFFSET_RAD, CYBERGEAR_MOTION_DURATION_MS,
        &velocity_rad_s));
    CHECK(fabsf(velocity_rad_s - 0.62831853f) < 0.0000001f);
    CHECK(cybergear_motion_desired_velocity(
        CYBERGEAR_MOTION_OFFSET_RAD, 0.0f, CYBERGEAR_MOTION_DURATION_MS,
        &velocity_rad_s));
    CHECK(fabsf(velocity_rad_s - -0.62831853f) < 0.0000001f);
    CHECK(cybergear_motion_desired_velocity(1.0f, 1.0f, 10000U, &velocity_rad_s));
    CHECK(velocity_rad_s == 0.0f);

    CHECK(!cybergear_motion_desired_velocity(0.0f, 1.0f, 0U, &velocity_rad_s));
    CHECK(!cybergear_motion_desired_velocity(0.0f, 1.0f, 10000U, NULL));
    CHECK(!cybergear_motion_desired_velocity(NAN, 1.0f, 10000U, &velocity_rad_s));
    CHECK(!cybergear_motion_desired_velocity(0.0f, INFINITY, 10000U, &velocity_rad_s));
    CHECK(!cybergear_motion_desired_velocity(-13.0f, 0.0f, 10000U, &velocity_rad_s));
    CHECK(!cybergear_motion_desired_velocity(0.0f, 13.0f, 10000U, &velocity_rad_s));
    CHECK(!cybergear_motion_desired_velocity(
        0.0f, CYBERGEAR_MOTION_OFFSET_RAD, 1U, &velocity_rad_s));
}

// The independent tracking and velocity guards can each pass while their
// controller terms add up beyond the intended effort envelope. Estimate the
// complete outgoing command against the latest measured state before sending.
static void test_combined_command_effort_guard(void)
{
    cybergear_feedback_t feedback = {
        .mode = CYBERGEAR_MODE_MOTOR,
        .position_rad = 0.0f,
        .velocity_rad_s = -0.999f,
    };
    float estimated_effort_nm = 0.0f;
    CHECK(!cybergear_motion_command_effort_is_safe(
        &feedback, 0.099f, 0.62831853f, 0.0f,
        CYBERGEAR_MOTION_KP, CYBERGEAR_MOTION_KD,
        CYBERGEAR_MOTION_TORQUE_GUARD_NM, &estimated_effort_nm));
    CHECK(fabsf(estimated_effort_nm - 10.537318f) < 0.00001f);

    feedback.velocity_rad_s = 0.0f;
    CHECK(cybergear_motion_command_effort_is_safe(
        &feedback, 0.09f, 0.62831853f, 0.0f,
        CYBERGEAR_MOTION_KP, CYBERGEAR_MOTION_KD,
        CYBERGEAR_MOTION_TORQUE_GUARD_NM, &estimated_effort_nm));
    CHECK(fabsf(estimated_effort_nm - 8.728318f) < 0.00001f);

    feedback.position_rad = 0.099f;
    feedback.velocity_rad_s = 0.62831853f;
    CHECK(!cybergear_motion_command_effort_is_safe(
        &feedback, 0.0f, -0.62831853f, 0.0f,
        CYBERGEAR_MOTION_KP, CYBERGEAR_MOTION_KD,
        CYBERGEAR_MOTION_TORQUE_GUARD_NM, &estimated_effort_nm));
    CHECK(fabsf(estimated_effort_nm - -10.166637f) < 0.00001f);

    feedback.position_rad = 0.0f;
    feedback.velocity_rad_s = 0.0f;
    CHECK(!cybergear_motion_command_effort_is_safe(
        &feedback, 0.0f, 0.0f, CYBERGEAR_MOTION_TORQUE_GUARD_NM,
        0.0f, 0.0f, CYBERGEAR_MOTION_TORQUE_GUARD_NM, &estimated_effort_nm));
    CHECK(!cybergear_motion_command_effort_is_safe(
        &feedback, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        CYBERGEAR_MOTION_TORQUE_GUARD_NM, NULL));
    CHECK(!cybergear_motion_command_effort_is_safe(
        &feedback, NAN, 0.0f, 0.0f, 0.0f, 0.0f,
        CYBERGEAR_MOTION_TORQUE_GUARD_NM, &estimated_effort_nm));
}

static void test_invalid_targets(void)
{
    const float invalid[] = {NAN, INFINITY, -INFINITY, -13.0f, 13.0f,
        nextafterf(CYBERGEAR_POSITION_MIN_RAD, -INFINITY),
        nextafterf(CYBERGEAR_POSITION_MAX_RAD, INFINITY)};
    float target = 0.0f;
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        CHECK(!cybergear_motion_endpoint(invalid[i], &target));
        CHECK(!cybergear_motion_target(invalid[i], 0.0f, 0U, 5000U, &target));
        CHECK(!cybergear_motion_target(0.0f, invalid[i], 5000U, 5000U, &target));
    }
    CHECK(!cybergear_motion_endpoint(0.0f, NULL));
    CHECK(!cybergear_motion_target(0.0f, 0.05f, 0U, 5000U, NULL));
    CHECK(!cybergear_motion_target(0.0f, 0.05f, 0U, 0U, &target));
}

// Catch non-Motor acceptance, one-sided guards, inclusive limits, and nonfinite bypasses.
static void test_control_feedback_guards(float velocity_limit_rad_s, float torque_limit_nm)
{
    cybergear_feedback_t feedback = {.mode = CYBERGEAR_MODE_MOTOR};
    CHECK(cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                             torque_limit_nm));
    const cybergear_mode_t invalid_modes[] = {
        CYBERGEAR_MODE_RESET, CYBERGEAR_MODE_CALIBRATION, (cybergear_mode_t)3,
    };
    for (size_t i = 0; i < sizeof(invalid_modes) / sizeof(invalid_modes[0]); ++i) {
        feedback.mode = invalid_modes[i];
        CHECK(!cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                                  torque_limit_nm));
    }
    feedback.mode = CYBERGEAR_MODE_MOTOR;
    const float invalid_velocity[] = {velocity_limit_rad_s, -velocity_limit_rad_s,
                                      velocity_limit_rad_s + 0.1f, -velocity_limit_rad_s - 0.1f, 31.0f, -31.0f,
                                      NAN, INFINITY, -INFINITY};
    for (size_t i = 0; i < sizeof(invalid_velocity) / sizeof(invalid_velocity[0]); ++i) {
        feedback.velocity_rad_s = invalid_velocity[i];
        CHECK(!cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                                  torque_limit_nm));
    }
    feedback.velocity_rad_s = nextafterf(velocity_limit_rad_s, 0.0f);
    CHECK(cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                             torque_limit_nm));
    feedback.velocity_rad_s = -nextafterf(velocity_limit_rad_s, 0.0f);
    CHECK(cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                             torque_limit_nm));
    feedback.velocity_rad_s = 0.0f;
    const float invalid_position[] = {0.10f, -0.10f, 0.11f, -0.11f, 13.0f, -13.0f,
                                      NAN, INFINITY, -INFINITY};
    for (size_t i = 0; i < sizeof(invalid_position) / sizeof(invalid_position[0]); ++i) {
        feedback.position_rad = invalid_position[i];
        CHECK(!cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                                  torque_limit_nm));
    }
    const float valid_position[] = {-0.999f, -0.95f, -0.9002f, -0.9f, -0.8998f, -0.85f, -0.801f};
    for (size_t i = 0; i < sizeof(valid_position) / sizeof(valid_position[0]); ++i) {
        feedback.position_rad = valid_position[i];
        CHECK(cybergear_control_feedback_is_safe(&feedback, -0.9f, velocity_limit_rad_s,
                                                 torque_limit_nm));
    }
    feedback.position_rad = CYBERGEAR_POSITION_MIN_RAD;
    CHECK(cybergear_control_feedback_is_safe(&feedback, CYBERGEAR_POSITION_MIN_RAD,
                                             velocity_limit_rad_s, torque_limit_nm));
    feedback.position_rad = CYBERGEAR_POSITION_MAX_RAD;
    CHECK(cybergear_control_feedback_is_safe(&feedback, CYBERGEAR_POSITION_MAX_RAD,
                                             velocity_limit_rad_s, torque_limit_nm));
    // Isolate protocol range rejection from the displacement guard.
    feedback.position_rad = nextafterf(CYBERGEAR_POSITION_MAX_RAD, INFINITY);
    CHECK(!cybergear_control_feedback_is_safe(&feedback, CYBERGEAR_POSITION_MAX_RAD,
                                              velocity_limit_rad_s, torque_limit_nm));
    feedback.position_rad = nextafterf(CYBERGEAR_POSITION_MIN_RAD, -INFINITY);
    CHECK(!cybergear_control_feedback_is_safe(&feedback, CYBERGEAR_POSITION_MIN_RAD,
                                              velocity_limit_rad_s, torque_limit_nm));
    feedback.position_rad = CYBERGEAR_POSITION_MAX_RAD;
    CHECK(!cybergear_control_feedback_is_safe(&feedback,
                                             nextafterf(CYBERGEAR_POSITION_MAX_RAD, INFINITY),
                                             velocity_limit_rad_s, torque_limit_nm));
    feedback.position_rad = CYBERGEAR_POSITION_MIN_RAD;
    CHECK(!cybergear_control_feedback_is_safe(&feedback,
                                             nextafterf(CYBERGEAR_POSITION_MIN_RAD, -INFINITY),
                                             velocity_limit_rad_s, torque_limit_nm));
    feedback.position_rad = 0.0f;
    const float invalid_origin[] = {NAN, INFINITY, -INFINITY, 13.0f, -13.0f};
    for (size_t i = 0; i < sizeof(invalid_origin) / sizeof(invalid_origin[0]); ++i) {
        CHECK(!cybergear_control_feedback_is_safe(&feedback, invalid_origin[i],
                                                  velocity_limit_rad_s, torque_limit_nm));
    }
    CHECK(!cybergear_control_feedback_is_safe(NULL, 0.0f, velocity_limit_rad_s,
                                              torque_limit_nm));

    const float invalid_torque[] = {torque_limit_nm, -torque_limit_nm,
                                    torque_limit_nm + 0.1f, -torque_limit_nm - 0.1f,
                                    NAN, INFINITY, -INFINITY};
    for (size_t i = 0; i < sizeof(invalid_torque) / sizeof(invalid_torque[0]); ++i) {
        feedback.torque_nm = invalid_torque[i];
        CHECK(!cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                                  torque_limit_nm));
    }
    feedback.torque_nm = nextafterf(torque_limit_nm, 0.0f);
    CHECK(cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                             torque_limit_nm));
    feedback.torque_nm = -nextafterf(torque_limit_nm, 0.0f);
    CHECK(cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                             torque_limit_nm));
    feedback.torque_nm = 0.0f;

    const float invalid_torque_limit[] = {0.0f, -1.0f, NAN, INFINITY, -INFINITY,
                                          nextafterf(CYBERGEAR_TORQUE_MAX_NM, INFINITY)};
    for (size_t i = 0; i < sizeof(invalid_torque_limit) / sizeof(invalid_torque_limit[0]); ++i) {
        CHECK(!cybergear_control_feedback_is_safe(&feedback, 0.0f, velocity_limit_rad_s,
                                                  invalid_torque_limit[i]));
    }
}

// The faster opt-in profile must not relax the default zero-effort check.
static void test_phase_speed_guards(void)
{
    CHECK(CYBERGEAR_ZERO_EFFORT_SPEED_GUARD_RAD_S == 0.5f);
    CHECK(CYBERGEAR_MOTION_SPEED_GUARD_RAD_S == 1.0f);
    cybergear_feedback_t feedback = {.mode = CYBERGEAR_MODE_MOTOR};
    const float moving_velocity[] = {0.62831853f, -0.62831853f};
    for (size_t i = 0; i < sizeof(moving_velocity) / sizeof(moving_velocity[0]); ++i) {
        feedback.velocity_rad_s = moving_velocity[i];
        CHECK(cybergear_control_feedback_is_safe(&feedback, 0.0f,
                                                 CYBERGEAR_MOTION_SPEED_GUARD_RAD_S,
                                                 1.0f));
        CHECK(!cybergear_control_feedback_is_safe(&feedback, 0.0f,
                                                  CYBERGEAR_ZERO_EFFORT_SPEED_GUARD_RAD_S,
                                                  1.0f));
    }
    feedback.velocity_rad_s = 0.0f;
    const float invalid_limit[] = {0.0f, -0.5f, NAN, INFINITY, -INFINITY, 31.0f};
    for (size_t i = 0; i < sizeof(invalid_limit) / sizeof(invalid_limit[0]); ++i) {
        CHECK(!cybergear_control_feedback_is_safe(&feedback, 0.0f, invalid_limit[i],
                                                  1.0f));
    }
}

// Catch a diagnostic build that changes effort, target, or guards instead of only extending traffic time.
static void test_zero_effort_profiles(void)
{
    cybergear_zero_effort_profile_t profile = {0};
    CHECK(cybergear_zero_effort_profile(false, &profile));
    CHECK(profile.duration_ms == 500U);
    CHECK(profile.target_offset_rad == 0.0f);
    CHECK(profile.desired_velocity_rad_s == 0.0f);
    CHECK(profile.feedforward_torque_nm == 0.0f);
    CHECK(profile.kp == 0.0f);
    CHECK(profile.kd == 0.0f);
    CHECK(profile.speed_guard_rad_s == 0.5f);

    CHECK(cybergear_zero_effort_profile(true, &profile));
    CHECK(profile.duration_ms == 60000U);
    CHECK(profile.target_offset_rad == 0.0f);
    CHECK(profile.desired_velocity_rad_s == 0.0f);
    CHECK(profile.feedforward_torque_nm == 0.0f);
    CHECK(profile.kp == 0.0f);
    CHECK(profile.kd == 0.0f);
    CHECK(profile.speed_guard_rad_s == 0.5f);
    CHECK(!cybergear_zero_effort_profile(false, NULL));
}

int main(void)
{
    test_elapsed_targets();
    test_endpoint_selection();
    test_full_turn_in_ten_seconds();
    test_rated_force_profile();
    test_combined_command_effort_guard();
    test_invalid_targets();
    test_control_feedback_guards(CYBERGEAR_ZERO_EFFORT_SPEED_GUARD_RAD_S,
                                 1.0f);
    test_control_feedback_guards(CYBERGEAR_MOTION_SPEED_GUARD_RAD_S,
                                 1.0f);
    test_phase_speed_guards();
    test_zero_effort_profiles();
    if (failures != 0U) {
        fprintf(stderr, "%u motion checks failed\n", failures);
        return EXIT_FAILURE;
    }
    puts("All CyberGear motion tests passed");
    return EXIT_SUCCESS;
}
