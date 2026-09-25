#include "rs02_motion_config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define PI_F 3.14159265358979323846f

static unsigned int s_failures;

#define CHECK(condition)                                                                    \
    do {                                                                                    \
        if (!(condition)) {                                                                 \
            fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #condition); \
            s_failures++;                                                                   \
        }                                                                                   \
    } while (0)

static rs02_motion_config_t absolute_config(float target_deg)
{
    rs02_motion_config_t config = rs02_motion_config_default();
    config.angle_deg = target_deg;
    config.planned_delta_deg = RS02_WEB_MAX_SINGLE_TURN_ANGLE_DEG;
    config.speed_limit_rad_s = 3.4f;
    config.absolute_single_turn_target = true;
    config.return_to_origin = false;
    return config;
}

static void test_default_config(void)
{
    rs02_motion_config_t config = rs02_motion_config_default();
    float peak_speed = 0.0f;
    rs02_motion_config_error_t error = RS02_MOTION_CONFIG_NONFINITE;

    CHECK(config.angle_deg == 60.0f);
    CHECK(config.planned_delta_deg == 60.0f);
    CHECK(config.duration_ms == 3000U);
    CHECK(config.torque_limit_nm == 1.0f);
    CHECK(config.speed_limit_rad_s == 0.8f);
    CHECK(!config.absolute_single_turn_target);
    CHECK(config.return_to_origin);
    CHECK(rs02_motion_config_validate(&config, &peak_speed, &error));
    CHECK(error == RS02_MOTION_CONFIG_OK);
    CHECK(fabsf(peak_speed - 0.5235988f) < 0.000001f);

    float endpoint = 0.0f;
    CHECK(rs02_motion_config_resolve_endpoint(
        1.0f, 0.0f, &config, NULL, &endpoint, &error));
    CHECK(fabsf(endpoint - 2.0471976f) < 0.000001f);
}

static void test_validation_boundaries(void)
{
    rs02_motion_config_t config = rs02_motion_config_default();
    rs02_motion_config_error_t error = RS02_MOTION_CONFIG_OK;

    config.angle_deg = -60.0f;
    config.planned_delta_deg = -60.0f;
    CHECK(rs02_motion_config_validate(&config, NULL, &error));

    config.angle_deg = 0.0f;
    config.planned_delta_deg = 0.0f;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_ANGLE_RANGE);

    config = rs02_motion_config_default();
    config.angle_deg = NAN;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_NONFINITE);

    config = rs02_motion_config_default();
    config.duration_ms = 299U;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_DURATION_RANGE);
    config.duration_ms = 300U;
    config.angle_deg = 1.0f;
    config.planned_delta_deg = 1.0f;
    config.speed_limit_rad_s = 1.0f;
    CHECK(rs02_motion_config_validate(&config, NULL, &error));
    config.duration_ms = 30000U;
    CHECK(rs02_motion_config_validate(&config, NULL, &error));
    config.duration_ms = 30001U;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_DURATION_RANGE);

    config = rs02_motion_config_default();
    config.torque_limit_nm = 0.09f;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_TORQUE_RANGE);
    config.torque_limit_nm = 14.0f;
    CHECK(rs02_motion_config_validate(&config, NULL, &error));
    config.torque_limit_nm = 14.01f;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_TORQUE_RANGE);

    config = rs02_motion_config_default();
    config.speed_limit_rad_s = 0.29f;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_SPEED_RANGE);
    config.speed_limit_rad_s = 33.0f;
    CHECK(rs02_motion_config_validate(&config, NULL, &error));
    config.speed_limit_rad_s = 33.01f;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_SPEED_RANGE);

    config = rs02_motion_config_default();
    config.angle_deg = 180.0f;
    config.planned_delta_deg = 180.0f;
    config.duration_ms = 3000U;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_PROFILE_SPEED);
    config.duration_ms = 9000U;
    CHECK(rs02_motion_config_validate(&config, NULL, &error));

    config.angle_deg = RS02_RELATIVE_MAX_ABS_ANGLE_DEG + 0.1f;
    config.planned_delta_deg = config.angle_deg;
    config.duration_ms = 70000U;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_ANGLE_RANGE);
}

static void test_relative_endpoint_direction_and_range(void)
{
    rs02_motion_config_t config = rs02_motion_config_default();
    rs02_motion_config_error_t error = RS02_MOTION_CONFIG_OK;
    float endpoint = 0.0f;

    config.angle_deg = -60.0f;
    config.planned_delta_deg = -60.0f;
    CHECK(rs02_motion_config_resolve_endpoint(
        1.0f, 0.0f, &config, NULL, &endpoint, &error));
    CHECK(fabsf(endpoint - (-0.0471976f)) < 0.000001f);

    config.angle_deg = 180.0f;
    config.planned_delta_deg = 180.0f;
    config.duration_ms = 9000U;
    CHECK(!rs02_motion_config_resolve_endpoint(
        12.0f, 0.0f, &config, NULL, &endpoint, &error));
    CHECK(error == RS02_MOTION_CONFIG_ENDPOINT_RANGE);
    CHECK(!rs02_motion_config_resolve_endpoint(
        NAN, 0.0f, &config, NULL, &endpoint, &error));
    CHECK(error == RS02_MOTION_CONFIG_ENDPOINT_RANGE);
    CHECK(!rs02_motion_config_resolve_endpoint(
        0.0f, NAN, &config, NULL, &endpoint, &error));
    CHECK(error == RS02_MOTION_CONFIG_ENDPOINT_RANGE);
}

static void test_absolute_single_turn_targets(void)
{
    rs02_motion_config_error_t error = RS02_MOTION_CONFIG_OK;
    float current_deg = 0.0f;
    float endpoint = 0.0f;

    rs02_motion_config_t config = absolute_config(90.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        45.0f * PI_F / 180.0f,
        45.0f * PI_F / 180.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(current_deg - 45.0f) < 0.001f);
    CHECK(fabsf(config.planned_delta_deg - 45.0f) < 0.001f);
    CHECK(fabsf(endpoint - (90.0f * PI_F / 180.0f)) < 0.001f);
    CHECK(fabsf(config.speed_limit_rad_s - 0.652699f) < 0.001f);

    config = absolute_config(45.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        90.0f * PI_F / 180.0f,
        90.0f * PI_F / 180.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(config.planned_delta_deg - (-45.0f)) < 0.001f);
    CHECK(fabsf(endpoint - (45.0f * PI_F / 180.0f)) < 0.001f);

    config = absolute_config(0.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        45.0f * PI_F / 180.0f,
        45.0f * PI_F / 180.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(config.planned_delta_deg - (-45.0f)) < 0.001f);
    CHECK(fabsf(endpoint) < 0.001f);

    config = absolute_config(90.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        405.0f * PI_F / 180.0f,
        405.0f * PI_F / 180.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(current_deg - 45.0f) < 0.001f);
    CHECK(fabsf(config.planned_delta_deg - 45.0f) < 0.001f);
    CHECK(fabsf(endpoint - (450.0f * PI_F / 180.0f)) < 0.001f);

    config = absolute_config(90.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        1.0f,
        45.0f * PI_F / 180.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(config.planned_delta_deg - 45.0f) < 0.001f);
    CHECK(fabsf(endpoint - (1.0f + 45.0f * PI_F / 180.0f)) < 0.001f);

    config = absolute_config(10.0f);
    config.duration_ms = 30000U;
    CHECK(rs02_motion_config_resolve_endpoint(
        350.0f * PI_F / 180.0f,
        350.0f * PI_F / 180.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(config.planned_delta_deg - (-340.0f)) < 0.001f);
    CHECK(fabsf(endpoint - (10.0f * PI_F / 180.0f)) < 0.001f);

    config = absolute_config(360.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        45.0f * PI_F / 180.0f,
        45.0f * PI_F / 180.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(config.planned_delta_deg - 315.0f) < 0.001f);
    CHECK(fabsf(endpoint - (360.0f * PI_F / 180.0f)) < 0.001f);

    config = absolute_config(360.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        2.0f * PI_F,
        2.0f * PI_F,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(current_deg - 360.0f) < 0.001f);
    CHECK(fabsf(config.planned_delta_deg) < 0.001f);
    CHECK(fabsf(endpoint - (2.0f * PI_F)) < 0.001f);

    config = absolute_config(0.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        2.0f * PI_F,
        2.0f * PI_F,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(current_deg - 360.0f) < 0.001f);
    CHECK(fabsf(config.planned_delta_deg - (-360.0f)) < 0.001f);
    CHECK(fabsf(endpoint) < 0.001f);

    config = absolute_config(360.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        0.0f,
        0.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(current_deg) < 0.001f);
    CHECK(fabsf(config.planned_delta_deg - 360.0f) < 0.001f);
    CHECK(fabsf(endpoint - (2.0f * PI_F)) < 0.001f);

    config = absolute_config(0.0f);
    CHECK(rs02_motion_config_resolve_endpoint(
        0.0f,
        0.0f,
        &config,
        &current_deg,
        &endpoint,
        &error));
    CHECK(fabsf(current_deg) < 0.001f);
    CHECK(fabsf(config.planned_delta_deg) < 0.001f);
    CHECK(fabsf(endpoint) < 0.001f);
}

static void test_absolute_target_boundaries(void)
{
    rs02_motion_config_error_t error = RS02_MOTION_CONFIG_OK;
    rs02_motion_config_t config = absolute_config(360.0f);

    CHECK(rs02_motion_config_validate(&config, NULL, &error));
    config.angle_deg = 360.1f;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_ANGLE_RANGE);
    config.angle_deg = -0.1f;
    CHECK(!rs02_motion_config_validate(&config, NULL, &error));
    CHECK(error == RS02_MOTION_CONFIG_ANGLE_RANGE);

    config = absolute_config(360.0f);
    config.duration_ms = 300U;
    config.speed_limit_rad_s = 31.676f;
    CHECK(rs02_motion_config_validate(&config, NULL, &error));
}

int main(void)
{
    test_default_config();
    test_validation_boundaries();
    test_relative_endpoint_direction_and_range();
    test_absolute_single_turn_targets();
    test_absolute_target_boundaries();

    if (s_failures != 0U) {
        fprintf(stderr, "%u motion-config test assertion(s) failed\n", s_failures);
        return EXIT_FAILURE;
    }

    puts("all RS02 motion-config tests passed");
    return EXIT_SUCCESS;
}
