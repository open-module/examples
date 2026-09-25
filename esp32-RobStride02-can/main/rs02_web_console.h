#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "rs02_motion_config.h"
#include "rs02_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RS02_WEB_STATE_IDLE = 0,
    RS02_WEB_STATE_ARMED,
    RS02_WEB_STATE_PREPARING,
    RS02_WEB_STATE_STARTING,
    RS02_WEB_STATE_RUNNING,
    RS02_WEB_STATE_COMPLETE,
    RS02_WEB_STATE_FAULT,
} rs02_web_state_t;

esp_err_t rs02_web_console_start(void);
void rs02_web_console_tick(void);
bool rs02_web_console_take_prepare_request(rs02_motion_config_t *config);
bool rs02_web_console_take_start_request(rs02_motion_config_t *config);
bool rs02_web_console_should_abort(void);

void rs02_web_console_finish_preflight(bool accepted, const char *message);
void rs02_web_console_set_state(rs02_web_state_t state, const char *message);
void rs02_web_console_update_feedback(const rs02_feedback_t *feedback);
void rs02_web_console_update_bus(float bus_voltage_v, float mechanical_position_rad);
void rs02_web_console_update_motion_plan(
    float current_angle_deg,
    float target_angle_deg,
    float signed_delta_deg,
    const rs02_motion_config_t *resolved_config);
void rs02_web_console_update_motion(const char *phase, float target_rad, float progress);

#ifdef __cplusplus
}
#endif
