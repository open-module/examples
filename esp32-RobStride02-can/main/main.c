#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rs02_control.h"
#include "rs02_protocol.h"
#include "rs02_twai.h"

#define TAG "robstride_rs02"

#define DEVICE_RESPONSE_TIMEOUT_MS 1500U
#define PARAMETER_RESPONSE_TIMEOUT_MS 1500U
#define STAGE_FEEDBACK_TIMEOUT_MS 1500U
#define CONTROL_FEEDBACK_TIMEOUT_MS 250U
#define CONTROL_PERIOD_MS 20U
#define CONTROL_LOG_PERIOD_MS 250U
#define PARAMETER_WRITE_SETTLE_MS 10U

#if CONFIG_RS02_RUN_LINK_SOAK_TEST
#define LINK_SOAK_TEST_STATUS "ENABLED"
#define LINK_SOAK_TEST_ENABLED true
#else
#define LINK_SOAK_TEST_STATUS "disabled"
#define LINK_SOAK_TEST_ENABLED false
#endif

#if CONFIG_RS02_RUN_MOTION_TEST
#define MOTION_TEST_STATUS "ENABLED"
#else
#define MOTION_TEST_STATUS "disabled"
#endif

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_UNKNOWN: return "unknown";
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_EXT: return "external-pin";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt-watchdog";
    case ESP_RST_TASK_WDT: return "task-watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO: return "sdio";
    case ESP_RST_USB: return "usb";
    case ESP_RST_JTAG: return "jtag";
    case ESP_RST_EFUSE: return "efuse";
    case ESP_RST_PWR_GLITCH: return "power-glitch";
    case ESP_RST_CPU_LOCKUP: return "cpu-lockup";
    default: return "unrecognized";
    }
}

static uint32_t monotonic_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void log_bus_status(void)
{
    twai_status_info_t status = {0};
    const esp_err_t err = rs02_twai_get_status(&status);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot read TWAI status: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(
        TAG,
        "TWAI state=%d txq=%" PRIu32 " rxq=%" PRIu32 " TEC=%" PRIu32
        " REC=%" PRIu32 " tx_failed=%" PRIu32 " missed=%" PRIu32
        " bus_errors=%" PRIu32,
        status.state,
        status.msgs_to_tx,
        status.msgs_to_rx,
        status.tx_error_counter,
        status.rx_error_counter,
        status.tx_failed_count,
        status.rx_missed_count,
        status.bus_error_count);
}

static esp_err_t check_bus_alerts(void)
{
    uint32_t alerts = 0;
    const esp_err_t err = rs02_twai_read_alerts(&alerts, 0);
    if (err == ESP_ERR_TIMEOUT) {
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    const uint32_t fatal_alerts = TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL |
                                  TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR |
                                  TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_FIFO_OVERRUN;
    if ((alerts & fatal_alerts) != 0U) {
        ESP_LOGE(TAG, "fatal TWAI alert mask=0x%08" PRIx32, alerts & fatal_alerts);
        log_bus_status();
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t transmit_frame(const rs02_frame_t *frame, const char *label)
{
    ESP_RETURN_ON_ERROR(check_bus_alerts(), TAG, "TWAI alert before transmit");
    const esp_err_t err = rs02_twai_transmit(frame, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s transmit failed: %s", label, esp_err_to_name(err));
        log_bus_status();
        return err;
    }

    ESP_LOGD(TAG, "%s TX id=0x%08" PRIx32, label, frame->identifier);
    return ESP_OK;
}

static esp_err_t reject_fault_feedback(const rs02_frame_t *frame)
{
    rs02_fault_feedback_t fault = {0};
    if (!rs02_parse_fault_feedback(
            frame,
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            &fault)) {
        return ESP_OK;
    }
    if (fault.faults == 0U && fault.warnings == 0U) {
        return ESP_OK;
    }

    ESP_LOGE(
        TAG,
        "motor fault frame: fault=0x%08" PRIx32 " warning=0x%08" PRIx32,
        fault.faults,
        fault.warnings);
    return ESP_FAIL;
}

static bool base_feedback_is_safe(const rs02_feedback_t *feedback)
{
    if (feedback->faults != 0U) {
        ESP_LOGE(TAG, "motor feedback fault flags=0x%02x", feedback->faults);
        return false;
    }
    if (!isfinite(feedback->temperature_c) || feedback->temperature_c < -20.0f ||
        feedback->temperature_c >= RS02_SOFTWARE_TEMPERATURE_GUARD_C) {
        ESP_LOGE(
            TAG,
            "motor temperature %.1f C is outside the provisional [-20, %.1f) C guard",
            (double)feedback->temperature_c,
            (double)RS02_SOFTWARE_TEMPERATURE_GUARD_C);
        return false;
    }
    return true;
}

static void log_feedback_sample(const char *prefix, const rs02_feedback_t *feedback)
{
    ESP_LOGI(
        TAG,
        "%s motor=0x%02x mode=%u pos=%.4f rad vel=%.3f rad/s torque=%.3f N m temp=%.1f C",
        prefix,
        feedback->motor_id,
        (unsigned int)feedback->mode,
        (double)feedback->position_rad,
        (double)feedback->velocity_rad_s,
        (double)feedback->torque_nm,
        (double)feedback->temperature_c);
}

static esp_err_t wait_for_device_id(rs02_device_info_t *device)
{
    const uint32_t start_ms = monotonic_ms();
    while ((uint32_t)(monotonic_ms() - start_ms) < DEVICE_RESPONSE_TIMEOUT_MS) {
        ESP_RETURN_ON_ERROR(check_bus_alerts(), TAG, "TWAI alert while waiting for device ID");

        rs02_frame_t frame = {0};
        const esp_err_t err = rs02_twai_receive(&frame, pdMS_TO_TICKS(20));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_RETURN_ON_ERROR(err, TAG, "invalid CAN frame while waiting for device ID");
        ESP_RETURN_ON_ERROR(
            reject_fault_feedback(&frame), TAG, "motor fault while waiting for device ID");

        if (rs02_parse_device_id(&frame, (uint8_t)CONFIG_RS02_MOTOR_ID, device)) {
            return ESP_OK;
        }
        ESP_LOGD(TAG, "ignored frame id=0x%08" PRIx32 " while waiting for device ID",
                 frame.identifier);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t wait_for_feedback(uint32_t timeout_ms, bool log_sample, rs02_feedback_t *feedback)
{
    const uint32_t start_ms = monotonic_ms();
    while ((uint32_t)(monotonic_ms() - start_ms) < timeout_ms) {
        ESP_RETURN_ON_ERROR(check_bus_alerts(), TAG, "TWAI alert while waiting for feedback");

        rs02_frame_t frame = {0};
        const esp_err_t err = rs02_twai_receive(&frame, pdMS_TO_TICKS(20));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_RETURN_ON_ERROR(err, TAG, "invalid CAN frame while waiting for feedback");
        ESP_RETURN_ON_ERROR(reject_fault_feedback(&frame), TAG, "motor fault feedback received");

        if (!rs02_parse_feedback(
                &frame,
                (uint8_t)CONFIG_RS02_MOTOR_ID,
                (uint8_t)CONFIG_RS02_HOST_ID,
                feedback)) {
            ESP_LOGD(TAG, "ignored frame id=0x%08" PRIx32 " while waiting for feedback",
                     frame.identifier);
            continue;
        }
        if (!base_feedback_is_safe(feedback)) {
            return ESP_FAIL;
        }
        if (log_sample) {
            log_feedback_sample("feedback", feedback);
        }
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t wait_for_parameter_float(uint16_t parameter_index, float *value)
{
    const uint32_t start_ms = monotonic_ms();
    while ((uint32_t)(monotonic_ms() - start_ms) < PARAMETER_RESPONSE_TIMEOUT_MS) {
        ESP_RETURN_ON_ERROR(check_bus_alerts(), TAG, "TWAI alert while waiting for parameter");

        rs02_frame_t frame = {0};
        const esp_err_t err = rs02_twai_receive(&frame, pdMS_TO_TICKS(20));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_RETURN_ON_ERROR(err, TAG, "invalid CAN frame while waiting for parameter");
        ESP_RETURN_ON_ERROR(reject_fault_feedback(&frame), TAG, "motor fault while reading parameter");

        if (rs02_parse_parameter_float(
                &frame,
                (uint8_t)CONFIG_RS02_MOTOR_ID,
                (uint8_t)CONFIG_RS02_HOST_ID,
                parameter_index,
                value)) {
            return ESP_OK;
        }
        ESP_LOGD(TAG, "ignored frame id=0x%08" PRIx32 " while waiting for parameter 0x%04x",
                 frame.identifier, parameter_index);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t wait_for_parameter_u8(uint16_t parameter_index, uint8_t *value)
{
    const uint32_t start_ms = monotonic_ms();
    while ((uint32_t)(monotonic_ms() - start_ms) < PARAMETER_RESPONSE_TIMEOUT_MS) {
        ESP_RETURN_ON_ERROR(check_bus_alerts(), TAG, "TWAI alert while waiting for parameter");

        rs02_frame_t frame = {0};
        const esp_err_t err = rs02_twai_receive(&frame, pdMS_TO_TICKS(20));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_RETURN_ON_ERROR(err, TAG, "invalid CAN frame while waiting for parameter");
        ESP_RETURN_ON_ERROR(reject_fault_feedback(&frame), TAG, "motor fault while reading parameter");

        if (rs02_parse_parameter_u8(
                &frame,
                (uint8_t)CONFIG_RS02_MOTOR_ID,
                (uint8_t)CONFIG_RS02_HOST_ID,
                parameter_index,
                value)) {
            return ESP_OK;
        }
        ESP_LOGD(TAG, "ignored frame id=0x%08" PRIx32 " while waiting for parameter 0x%04x",
                 frame.identifier, parameter_index);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t read_parameter_float(uint16_t parameter_index, const char *label, float *value)
{
    rs02_frame_t frame = {0};
    if (!rs02_make_parameter_read(
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            parameter_index,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(transmit_frame(&frame, label), TAG, "parameter request failed");
    return wait_for_parameter_float(parameter_index, value);
}

static esp_err_t read_parameter_u8(uint16_t parameter_index, const char *label, uint8_t *value)
{
    rs02_frame_t frame = {0};
    if (!rs02_make_parameter_read(
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            parameter_index,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(transmit_frame(&frame, label), TAG, "parameter request failed");
    return wait_for_parameter_u8(parameter_index, value);
}

static esp_err_t select_operation_control_mode(void)
{
    rs02_frame_t frame = {0};
    if (!rs02_make_parameter_write_u8(
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            RS02_PARAMETER_RUN_MODE,
            0U,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(
        transmit_frame(&frame, "operation-control mode write"),
        TAG,
        "run_mode write failed");
    vTaskDelay(pdMS_TO_TICKS(PARAMETER_WRITE_SETTLE_MS));

    uint8_t run_mode = UINT8_MAX;
    ESP_RETURN_ON_ERROR(
        read_parameter_u8(RS02_PARAMETER_RUN_MODE, "run-mode readback", &run_mode),
        TAG,
        "run_mode readback failed");
    if (run_mode != 0U) {
        ESP_LOGE(TAG, "operation-control mode not confirmed: run_mode=%u", run_mode);
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "operation-control mode confirmed: run_mode=0");
    return ESP_OK;
}

#if CONFIG_RS02_RUN_MOTION_TEST
static esp_err_t configure_motion_torque_limit(void)
{
    rs02_frame_t frame = {0};
    if (!rs02_make_parameter_write_float(
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            RS02_PARAMETER_TORQUE_LIMIT,
            RS02_MOTION_MOTOR_TORQUE_LIMIT_NM,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(
        transmit_frame(&frame, "motion torque-limit write"),
        TAG,
        "torque-limit write failed");
    vTaskDelay(pdMS_TO_TICKS(PARAMETER_WRITE_SETTLE_MS));

    float torque_limit_nm = 0.0f;
    ESP_RETURN_ON_ERROR(
        read_parameter_float(
            RS02_PARAMETER_TORQUE_LIMIT,
            "motion torque-limit readback",
            &torque_limit_nm),
        TAG,
        "torque-limit readback failed");
    if (!isfinite(torque_limit_nm) ||
        fabsf(torque_limit_nm - RS02_MOTION_MOTOR_TORQUE_LIMIT_NM) > 0.001f) {
        ESP_LOGE(TAG, "motion torque limit not confirmed: %.3f N m", (double)torque_limit_nm);
        return ESP_ERR_INVALID_STATE;
    }
    ESP_LOGI(TAG, "temporary motion torque limit confirmed: %.3f N m",
             (double)torque_limit_nm);
    return ESP_OK;
}
#endif

static esp_err_t send_stop(bool clear_fault)
{
    rs02_frame_t frame = {0};
    if (!rs02_make_stop(
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            clear_fault,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    return transmit_frame(&frame, clear_fault ? "clear-fault stop" : "stop");
}

static esp_err_t send_stop_best_effort(void)
{
    rs02_frame_t frame = {0};
    if (!rs02_make_stop(
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            false,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < 32U; ++i) {
        rs02_frame_t stale_frame = {0};
        const esp_err_t receive_error = rs02_twai_receive(&stale_frame, 0);
        if (receive_error == ESP_ERR_TIMEOUT) {
            break;
        }
        if (receive_error != ESP_OK) {
            ESP_LOGW(TAG, "could not drain TWAI RX queue before stop: %s",
                     esp_err_to_name(receive_error));
            break;
        }
    }

    // Attempt stop even when a previously latched TWAI alert would reject normal traffic.
    ESP_RETURN_ON_ERROR(
        rs02_twai_transmit(&frame, pdMS_TO_TICKS(100)),
        TAG,
        "best-effort stop transmit failed");

    const uint32_t start_ms = monotonic_ms();
    while ((uint32_t)(monotonic_ms() - start_ms) < CONTROL_FEEDBACK_TIMEOUT_MS) {
        rs02_frame_t response = {0};
        const esp_err_t receive_error = rs02_twai_receive(&response, pdMS_TO_TICKS(20));
        if (receive_error == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_RETURN_ON_ERROR(receive_error, TAG, "stop confirmation receive failed");

        rs02_feedback_t feedback = {0};
        if (rs02_parse_feedback(
                &response,
                (uint8_t)CONFIG_RS02_MOTOR_ID,
                (uint8_t)CONFIG_RS02_HOST_ID,
                &feedback) && feedback.mode == RS02_MODE_RESET) {
            ESP_LOGI(TAG, "stop feedback received before TWAI shutdown; faults=0x%02x",
                     feedback.faults);
            return ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t run_zero_effort_phase(rs02_feedback_t *feedback)
{
    rs02_zero_effort_profile_t profile = {0};
    if (!rs02_zero_effort_profile(LINK_SOAK_TEST_ENABLED, &profile)) {
        return ESP_ERR_INVALID_STATE;
    }

    const float initial_position_rad = feedback->position_rad;
    ESP_LOGI(TAG, "zero-effort phase at measured position %.4f rad for %" PRIu32 " ms",
             (double)initial_position_rad, profile.duration_ms);

    const uint32_t start_ms = monotonic_ms();
    uint32_t elapsed_ms = 0U;
    uint32_t next_log_ms = 0U;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        if (!rs02_zero_effort_feedback_is_safe(feedback, initial_position_rad)) {
            ESP_LOGE(
                TAG,
                "unsafe zero-effort feedback: mode=%u faults=0x%02x pos=%.4f rad "
                "vel=%.3f rad/s torque=%.3f N m temp=%.1f C origin=%.4f rad",
                (unsigned int)feedback->mode,
                feedback->faults,
                (double)feedback->position_rad,
                (double)feedback->velocity_rad_s,
                (double)feedback->torque_nm,
                (double)feedback->temperature_c,
                (double)initial_position_rad);
            return ESP_ERR_INVALID_STATE;
        }

        rs02_frame_t frame = {0};
        if (!rs02_make_operation_control(
                (uint8_t)CONFIG_RS02_MOTOR_ID,
                initial_position_rad,
                profile.desired_velocity_rad_s,
                profile.feedforward_torque_nm,
                profile.kp,
                profile.kd,
                &frame)) {
            return ESP_ERR_INVALID_ARG;
        }
        ESP_RETURN_ON_ERROR(
            transmit_frame(&frame, "zero-effort operation control"),
            TAG,
            "zero-effort transmit failed");
        ESP_RETURN_ON_ERROR(
            wait_for_feedback(CONTROL_FEEDBACK_TIMEOUT_MS, false, feedback),
            TAG,
            "zero-effort feedback timeout or fault");
        if (!rs02_zero_effort_feedback_is_safe(feedback, initial_position_rad)) {
            ESP_LOGE(
                TAG,
                "unsafe post-command feedback: mode=%u faults=0x%02x pos=%.4f rad "
                "vel=%.3f rad/s torque=%.3f N m temp=%.1f C origin=%.4f rad",
                (unsigned int)feedback->mode,
                feedback->faults,
                (double)feedback->position_rad,
                (double)feedback->velocity_rad_s,
                (double)feedback->torque_nm,
                (double)feedback->temperature_c,
                (double)initial_position_rad);
            return ESP_ERR_INVALID_STATE;
        }

        if (elapsed_ms >= next_log_ms || elapsed_ms >= profile.duration_ms) {
            log_feedback_sample("zero-effort feedback", feedback);
            if (next_log_ms <= UINT32_MAX - CONTROL_LOG_PERIOD_MS) {
                next_log_ms += CONTROL_LOG_PERIOD_MS;
            }
        }
        if (elapsed_ms >= profile.duration_ms) {
            return ESP_OK;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        elapsed_ms = (uint32_t)(monotonic_ms() - start_ms);
    }
}

#if CONFIG_RS02_RUN_MOTION_TEST
static esp_err_t check_motion_feedback(
    const rs02_feedback_t *feedback,
    float target_rad,
    const char *phase_label,
    const char *timing_label)
{
    if (rs02_motion_feedback_is_safe(feedback, target_rad)) {
        return ESP_OK;
    }

    ESP_LOGE(
        TAG,
        "%s unsafe %s feedback: mode=%u faults=0x%02x pos=%.4f rad target=%.4f rad "
        "error=%.4f rad vel=%.3f rad/s torque=%.3f N m temp=%.1f C",
        phase_label,
        timing_label,
        (unsigned int)feedback->mode,
        feedback->faults,
        (double)feedback->position_rad,
        (double)target_rad,
        (double)(feedback->position_rad - target_rad),
        (double)feedback->velocity_rad_s,
        (double)feedback->torque_nm,
        (double)feedback->temperature_c);
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t run_motion_phase(
    const char *label,
    float start_rad,
    float end_rad,
    uint32_t duration_ms,
    rs02_feedback_t *feedback)
{
    ESP_LOGI(
        TAG,
        "%s: start=%.4f rad end=%.4f rad duration=%" PRIu32 " ms",
        label,
        (double)start_rad,
        (double)end_rad,
        duration_ms);

    const uint32_t start_ms = monotonic_ms();
    uint32_t elapsed_ms = 0U;
    uint32_t next_log_ms = 0U;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        float target_rad = 0.0f;
        float desired_velocity_rad_s = 0.0f;
        if (!rs02_motion_sample(
                start_rad,
                end_rad,
                elapsed_ms,
                duration_ms,
                &target_rad,
                &desired_velocity_rad_s)) {
            return ESP_ERR_INVALID_ARG;
        }

        ESP_RETURN_ON_ERROR(
            check_motion_feedback(feedback, target_rad, label, "pre-command"),
            TAG,
            "unsafe motion feedback before transmit");

        float estimated_effort_nm = 0.0f;
        if (!rs02_motion_effort_is_safe(
                feedback,
                target_rad,
                desired_velocity_rad_s,
                &estimated_effort_nm)) {
            ESP_LOGE(
                TAG,
                "%s unsafe estimated effort %.3f N m: target=%.4f rad measured=%.4f rad "
                "desired_vel=%.3f rad/s measured_vel=%.3f rad/s",
                label,
                (double)estimated_effort_nm,
                (double)target_rad,
                (double)feedback->position_rad,
                (double)desired_velocity_rad_s,
                (double)feedback->velocity_rad_s);
            return ESP_ERR_INVALID_STATE;
        }

        rs02_frame_t frame = {0};
        if (!rs02_make_operation_control(
                (uint8_t)CONFIG_RS02_MOTOR_ID,
                target_rad,
                desired_velocity_rad_s,
                0.0f,
                RS02_MOTION_KP,
                RS02_MOTION_KD,
                &frame)) {
            return ESP_ERR_INVALID_ARG;
        }
        ESP_RETURN_ON_ERROR(transmit_frame(&frame, label), TAG, "motion transmit failed");
        ESP_RETURN_ON_ERROR(
            wait_for_feedback(CONTROL_FEEDBACK_TIMEOUT_MS, false, feedback),
            TAG,
            "motion feedback timeout or fault");
        ESP_RETURN_ON_ERROR(
            check_motion_feedback(feedback, target_rad, label, "post-command"),
            TAG,
            "unsafe motion feedback after transmit");

        if (elapsed_ms >= next_log_ms || elapsed_ms >= duration_ms) {
            ESP_LOGI(
                TAG,
                "%s feedback: target=%.4f rad pos=%.4f rad error=%.4f rad "
                "vel=%.3f rad/s torque=%.3f N m effort_est=%.3f N m",
                label,
                (double)target_rad,
                (double)feedback->position_rad,
                (double)(feedback->position_rad - target_rad),
                (double)feedback->velocity_rad_s,
                (double)feedback->torque_nm,
                (double)estimated_effort_nm);
            if (next_log_ms <= UINT32_MAX - CONTROL_LOG_PERIOD_MS) {
                next_log_ms += CONTROL_LOG_PERIOD_MS;
            }
        }
        if (elapsed_ms >= duration_ms) {
            return ESP_OK;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        elapsed_ms = (uint32_t)(monotonic_ms() - start_ms);
    }
}

static esp_err_t run_motion_test(rs02_feedback_t *feedback)
{
    const float origin_rad = feedback->position_rad;
    float endpoint_rad = 0.0f;
    if (!rs02_motion_endpoint(origin_rad, &endpoint_rad)) {
        ESP_LOGE(TAG, "cannot choose motion endpoint from origin %.4f rad", (double)origin_rad);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(
        TAG,
        "motion test ENABLED: one %.1f degree out-and-back movement, Kp=%.1f Kd=%.1f",
        (double)(RS02_MOTION_OFFSET_RAD * 180.0f / 3.14159265358979323846f),
        (double)RS02_MOTION_KP,
        (double)RS02_MOTION_KD);
    ESP_RETURN_ON_ERROR(
        run_motion_phase(
            "motion outbound",
            origin_rad,
            endpoint_rad,
            RS02_MOTION_LEG_DURATION_MS,
            feedback),
        TAG,
        "outbound motion failed");
    ESP_RETURN_ON_ERROR(
        run_motion_phase(
            "outbound settle",
            endpoint_rad,
            endpoint_rad,
            RS02_MOTION_SETTLE_DURATION_MS,
            feedback),
        TAG,
        "outbound settle failed");
    if (!rs02_motion_endpoint_reached(feedback, endpoint_rad)) {
        ESP_LOGE(
            TAG,
            "outbound endpoint not reached: target=%.4f rad measured=%.4f rad error=%.4f rad",
            (double)endpoint_rad,
            (double)feedback->position_rad,
            (double)(feedback->position_rad - endpoint_rad));
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(
        run_motion_phase(
            "motion return",
            endpoint_rad,
            origin_rad,
            RS02_MOTION_LEG_DURATION_MS,
            feedback),
        TAG,
        "return motion failed");
    ESP_RETURN_ON_ERROR(
        run_motion_phase(
            "return settle",
            origin_rad,
            origin_rad,
            RS02_MOTION_SETTLE_DURATION_MS,
            feedback),
        TAG,
        "return settle failed");
    if (!rs02_motion_endpoint_reached(feedback, origin_rad)) {
        ESP_LOGE(
            TAG,
            "return endpoint not reached: target=%.4f rad measured=%.4f rad error=%.4f rad",
            (double)origin_rad,
            (double)feedback->position_rad,
            (double)(feedback->position_rad - origin_rad));
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(
        TAG,
        "%.1f-degree out-and-back motion test completed",
        (double)(RS02_MOTION_OFFSET_RAD * 180.0f / 3.14159265358979323846f));
    return ESP_OK;
}
#endif

static esp_err_t run_test_sequence(void)
{
    rs02_frame_t frame = {0};
    rs02_device_info_t device = {0};
    rs02_feedback_t feedback = {0};

    ESP_RETURN_ON_ERROR(send_stop(false), TAG, "initial stop failed");
    ESP_RETURN_ON_ERROR(
        wait_for_feedback(STAGE_FEEDBACK_TIMEOUT_MS, true, &feedback),
        TAG,
        "no feedback after initial stop");
    if (feedback.mode != RS02_MODE_RESET) {
        ESP_LOGE(TAG, "initial stop not confirmed: expected Reset mode, got mode=%u",
                 (unsigned int)feedback.mode);
        return ESP_ERR_INVALID_STATE;
    }

    if (!rs02_make_device_id_request(
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(transmit_frame(&frame, "device ID request"), TAG, "device ID request failed");
    ESP_RETURN_ON_ERROR(wait_for_device_id(&device), TAG, "device ID response timeout");
    ESP_LOGI(
        TAG,
        "device found: motor=0x%02x uid=%02x%02x%02x%02x%02x%02x%02x%02x",
        device.motor_id,
        device.mcu_uid[0], device.mcu_uid[1], device.mcu_uid[2], device.mcu_uid[3],
        device.mcu_uid[4], device.mcu_uid[5], device.mcu_uid[6], device.mcu_uid[7]);

    float bus_voltage_v = 0.0f;
    ESP_RETURN_ON_ERROR(
        read_parameter_float(RS02_PARAMETER_BUS_VOLTAGE, "bus-voltage read", &bus_voltage_v),
        TAG,
        "bus-voltage parameter read failed");
    ESP_LOGI(TAG, "reported bus voltage: %.2f V", (double)bus_voltage_v);
    if (!rs02_bus_voltage_is_safe(bus_voltage_v)) {
        ESP_LOGE(TAG, "bus voltage %.2f V is outside the documented 24-60 V range",
                 (double)bus_voltage_v);
        return ESP_ERR_INVALID_STATE;
    }

    float mechanical_position_rad = 0.0f;
    ESP_RETURN_ON_ERROR(
        read_parameter_float(
            RS02_PARAMETER_MECHANICAL_POSITION,
            "mechanical-position read",
            &mechanical_position_rad),
        TAG,
        "mechanical-position parameter read failed");
    ESP_LOGI(TAG, "reported load-side mechanical position: %.4f rad",
             (double)mechanical_position_rad);

    ESP_RETURN_ON_ERROR(
        select_operation_control_mode(),
        TAG,
        "operation-control mode selection failed");
#if CONFIG_RS02_RUN_MOTION_TEST
    ESP_RETURN_ON_ERROR(
        configure_motion_torque_limit(),
        TAG,
        "motion torque-limit configuration failed");
#endif

    if (!rs02_make_enable(
            (uint8_t)CONFIG_RS02_MOTOR_ID,
            (uint8_t)CONFIG_RS02_HOST_ID,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(transmit_frame(&frame, "enable"), TAG, "enable failed");
    ESP_RETURN_ON_ERROR(
        wait_for_feedback(STAGE_FEEDBACK_TIMEOUT_MS, true, &feedback),
        TAG,
        "no feedback after enable");
    if (feedback.mode != RS02_MODE_MOTOR) {
        ESP_LOGE(TAG, "motor did not enter Motor mode; mode=%u", (unsigned int)feedback.mode);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(run_zero_effort_phase(&feedback), TAG, "zero-effort phase failed");
    if (LINK_SOAK_TEST_ENABLED) {
        ESP_LOGI(TAG, "60-second zero-effort link soak completed without a fatal TWAI alert");
        log_bus_status();
    }

#if CONFIG_RS02_RUN_MOTION_TEST
    ESP_RETURN_ON_ERROR(run_motion_test(&feedback), TAG, "motion test failed");
#endif

    ESP_RETURN_ON_ERROR(send_stop(false), TAG, "final stop failed");
    ESP_RETURN_ON_ERROR(
        wait_for_feedback(STAGE_FEEDBACK_TIMEOUT_MS, true, &feedback),
        TAG,
        "no feedback after final stop");
    if (feedback.mode != RS02_MODE_RESET) {
        ESP_LOGE(TAG, "final stop not confirmed: expected Reset mode, got mode=%u",
                 (unsigned int)feedback.mode);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

void app_main(void)
{
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGW(TAG, "reset reason=%d (%s)", (int)reset_reason, reset_reason_name(reset_reason));
    ESP_LOGW(TAG, "advanced 48 V RS02 experiment: secure and unload the motor before power-up");
    ESP_LOGI(
        TAG,
        "TWAI 1 Mbit/s at 80%% sample point, TX GPIO%d, RX GPIO%d, motor=0x%02x, "
        "host=0x%02x, link_soak=%s, motion=%s",
        CONFIG_RS02_TWAI_TX_GPIO,
        CONFIG_RS02_TWAI_RX_GPIO,
        CONFIG_RS02_MOTOR_ID,
        CONFIG_RS02_HOST_ID,
        LINK_SOAK_TEST_STATUS,
        MOTION_TEST_STATUS);

    esp_err_t err = rs02_twai_install(
        (gpio_num_t)CONFIG_RS02_TWAI_TX_GPIO,
        (gpio_num_t)CONFIG_RS02_TWAI_RX_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI install failed: %s", esp_err_to_name(err));
        return;
    }

    err = rs02_twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(err));
        rs02_twai_shutdown();
        return;
    }

    err = run_test_sequence();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "test aborted: %s", esp_err_to_name(err));
        const esp_err_t stop_error = send_stop_best_effort();
        if (stop_error != ESP_OK) {
            ESP_LOGE(
                TAG,
                "best-effort stop was not confirmed: %s; disconnect motor power",
                esp_err_to_name(stop_error));
        }
    } else {
        ESP_LOGI(TAG, "test sequence completed and stop feedback was received");
    }

    const esp_err_t shutdown_error = rs02_twai_shutdown();
    if (shutdown_error != ESP_OK) {
        ESP_LOGE(TAG, "TWAI shutdown failed: %s", esp_err_to_name(shutdown_error));
    }
    ESP_LOGI(TAG, "TWAI is shut down; remove RS02 motor power before rewiring");
}
