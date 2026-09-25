#include <inttypes.h>
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

#include "cybergear_motion.h"
#include "cybergear_protocol.h"
#include "cybergear_twai.h"

#define TAG "cybergear_can"

#define DEVICE_RESPONSE_TIMEOUT_MS 1500U
#define STAGE_FEEDBACK_TIMEOUT_MS 1500U
#define CONTROL_FEEDBACK_TIMEOUT_MS 250U
#define CONTROL_PERIOD_MS 20U
#define CONTROL_LOG_PERIOD_MS 250U
#define TEMPERATURE_LIMIT_C 70.0f

#if CONFIG_CYBERGEAR_RUN_MOTION_TEST
#define MOTION_TEST_STATUS "ENABLED"
#else
#define MOTION_TEST_STATUS "disabled"
#endif

#if CONFIG_CYBERGEAR_RUN_LINK_SOAK_TEST
#define LINK_SOAK_TEST_STATUS "ENABLED"
#define LINK_SOAK_TEST_ENABLED true
#else
#define LINK_SOAK_TEST_STATUS "disabled"
#define LINK_SOAK_TEST_ENABLED false
#endif

#if CONFIG_CYBERGEAR_RUN_LINK_SOAK_TEST && CONFIG_CYBERGEAR_RUN_MOTION_TEST
#error "Enable either the zero-effort link soak or the motion test, not both"
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
    const esp_err_t err = cybergear_twai_get_status(&status);
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
    const esp_err_t err = cybergear_twai_read_alerts(&alerts, 0);
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

static esp_err_t transmit_frame(const cybergear_frame_t *frame, const char *label)
{
    const esp_err_t alert_error = check_bus_alerts();
    if (alert_error != ESP_OK) {
        return alert_error;
    }

    const esp_err_t err = cybergear_twai_transmit(frame, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s transmit failed: %s", label, esp_err_to_name(err));
        log_bus_status();
        return err;
    }

    ESP_LOGD(TAG, "%s TX id=0x%08" PRIx32, label, frame->identifier);
    return ESP_OK;
}

static bool feedback_is_safe(const cybergear_feedback_t *feedback)
{
    if (feedback->motor_id != (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID) {
        ESP_LOGE(TAG, "feedback came from unexpected motor 0x%02x", feedback->motor_id);
        return false;
    }
    if (feedback->faults != 0U) {
        ESP_LOGE(TAG, "motor fault flags=0x%02x", feedback->faults);
        return false;
    }
    if (feedback->temperature_c >= TEMPERATURE_LIMIT_C) {
        ESP_LOGE(TAG, "motor temperature %.1f C reached the %.1f C limit",
                 (double)feedback->temperature_c,
                 (double)TEMPERATURE_LIMIT_C);
        return false;
    }
    return true;
}

static esp_err_t reject_fault_feedback(const cybergear_frame_t *frame)
{
    cybergear_fault_feedback_t fault = {0};
    if (!cybergear_parse_fault_feedback(
            frame,
            (uint8_t)CONFIG_CYBERGEAR_HOST_ID,
            &fault)) {
        return ESP_OK;
    }
    if (fault.motor_id != (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID) {
        ESP_LOGW(TAG, "ignored fault feedback from unexpected motor 0x%02x", fault.motor_id);
        return ESP_OK;
    }
    if (!fault.has_fault && !fault.has_warning) {
        return ESP_OK;
    }

    ESP_LOGE(
        TAG,
        "motor fault frame: fault=%02x%02x%02x%02x warning=%02x%02x%02x%02x",
        fault.fault_data[0], fault.fault_data[1], fault.fault_data[2], fault.fault_data[3],
        fault.warning_data[0], fault.warning_data[1], fault.warning_data[2], fault.warning_data[3]);
    return ESP_FAIL;
}

static esp_err_t wait_for_device_id(cybergear_device_info_t *device)
{
    const uint32_t start_ms = monotonic_ms();
    while ((uint32_t)(monotonic_ms() - start_ms) < DEVICE_RESPONSE_TIMEOUT_MS) {
        ESP_RETURN_ON_ERROR(check_bus_alerts(), TAG, "TWAI alert while waiting for device ID");

        cybergear_frame_t frame = {0};
        const esp_err_t err = cybergear_twai_receive(&frame, pdMS_TO_TICKS(20));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_RETURN_ON_ERROR(err, TAG, "invalid CAN frame while waiting for device ID");
        ESP_RETURN_ON_ERROR(reject_fault_feedback(&frame), TAG, "motor fault while waiting for device ID");

        if (cybergear_parse_device_id(&frame, device)) {
            return ESP_OK;
        }
        ESP_LOGD(TAG, "ignored frame id=0x%08" PRIx32 " while waiting for device ID", frame.identifier);
    }
    return ESP_ERR_TIMEOUT;
}

static void log_feedback_sample(const char *prefix, const cybergear_feedback_t *feedback)
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

static esp_err_t wait_for_feedback(uint32_t timeout_ms, bool log_sample,
                                   cybergear_feedback_t *feedback)
{
    const uint32_t start_ms = monotonic_ms();
    while ((uint32_t)(monotonic_ms() - start_ms) < timeout_ms) {
        ESP_RETURN_ON_ERROR(check_bus_alerts(), TAG, "TWAI alert while waiting for feedback");

        cybergear_frame_t frame = {0};
        const esp_err_t err = cybergear_twai_receive(&frame, pdMS_TO_TICKS(20));
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_RETURN_ON_ERROR(err, TAG, "invalid CAN frame while waiting for feedback");
        ESP_RETURN_ON_ERROR(reject_fault_feedback(&frame), TAG, "motor fault feedback received");

        if (!cybergear_parse_feedback(&frame, (uint8_t)CONFIG_CYBERGEAR_HOST_ID, feedback)) {
            ESP_LOGD(TAG, "ignored frame id=0x%08" PRIx32 " while waiting for feedback", frame.identifier);
            continue;
        }
        if (!feedback_is_safe(feedback)) {
            return ESP_FAIL;
        }

        if (log_sample) {
            log_feedback_sample("feedback", feedback);
        }
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t send_stop(bool clear_fault)
{
    cybergear_frame_t frame = {0};
    if (!cybergear_make_stop(
            (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID,
            (uint8_t)CONFIG_CYBERGEAR_HOST_ID,
            clear_fault,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    return transmit_frame(&frame, clear_fault ? "clear-fault stop" : "stop");
}

static esp_err_t send_stop_best_effort(void)
{
    cybergear_frame_t frame = {0};
    if (!cybergear_make_stop(
            (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID,
            (uint8_t)CONFIG_CYBERGEAR_HOST_ID,
            false,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < 32U; ++i) {
        cybergear_frame_t stale_frame = {0};
        const esp_err_t receive_error = cybergear_twai_receive(&stale_frame, 0);
        if (receive_error == ESP_ERR_TIMEOUT) {
            break;
        }
        if (receive_error != ESP_OK) {
            ESP_LOGW(
                TAG,
                "could not drain TWAI RX queue before stop: %s",
                esp_err_to_name(receive_error));
            break;
        }
        if (i == 31U) {
            ESP_LOGW(TAG, "TWAI RX queue remained busy before stop confirmation");
        }
    }

    // Bypass latched alert rejection so a stop is still attempted when the controller can transmit.
    ESP_RETURN_ON_ERROR(
        cybergear_twai_transmit(&frame, pdMS_TO_TICKS(100)),
        TAG,
        "best-effort stop transmit failed");

    const uint32_t start_ms = monotonic_ms();
    while ((uint32_t)(monotonic_ms() - start_ms) < CONTROL_FEEDBACK_TIMEOUT_MS) {
        cybergear_frame_t response = {0};
        const esp_err_t receive_error = cybergear_twai_receive(&response, pdMS_TO_TICKS(20));
        if (receive_error == ESP_ERR_TIMEOUT) {
            continue;
        }
        ESP_RETURN_ON_ERROR(receive_error, TAG, "stop confirmation receive failed");

        cybergear_feedback_t feedback = {0};
        if (cybergear_parse_feedback(
                &response,
                (uint8_t)CONFIG_CYBERGEAR_HOST_ID,
                &feedback) &&
            feedback.motor_id == (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID) {
            if (feedback.mode != CYBERGEAR_MODE_RESET) {
                ESP_LOGW(TAG, "stop not confirmed: expected Reset mode, got mode=%u",
                         (unsigned int)feedback.mode);
                continue;
            }
            ESP_LOGI(TAG, "stop feedback received before TWAI shutdown; mode=%u faults=0x%02x",
                     (unsigned int)feedback.mode,
                     feedback.faults);
            return ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t check_control_feedback(
    const cybergear_feedback_t *feedback,
    float target_rad,
    float velocity_limit_rad_s,
    float torque_limit_nm,
    const char *phase_label,
    const char *timing_label)
{
    if (cybergear_control_feedback_is_safe(feedback, target_rad, velocity_limit_rad_s,
                                           torque_limit_nm)) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "%s unsafe %s feedback: mode=%u pos=%.4f rad "
             "vel=%.3f rad/s torque=%.3f N m target=%.4f rad tracking_error=%.4f rad "
             "(require Motor, |vel|<%.1f, |torque|<%.1f, |tracking_error|<0.10)",
             phase_label, timing_label, (unsigned int)feedback->mode,
             (double)feedback->position_rad, (double)feedback->velocity_rad_s,
             (double)feedback->torque_nm, (double)target_rad,
             (double)(feedback->position_rad - target_rad),
             (double)velocity_limit_rad_s, (double)torque_limit_nm);
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t run_control_phase(
    const char *label,
    float start_position_rad,
    float end_position_rad,
    float desired_velocity_rad_s,
    float feedforward_torque_nm,
    float kp,
    float kd,
    uint32_t duration_ms,
    float velocity_limit_rad_s,
    float torque_limit_nm,
    cybergear_feedback_t *feedback)
{
    ESP_LOGI(TAG, "%s targets: start=%.4f rad end=%.4f rad duration=%" PRIu32
             " ms speed guard=%.1f rad/s torque guard=%.1f N m",
             label, (double)start_position_rad, (double)end_position_rad, duration_ms,
             (double)velocity_limit_rad_s, (double)torque_limit_nm);
    const uint32_t start_ms = monotonic_ms();
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t elapsed_ms = 0U;
    uint32_t next_log_ms = 0U;

    for (;;) {
        float target_position_rad = 0.0f;
        if (!cybergear_motion_target(start_position_rad, end_position_rad,
                                    elapsed_ms, duration_ms, &target_position_rad)) {
            return ESP_ERR_INVALID_ARG;
        }
        // Check against the target about to be sent, so a stalled motor or delayed
        // trajectory jump cannot accumulate a large position error before transmission.
        ESP_RETURN_ON_ERROR(
            check_control_feedback(feedback, target_position_rad, velocity_limit_rad_s,
                                   torque_limit_nm, label, "pre-command"),
            TAG, "unsafe feedback before control command");
        float estimated_effort_nm = 0.0f;
        if (!cybergear_motion_command_effort_is_safe(
                feedback, target_position_rad, desired_velocity_rad_s,
                feedforward_torque_nm, kp, kd, torque_limit_nm,
                &estimated_effort_nm)) {
            ESP_LOGE(TAG, "%s unsafe pre-command controller effort: estimated=%.3f N m "
                     "limit=%.1f N m target=%.4f rad measured=%.4f rad "
                     "desired_vel=%.3f rad/s measured_vel=%.3f rad/s Kp=%.1f Kd=%.1f",
                     label, (double)estimated_effort_nm, (double)torque_limit_nm,
                     (double)target_position_rad, (double)feedback->position_rad,
                     (double)desired_velocity_rad_s, (double)feedback->velocity_rad_s,
                     (double)kp, (double)kd);
            return ESP_ERR_INVALID_STATE;
        }
        cybergear_frame_t frame = {0};
        if (!cybergear_make_motion_control(
                (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID,
                target_position_rad,
                desired_velocity_rad_s,
                feedforward_torque_nm,
                kp,
                kd,
                &frame)) {
            return ESP_ERR_INVALID_ARG;
        }

        ESP_RETURN_ON_ERROR(transmit_frame(&frame, "motion control"), TAG, "control transmit failed");
        ESP_RETURN_ON_ERROR(
            wait_for_feedback(CONTROL_FEEDBACK_TIMEOUT_MS, false, feedback),
            TAG,
            "control feedback timeout or fault");
        ESP_RETURN_ON_ERROR(
            check_control_feedback(feedback, target_position_rad, velocity_limit_rad_s,
                                   torque_limit_nm, label, "post-command"),
            TAG, "unsafe feedback after control command");
        // Even a delayed sample sends and checks the exact final target before completion.
        static const char *const telemetry_prefix = "control feedback";
        if (elapsed_ms >= next_log_ms || elapsed_ms >= duration_ms) {
            log_feedback_sample(telemetry_prefix, feedback);
            if (next_log_ms <= UINT32_MAX - CONTROL_LOG_PERIOD_MS) {
                next_log_ms += CONTROL_LOG_PERIOD_MS;
            }
        }
        if (elapsed_ms >= duration_ms) {
            ESP_LOGI(TAG, "%s command sequence complete: target=%.4f rad measured=%.4f rad "
                     "endpoint error(measured-target)=%.4f rad; arrival is not asserted",
                     label, (double)end_position_rad, (double)feedback->position_rad,
                     (double)(feedback->position_rad - end_position_rad));
            return ESP_OK;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
        elapsed_ms = (uint32_t)(monotonic_ms() - start_ms);
    }
}

static esp_err_t run_test_sequence(void)
{
    cybergear_frame_t frame = {0};
    cybergear_device_info_t device = {0};
    cybergear_feedback_t feedback = {0};

    ESP_RETURN_ON_ERROR(send_stop(false), TAG, "initial stop failed");
    ESP_RETURN_ON_ERROR(
        wait_for_feedback(STAGE_FEEDBACK_TIMEOUT_MS, true, &feedback),
        TAG,
        "no feedback after initial stop");
    if (feedback.mode != CYBERGEAR_MODE_RESET) {
        ESP_LOGE(TAG, "initial stop not confirmed: expected Reset mode, got mode=%u",
                 (unsigned int)feedback.mode);
        return ESP_ERR_INVALID_STATE;
    }

    if (!cybergear_make_device_id_request(
            (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID,
            (uint8_t)CONFIG_CYBERGEAR_HOST_ID,
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
    if (device.motor_id != (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (!cybergear_make_enable(
            (uint8_t)CONFIG_CYBERGEAR_MOTOR_ID,
            (uint8_t)CONFIG_CYBERGEAR_HOST_ID,
            &frame)) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(transmit_frame(&frame, "enable"), TAG, "enable failed");
    ESP_RETURN_ON_ERROR(
        wait_for_feedback(STAGE_FEEDBACK_TIMEOUT_MS, true, &feedback),
        TAG,
        "no feedback after enable");
    if (feedback.mode != CYBERGEAR_MODE_MOTOR) {
        ESP_LOGE(TAG, "motor did not enter Motor mode; mode=%u", (unsigned int)feedback.mode);
        return ESP_ERR_INVALID_STATE;
    }

    cybergear_zero_effort_profile_t zero_effort = {0};
    if (!cybergear_zero_effort_profile(LINK_SOAK_TEST_ENABLED, &zero_effort)) {
        return ESP_ERR_INVALID_STATE;
    }
    const float initial_position_rad = feedback.position_rad;
    const float zero_effort_target_rad = initial_position_rad + zero_effort.target_offset_rad;
    ESP_LOGI(TAG, "zero-effort phase at measured position %.4f rad for %" PRIu32 " ms",
             (double)initial_position_rad, zero_effort.duration_ms);
    ESP_RETURN_ON_ERROR(
        run_control_phase("zero effort", initial_position_rad, zero_effort_target_rad,
                          zero_effort.desired_velocity_rad_s,
                          zero_effort.feedforward_torque_nm,
                          zero_effort.kp, zero_effort.kd, zero_effort.duration_ms,
                          zero_effort.speed_guard_rad_s,
                          CYBERGEAR_ZERO_EFFORT_TORQUE_GUARD_NM, &feedback),
        TAG,
        "zero-effort phase failed");
    if (LINK_SOAK_TEST_ENABLED) {
        ESP_LOGI(TAG, "60-second zero-effort link soak completed without a fatal TWAI alert");
        log_bus_status();
    }

#if CONFIG_CYBERGEAR_RUN_MOTION_TEST
    const float motion_origin_rad = feedback.position_rad;
    float motion_target_rad = 0.0f;
    if (!cybergear_motion_endpoint(motion_origin_rad, &motion_target_rad)) {
        ESP_LOGE(TAG, "invalid motion origin %.4f rad", (double)motion_origin_rad);
        return ESP_ERR_INVALID_ARG;
    }
    float outbound_velocity_rad_s = 0.0f;
    float return_velocity_rad_s = 0.0f;
    if (!cybergear_motion_desired_velocity(
            motion_origin_rad, motion_target_rad, CYBERGEAR_MOTION_DURATION_MS,
            &outbound_velocity_rad_s) ||
        !cybergear_motion_desired_velocity(
            motion_target_rad, motion_origin_rad, CYBERGEAR_MOTION_DURATION_MS,
            &return_velocity_rad_s)) {
        ESP_LOGE(TAG, "could not derive valid Motion velocity targets");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGW(TAG, "motion opt-in enabled: one 360-degree out-and-back ramp, 10+10 seconds; "
             "Kp=%.1f Kd=%.1f desired velocity=%+.3f/%+.3f rad/s feed-forward torque=0; "
             "anomaly guards are not hard limits",
             (double)CYBERGEAR_MOTION_KP, (double)CYBERGEAR_MOTION_KD,
             (double)outbound_velocity_rad_s, (double)return_velocity_rad_s);
    ESP_RETURN_ON_ERROR(
        run_control_phase("outbound", motion_origin_rad, motion_target_rad,
                          outbound_velocity_rad_s, 0.0f,
                          CYBERGEAR_MOTION_KP, CYBERGEAR_MOTION_KD,
                          CYBERGEAR_MOTION_DURATION_MS,
                          CYBERGEAR_MOTION_SPEED_GUARD_RAD_S,
                          CYBERGEAR_MOTION_TORQUE_GUARD_NM, &feedback),
        TAG,
        "outbound motion phase failed");
    ESP_RETURN_ON_ERROR(
        run_control_phase("return", motion_target_rad, motion_origin_rad,
                          return_velocity_rad_s, 0.0f,
                          CYBERGEAR_MOTION_KP, CYBERGEAR_MOTION_KD,
                          CYBERGEAR_MOTION_DURATION_MS,
                          CYBERGEAR_MOTION_SPEED_GUARD_RAD_S,
                          CYBERGEAR_MOTION_TORQUE_GUARD_NM, &feedback),
        TAG,
        "return motion phase failed");
#else
    ESP_LOGW(TAG, "motion test is disabled; communication and zero-effort validation only");
#endif

    ESP_RETURN_ON_ERROR(send_stop(false), TAG, "final stop failed");
    ESP_RETURN_ON_ERROR(
        wait_for_feedback(STAGE_FEEDBACK_TIMEOUT_MS, true, &feedback),
        TAG,
        "no feedback after final stop");
    if (feedback.mode != CYBERGEAR_MODE_RESET) {
        ESP_LOGE(TAG, "final stop not confirmed: expected Reset mode, got mode=%u",
                 (unsigned int)feedback.mode);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

void app_main(void)
{
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGW(TAG, "reset reason=%d (%s)", (int)reset_reason,
             reset_reason_name(reset_reason));
    ESP_LOGW(TAG, "advanced 24 V experiment: secure and unload the motor before applying power");
    ESP_LOGI(
        TAG,
        "TWAI 1 Mbit/s at 80%% sample point, TX GPIO%d, RX GPIO%d, motor=0x%02x, "
        "host=0x%02x, link_soak=%s, motion=%s",
        CONFIG_CYBERGEAR_TWAI_TX_GPIO,
        CONFIG_CYBERGEAR_TWAI_RX_GPIO,
        CONFIG_CYBERGEAR_MOTOR_ID,
        CONFIG_CYBERGEAR_HOST_ID,
        LINK_SOAK_TEST_STATUS,
        MOTION_TEST_STATUS);

    esp_err_t err = cybergear_twai_install(
        (gpio_num_t)CONFIG_CYBERGEAR_TWAI_TX_GPIO,
        (gpio_num_t)CONFIG_CYBERGEAR_TWAI_RX_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI install failed: %s", esp_err_to_name(err));
        return;
    }

    err = cybergear_twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(err));
        cybergear_twai_shutdown();
        return;
    }

    err = run_test_sequence();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "test aborted: %s", esp_err_to_name(err));
        const esp_err_t stop_error = send_stop_best_effort();
        if (stop_error != ESP_OK) {
            ESP_LOGE(
                TAG,
                "best-effort stop was not confirmed: %s; disconnect 24 V power",
                esp_err_to_name(stop_error));
        }
    } else {
        ESP_LOGI(TAG, "test sequence completed and stop feedback was received");
    }

    const esp_err_t shutdown_error = cybergear_twai_shutdown();
    if (shutdown_error != ESP_OK) {
        ESP_LOGE(TAG, "TWAI shutdown failed: %s", esp_err_to_name(shutdown_error));
    }
    ESP_LOGI(TAG, "TWAI is shut down; remove 24 V motor power before rewiring");
}
