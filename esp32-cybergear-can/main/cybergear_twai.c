#include "cybergear_twai.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "cybergear_twai_timing.h"

#define CYBERGEAR_TWAI_ALERTS                                                              \
    (TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_FAILED | TWAI_ALERT_RX_QUEUE_FULL |               \
     TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF |                     \
     TWAI_ALERT_RX_FIFO_OVERRUN)

static bool s_installed;
static bool s_started;

esp_err_t cybergear_twai_install(gpio_num_t tx_gpio, gpio_num_t rx_gpio)
{
    if (s_installed || tx_gpio == GPIO_NUM_NC || rx_gpio == GPIO_NUM_NC || tx_gpio == rx_gpio) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    general.tx_queue_len = 8;
    general.rx_queue_len = 32;
    general.alerts_enabled = CYBERGEAR_TWAI_ALERTS;

    cybergear_twai_timing_spec_t timing_spec = {0};
    if (!cybergear_twai_timing_1mbps_80_percent(&timing_spec)) {
        return ESP_ERR_INVALID_STATE;
    }
    const twai_timing_config_t timing = {
        .clk_src = TWAI_CLK_SRC_DEFAULT,
        .quanta_resolution_hz = timing_spec.quanta_resolution_hz,
        .brp = 0,
        .prop_seg = timing_spec.propagation_segment,
        .tseg_1 = timing_spec.time_segment_1,
        .tseg_2 = timing_spec.time_segment_2,
        .sjw = timing_spec.sync_jump_width,
        .ssp_offset = 0,
        .triple_sampling = timing_spec.triple_sampling,
    };
    const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    const esp_err_t err = twai_driver_install(&general, &timing, &filter);
    if (err == ESP_OK) {
        s_installed = true;
    }
    return err;
}

esp_err_t cybergear_twai_start(void)
{
    if (!s_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_started) {
        return ESP_OK;
    }

    const esp_err_t err = twai_start();
    if (err == ESP_OK) {
        s_started = true;
    }
    return err;
}

esp_err_t cybergear_twai_transmit(const cybergear_frame_t *frame, TickType_t ticks_to_wait)
{
    if (!s_started || frame == NULL || frame->identifier > CYBERGEAR_CAN_ID_MAX ||
        frame->data_length_code != CYBERGEAR_CAN_PAYLOAD_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t message = {
        .identifier = frame->identifier,
        .data_length_code = frame->data_length_code,
        .extd = 1,
        .rtr = 0,
    };
    memcpy(message.data, frame->data, sizeof(frame->data));
    return twai_transmit(&message, ticks_to_wait);
}

esp_err_t cybergear_twai_receive(cybergear_frame_t *frame, TickType_t ticks_to_wait)
{
    if (!s_started || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t message = {0};
    const esp_err_t err = twai_receive(&message, ticks_to_wait);
    if (err != ESP_OK) {
        return err;
    }
    if (!message.extd || message.rtr || message.data_length_code != CYBERGEAR_CAN_PAYLOAD_SIZE ||
        message.identifier > CYBERGEAR_CAN_ID_MAX) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(frame, 0, sizeof(*frame));
    frame->identifier = message.identifier;
    frame->data_length_code = message.data_length_code;
    memcpy(frame->data, message.data, sizeof(frame->data));
    return ESP_OK;
}

esp_err_t cybergear_twai_read_alerts(uint32_t *alerts, TickType_t ticks_to_wait)
{
    if (!s_started || alerts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return twai_read_alerts(alerts, ticks_to_wait);
}

esp_err_t cybergear_twai_get_status(twai_status_info_t *status)
{
    if (!s_installed || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return twai_get_status_info(status);
}

esp_err_t cybergear_twai_shutdown(void)
{
    if (!s_installed) {
        return ESP_OK;
    }

    esp_err_t first_error = ESP_OK;
    twai_status_info_t status = {0};
    if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_RUNNING) {
        const esp_err_t stop_error = twai_stop();
        if (stop_error != ESP_OK) {
            first_error = stop_error;
        } else {
            s_started = false;
        }
    } else {
        s_started = false;
    }

    const esp_err_t uninstall_error = twai_driver_uninstall();
    if (uninstall_error == ESP_OK) {
        s_installed = false;
        s_started = false;
    } else if (first_error == ESP_OK) {
        first_error = uninstall_error;
    }

    return first_error;
}
