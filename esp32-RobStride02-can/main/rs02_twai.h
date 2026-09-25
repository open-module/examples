#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "rs02_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t rs02_twai_install(gpio_num_t tx_gpio, gpio_num_t rx_gpio);
esp_err_t rs02_twai_start(void);
esp_err_t rs02_twai_transmit(const rs02_frame_t *frame, TickType_t ticks_to_wait);
esp_err_t rs02_twai_receive(rs02_frame_t *frame, TickType_t ticks_to_wait);
esp_err_t rs02_twai_read_alerts(uint32_t *alerts, TickType_t ticks_to_wait);
esp_err_t rs02_twai_get_status(twai_status_info_t *status);
esp_err_t rs02_twai_shutdown(void);

#ifdef __cplusplus
}
#endif
