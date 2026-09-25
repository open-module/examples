#include "rs02_web_console.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"

#define TAG "rs02_web"
#define ARM_WINDOW_MS 30000U
#define HEARTBEAT_TIMEOUT_MS 2000U
#define REQUEST_BODY_MAX 256U

extern const char web_index_html_start[] asm("_binary_index_html_start");
extern const char web_index_html_end[] asm("_binary_index_html_end");

typedef struct {
    rs02_web_state_t state;
    rs02_motion_config_t config;
    rs02_feedback_t feedback;
    bool has_feedback;
    bool stop_requested;
    bool prepare_requested;
    bool start_requested;
    uint32_t armed_at_ms;
    uint32_t heartbeat_at_ms;
    float bus_voltage_v;
    float mechanical_position_rad;
    float target_rad;
    float progress;
    bool has_motion_plan;
    float motion_plan_current_angle_deg;
    float motion_plan_target_angle_deg;
    float motion_plan_signed_delta_deg;
    uint32_t plan_token;
    bool risk_ack_required;
    char phase[32];
    char message[192];
} web_context_t;

static SemaphoreHandle_t s_lock;
static httpd_handle_t s_server;
static web_context_t s_context;

static uint32_t monotonic_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void copy_text(char *destination, size_t capacity, const char *source)
{
    if (capacity == 0U) {
        return;
    }
    snprintf(destination, capacity, "%s", source != NULL ? source : "");
}

static const char *state_name(rs02_web_state_t state)
{
    switch (state) {
    case RS02_WEB_STATE_IDLE: return "idle";
    case RS02_WEB_STATE_ARMED: return "armed";
    case RS02_WEB_STATE_PREPARING: return "preparing";
    case RS02_WEB_STATE_STARTING: return "starting";
    case RS02_WEB_STATE_RUNNING: return "running";
    case RS02_WEB_STATE_COMPLETE: return "complete";
    case RS02_WEB_STATE_FAULT: return "fault";
    default: return "unknown";
    }
}

static bool heartbeat_is_fresh_locked(uint32_t now_ms)
{
    return (uint32_t)(now_ms - s_context.heartbeat_at_ms) < HEARTBEAT_TIMEOUT_MS;
}

static void expire_arm_if_needed_locked(uint32_t now_ms)
{
    if (s_context.state != RS02_WEB_STATE_ARMED) {
        return;
    }
    if ((uint32_t)(now_ms - s_context.armed_at_ms) >= ARM_WINDOW_MS) {
        s_context.state = RS02_WEB_STATE_IDLE;
        s_context.plan_token = 0U;
        s_context.risk_ack_required = false;
        copy_text(s_context.message, sizeof(s_context.message),
                  "Automatic preflight expired. Move a slider or select Start Test to refresh it.");
    } else if (!heartbeat_is_fresh_locked(now_ms)) {
        s_context.state = RS02_WEB_STATE_IDLE;
        s_context.plan_token = 0U;
        s_context.risk_ack_required = false;
        copy_text(s_context.message, sizeof(s_context.message),
                  "Browser heartbeat was lost before web confirmation.");
    }
}

static esp_err_t send_json(httpd_req_t *request, const char *status, cJSON *root)
{
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (body == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON allocation failed");
    }

    httpd_resp_set_status(request, status);
    httpd_resp_set_type(request, "application/json");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    const esp_err_t result = httpd_resp_sendstr(request, body);
    cJSON_free(body);
    return result;
}

static cJSON *message_json(bool ok, const char *message)
{
    cJSON *root = cJSON_CreateObject();
    if (root != NULL) {
        cJSON_AddBoolToObject(root, "ok", ok);
        cJSON_AddStringToObject(root, "message", message);
    }
    return root;
}

static esp_err_t root_get_handler(httpd_req_t *request)
{
    httpd_resp_set_type(request, "text/html; charset=utf-8");
    httpd_resp_set_hdr(request, "Cache-Control", "no-store");
    return httpd_resp_send(
        request,
        web_index_html_start,
        (ssize_t)(web_index_html_end - web_index_html_start));
}

static esp_err_t status_get_handler(httpd_req_t *request)
{
    web_context_t snapshot = {0};
    const uint32_t now_ms = monotonic_ms();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    expire_arm_if_needed_locked(now_ms);
    snapshot = s_context;
    xSemaphoreGive(s_lock);

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return httpd_resp_send_err(request, HTTPD_500_INTERNAL_SERVER_ERROR, "JSON allocation failed");
    }
    cJSON_AddStringToObject(root, "state", state_name(snapshot.state));
    cJSON_AddStringToObject(root, "message", snapshot.message);
    cJSON_AddStringToObject(root, "phase", snapshot.phase);
    cJSON_AddNumberToObject(root, "progress", snapshot.progress);
    cJSON_AddNumberToObject(root, "target_rad", snapshot.target_rad);
    cJSON_AddNumberToObject(root, "bus_voltage_v", snapshot.bus_voltage_v);
    cJSON_AddNumberToObject(root, "mechanical_position_rad", snapshot.mechanical_position_rad);
    cJSON_AddBoolToObject(root, "has_feedback", snapshot.has_feedback);

    cJSON *motion_plan = cJSON_AddObjectToObject(root, "motion_plan");
    cJSON_AddBoolToObject(motion_plan, "valid", snapshot.has_motion_plan);
    cJSON_AddNumberToObject(
        motion_plan,
        "current_angle_deg",
        snapshot.motion_plan_current_angle_deg);
    cJSON_AddNumberToObject(
        motion_plan,
        "target_angle_deg",
        snapshot.motion_plan_target_angle_deg);
    cJSON_AddNumberToObject(
        motion_plan,
        "signed_delta_deg",
        snapshot.motion_plan_signed_delta_deg);

    cJSON *config = cJSON_AddObjectToObject(root, "config");
    cJSON_AddNumberToObject(config, "angle_deg", snapshot.config.angle_deg);
    cJSON_AddNumberToObject(config, "duration_s", (double)snapshot.config.duration_ms / 1000.0);
    cJSON_AddNumberToObject(config, "torque_limit_nm", snapshot.config.torque_limit_nm);
    cJSON_AddNumberToObject(config, "speed_limit_rad_s", snapshot.config.speed_limit_rad_s);

    cJSON *feedback = cJSON_AddObjectToObject(root, "feedback");
    cJSON_AddNumberToObject(feedback, "mode", snapshot.feedback.mode);
    cJSON_AddNumberToObject(feedback, "faults", snapshot.feedback.faults);
    cJSON_AddNumberToObject(feedback, "position_rad", snapshot.feedback.position_rad);
    cJSON_AddNumberToObject(feedback, "velocity_rad_s", snapshot.feedback.velocity_rad_s);
    cJSON_AddNumberToObject(feedback, "torque_nm", snapshot.feedback.torque_nm);
    cJSON_AddNumberToObject(feedback, "temperature_c", snapshot.feedback.temperature_c);

    uint32_t arm_remaining_ms = 0U;
    if (snapshot.state == RS02_WEB_STATE_ARMED) {
        const uint32_t elapsed_ms = (uint32_t)(now_ms - snapshot.armed_at_ms);
        arm_remaining_ms = elapsed_ms < ARM_WINDOW_MS ? ARM_WINDOW_MS - elapsed_ms : 0U;
    }
    cJSON_AddNumberToObject(root, "arm_remaining_ms", arm_remaining_ms);
    cJSON_AddNumberToObject(root, "plan_token", snapshot.plan_token);
    cJSON_AddBoolToObject(root, "risk_ack_required", snapshot.risk_ack_required);
    return send_json(request, "200 OK", root);
}

static bool receive_body(httpd_req_t *request, char *body, size_t capacity)
{
    if (request->content_len <= 0 || (size_t)request->content_len >= capacity) {
        return false;
    }

    size_t received = 0U;
    while (received < (size_t)request->content_len) {
        const int count = httpd_req_recv(
            request,
            body + received,
            (size_t)request->content_len - received);
        if (count == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (count <= 0) {
            return false;
        }
        received += (size_t)count;
    }
    body[received] = '\0';
    return true;
}

static esp_err_t config_post_handler(httpd_req_t *request)
{
    char body[REQUEST_BODY_MAX + 1U] = {0};
    if (!receive_body(request, body, sizeof(body))) {
        return send_json(
            request,
            "400 Bad Request",
            message_json(false, "Request body is missing or too large."));
    }

    cJSON *json = cJSON_Parse(body);
    const cJSON *angle = json != NULL ? cJSON_GetObjectItemCaseSensitive(json, "angle_deg") : NULL;
    const cJSON *duration = json != NULL ? cJSON_GetObjectItemCaseSensitive(json, "duration_s") : NULL;
    const cJSON *torque = json != NULL ? cJSON_GetObjectItemCaseSensitive(json, "torque_limit_nm") : NULL;
    if (!cJSON_IsNumber(angle) || !cJSON_IsNumber(duration) || !cJSON_IsNumber(torque)) {
        cJSON_Delete(json);
        return send_json(
            request,
            "400 Bad Request",
            message_json(
                false,
                "angle_deg, duration_s, and torque_limit_nm must be numbers."));
    }

    const double duration_seconds = duration->valuedouble;
    uint32_t duration_ms = 0U;
    if (isfinite(duration_seconds) &&
        duration_seconds >= (double)RS02_WEB_MIN_DURATION_MS / 1000.0 &&
        duration_seconds <= (double)RS02_WEB_MAX_DURATION_MS / 1000.0) {
        duration_ms = (uint32_t)lround(duration_seconds * 1000.0);
    }
    rs02_motion_config_t config = {
        .angle_deg = (float)angle->valuedouble,
        .planned_delta_deg = RS02_WEB_MAX_SINGLE_TURN_ANGLE_DEG,
        .duration_ms = duration_ms,
        .torque_limit_nm = (float)torque->valuedouble,
        .speed_limit_rad_s = RS02_WEB_MIN_SPEED_LIMIT_RAD_S,
        .absolute_single_turn_target = true,
        .return_to_origin = false,
    };
    cJSON_Delete(json);

    if (config.duration_ms > 0U) {
        const float angle_rad =
            RS02_WEB_MAX_SINGLE_TURN_ANGLE_DEG * 3.14159265358979323846f / 180.0f;
        const float peak_speed = 1.5f * angle_rad * 1000.0f / (float)config.duration_ms;
        config.speed_limit_rad_s = fmaxf(
            RS02_WEB_MIN_SPEED_LIMIT_RAD_S,
            peak_speed + RS02_WEB_PROFILE_SPEED_MARGIN_RAD_S +
                RS02_WEB_PROFILE_SPEED_PADDING_RAD_S);
    }

    float peak_speed_rad_s = 0.0f;
    rs02_motion_config_error_t error = RS02_MOTION_CONFIG_OK;
    if (!rs02_motion_config_validate(&config, &peak_speed_rad_s, &error)) {
        return send_json(
            request,
            "400 Bad Request",
            message_json(false, rs02_motion_config_error_name(error)));
    }

    const uint32_t now_ms = monotonic_ms();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_context.state == RS02_WEB_STATE_PREPARING ||
        s_context.state == RS02_WEB_STATE_STARTING ||
        s_context.state == RS02_WEB_STATE_RUNNING) {
        xSemaphoreGive(s_lock);
        return send_json(
            request,
            "409 Conflict",
            message_json(false, "Wait for the current preflight or motion to finish."));
    }
    s_context.config = config;
    s_context.state = RS02_WEB_STATE_PREPARING;
    s_context.stop_requested = false;
    s_context.prepare_requested = true;
    s_context.start_requested = false;
    s_context.heartbeat_at_ms = now_ms;
    s_context.progress = 0.0f;
    s_context.has_motion_plan = false;
    s_context.plan_token = 0U;
    s_context.risk_ack_required = false;
    copy_text(s_context.phase, sizeof(s_context.phase), "checking-position");
    copy_text(s_context.message, sizeof(s_context.message),
              "Slider settings received. Checking the current motor position automatically.");
    xSemaphoreGive(s_lock);

    cJSON *response = message_json(true, "Configuration accepted. Motor preflight is running.");
    cJSON_AddNumberToObject(response, "peak_speed_rad_s", peak_speed_rad_s);
    return send_json(request, "200 OK", response);
}

static esp_err_t start_post_handler(httpd_req_t *request)
{
    char body[REQUEST_BODY_MAX + 1U] = {0};
    if (!receive_body(request, body, sizeof(body))) {
        return send_json(
            request,
            "400 Bad Request",
            message_json(false, "A motion confirmation payload is required."));
    }

    cJSON *json = cJSON_Parse(body);
    const cJSON *plan_token =
        json != NULL ? cJSON_GetObjectItemCaseSensitive(json, "plan_token") : NULL;
    const cJSON *risk_acknowledged =
        json != NULL ? cJSON_GetObjectItemCaseSensitive(json, "risk_acknowledged") : NULL;
    const double token_value = cJSON_IsNumber(plan_token) ? plan_token->valuedouble : 0.0;
    const bool token_is_valid = isfinite(token_value) && token_value >= 1.0 &&
                                token_value <= (double)UINT32_MAX &&
                                floor(token_value) == token_value;
    if (!token_is_valid || !cJSON_IsBool(risk_acknowledged)) {
        cJSON_Delete(json);
        return send_json(
            request,
            "400 Bad Request",
            message_json(false, "plan_token and risk_acknowledged are required."));
    }
    const uint32_t submitted_token = (uint32_t)token_value;
    const bool submitted_risk_ack = cJSON_IsTrue(risk_acknowledged);
    cJSON_Delete(json);

    const uint32_t now_ms = monotonic_ms();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    expire_arm_if_needed_locked(now_ms);
    if (s_context.state != RS02_WEB_STATE_ARMED || !heartbeat_is_fresh_locked(now_ms) ||
        s_context.plan_token == 0U || submitted_token != s_context.plan_token) {
        xSemaphoreGive(s_lock);
        return send_json(
            request,
            "409 Conflict",
            message_json(false, "The confirmed motion plan is missing, stale, or expired."));
    }
    if (s_context.risk_ack_required && !submitted_risk_ack) {
        xSemaphoreGive(s_lock);
        return send_json(
            request,
            "400 Bad Request",
            message_json(false, "This profile requires an explicit high-risk acknowledgement."));
    }
    s_context.state = RS02_WEB_STATE_STARTING;
    s_context.prepare_requested = false;
    s_context.start_requested = true;
    s_context.stop_requested = false;
    s_context.plan_token = 0U;
    s_context.risk_ack_required = false;
    copy_text(s_context.phase, sizeof(s_context.phase), "preparing");
    copy_text(s_context.message, sizeof(s_context.message),
              "Web confirmation received. Rechecking the motor before one-way motion.");
    xSemaphoreGive(s_lock);
    return send_json(request, "200 OK", message_json(true, "Start request accepted."));
}

static esp_err_t heartbeat_post_handler(httpd_req_t *request)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_context.heartbeat_at_ms = monotonic_ms();
    xSemaphoreGive(s_lock);
    return send_json(request, "200 OK", message_json(true, "heartbeat"));
}

static esp_err_t stop_post_handler(httpd_req_t *request)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_context.state == RS02_WEB_STATE_ARMED) {
        s_context.state = RS02_WEB_STATE_IDLE;
        s_context.stop_requested = false;
        s_context.prepare_requested = false;
        s_context.start_requested = false;
        s_context.plan_token = 0U;
        s_context.risk_ack_required = false;
        copy_text(s_context.phase, sizeof(s_context.phase), "idle");
        copy_text(s_context.message, sizeof(s_context.message), "Preparation cancelled.");
    } else if (s_context.state == RS02_WEB_STATE_STARTING && s_context.start_requested) {
        s_context.state = RS02_WEB_STATE_IDLE;
        s_context.stop_requested = false;
        s_context.start_requested = false;
        copy_text(s_context.phase, sizeof(s_context.phase), "idle");
        copy_text(s_context.message, sizeof(s_context.message), "Start request cancelled.");
    } else if (s_context.state == RS02_WEB_STATE_PREPARING ||
               s_context.state == RS02_WEB_STATE_STARTING ||
               s_context.state == RS02_WEB_STATE_RUNNING) {
        s_context.stop_requested = true;
        copy_text(s_context.message, sizeof(s_context.message),
                  "Stop requested. Use the physical power cutoff for an emergency.");
    }
    xSemaphoreGive(s_lock);
    return send_json(request, "200 OK", message_json(true, "Stop request accepted."));
}

static const httpd_uri_t s_root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler,
};

static const httpd_uri_t s_status_uri = {
    .uri = "/api/status",
    .method = HTTP_GET,
    .handler = status_get_handler,
};

static const httpd_uri_t s_config_uri = {
    .uri = "/api/config",
    .method = HTTP_POST,
    .handler = config_post_handler,
};

static const httpd_uri_t s_heartbeat_uri = {
    .uri = "/api/heartbeat",
    .method = HTTP_POST,
    .handler = heartbeat_post_handler,
};

static const httpd_uri_t s_start_uri = {
    .uri = "/api/start",
    .method = HTTP_POST,
    .handler = start_post_handler,
};

static const httpd_uri_t s_stop_uri = {
    .uri = "/api/stop",
    .method = HTTP_POST,
    .handler = stop_post_handler,
};

static esp_err_t start_wifi(void)
{
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "network interface init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop init failed");
    if (esp_netif_create_default_wifi_ap() == NULL) {
        return ESP_FAIL;
    }

    wifi_init_config_t wifi_init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wifi_init), TAG, "Wi-Fi init failed");

    uint8_t mac[6] = {0};
    ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP), TAG, "cannot read Wi-Fi MAC");
    char ssid[33] = {0};
    snprintf(ssid, sizeof(ssid), "RS02-BENCH-%02X%02X", mac[4], mac[5]);

    const size_t password_length = strlen(CONFIG_RS02_WEB_AP_PASSWORD);
    if (password_length < 8U || password_length > 63U) {
        ESP_LOGE(TAG, "web console AP password must contain 8 to 63 characters");
        return ESP_ERR_INVALID_ARG;
    }

    wifi_config_t wifi_config = {0};
    wifi_config.ap.ssid_len = strlen(ssid);
    memcpy(wifi_config.ap.ssid, ssid, wifi_config.ap.ssid_len);
    snprintf(
        (char *)wifi_config.ap.password,
        sizeof(wifi_config.ap.password),
        "%s",
        CONFIG_RS02_WEB_AP_PASSWORD);
    wifi_config.ap.channel = 1U;
    wifi_config.ap.max_connection = 1U;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.pmf_cfg.required = true;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "cannot select AP mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wifi_config), TAG, "AP config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "Wi-Fi start failed");

    ESP_LOGW(TAG, "web console AP: SSID=%s password=%s", ssid, CONFIG_RS02_WEB_AP_PASSWORD);
    ESP_LOGW(TAG, "open http://192.168.4.1 after connecting; network access is not an emergency stop");
    return ESP_OK;
}

static esp_err_t start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 8U;
    ESP_RETURN_ON_ERROR(httpd_start(&s_server, &config), TAG, "HTTP server start failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &s_root_uri), TAG, "root route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &s_status_uri), TAG, "status route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &s_config_uri), TAG, "config route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &s_start_uri), TAG, "start route failed");
    ESP_RETURN_ON_ERROR(
        httpd_register_uri_handler(s_server, &s_heartbeat_uri), TAG, "heartbeat route failed");
    ESP_RETURN_ON_ERROR(httpd_register_uri_handler(s_server, &s_stop_uri), TAG, "stop route failed");
    return ESP_OK;
}

esp_err_t rs02_web_console_start(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }
    s_context.config = rs02_motion_config_default();
    s_context.state = RS02_WEB_STATE_IDLE;
    s_context.target_rad = NAN;
    s_context.bus_voltage_v = NAN;
    s_context.mechanical_position_rad = NAN;
    s_context.motion_plan_current_angle_deg = NAN;
    s_context.motion_plan_target_angle_deg = NAN;
    s_context.motion_plan_signed_delta_deg = NAN;
    copy_text(s_context.phase, sizeof(s_context.phase), "idle");
    copy_text(s_context.message, sizeof(s_context.message),
              "Move a slider to check the plan automatically, then select Start Test. Reset never starts motion.");

    esp_err_t nvs_error = nvs_flash_init();
    if (nvs_error == ESP_ERR_NVS_NO_FREE_PAGES || nvs_error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "NVS erase failed");
        nvs_error = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(nvs_error, TAG, "NVS init failed");
    ESP_RETURN_ON_ERROR(start_wifi(), TAG, "web console Wi-Fi failed");
    return start_server();
}

void rs02_web_console_tick(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    expire_arm_if_needed_locked(monotonic_ms());
    xSemaphoreGive(s_lock);
}

bool rs02_web_console_take_prepare_request(rs02_motion_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    bool accepted = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_context.state == RS02_WEB_STATE_PREPARING && s_context.prepare_requested &&
        !s_context.stop_requested && heartbeat_is_fresh_locked(monotonic_ms())) {
        *config = s_context.config;
        s_context.prepare_requested = false;
        accepted = true;
    } else if (s_context.state == RS02_WEB_STATE_PREPARING && s_context.prepare_requested) {
        s_context.state = RS02_WEB_STATE_IDLE;
        s_context.prepare_requested = false;
        s_context.stop_requested = false;
        copy_text(s_context.phase, sizeof(s_context.phase), "idle");
        copy_text(s_context.message, sizeof(s_context.message),
                  "Motor preflight was cancelled or its browser heartbeat expired.");
    }
    xSemaphoreGive(s_lock);
    return accepted;
}

bool rs02_web_console_take_start_request(rs02_motion_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    bool accepted = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_context.state == RS02_WEB_STATE_STARTING && s_context.start_requested &&
        !s_context.stop_requested && heartbeat_is_fresh_locked(monotonic_ms())) {
        *config = s_context.config;
        s_context.start_requested = false;
        s_context.stop_requested = false;
        accepted = true;
    } else if (s_context.state == RS02_WEB_STATE_STARTING && s_context.start_requested) {
        s_context.state = RS02_WEB_STATE_IDLE;
        s_context.start_requested = false;
        s_context.stop_requested = false;
        copy_text(s_context.phase, sizeof(s_context.phase), "idle");
        copy_text(s_context.message, sizeof(s_context.message),
                  "Start request expired before the control task accepted it.");
    }
    xSemaphoreGive(s_lock);
    return accepted;
}

bool rs02_web_console_should_abort(void)
{
    bool should_abort = false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_context.state == RS02_WEB_STATE_PREPARING ||
        s_context.state == RS02_WEB_STATE_STARTING ||
        s_context.state == RS02_WEB_STATE_RUNNING) {
        should_abort = s_context.stop_requested || !heartbeat_is_fresh_locked(monotonic_ms());
        if (should_abort && !s_context.stop_requested) {
            copy_text(s_context.message, sizeof(s_context.message),
                      "Browser heartbeat lost. Stopping the motor.");
        }
    }
    xSemaphoreGive(s_lock);
    return should_abort;
}

void rs02_web_console_finish_preflight(bool accepted, const char *message)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool cancelled = s_context.stop_requested;
    s_context.prepare_requested = false;
    s_context.start_requested = false;
    s_context.stop_requested = false;
    if (accepted && !cancelled) {
        s_context.state = RS02_WEB_STATE_ARMED;
        s_context.armed_at_ms = monotonic_ms();
        s_context.heartbeat_at_ms = s_context.armed_at_ms;
        do {
            s_context.plan_token = esp_random();
        } while (s_context.plan_token == 0U);
        copy_text(s_context.phase, sizeof(s_context.phase), "awaiting-web-start");
    } else if (cancelled) {
        s_context.state = RS02_WEB_STATE_IDLE;
        s_context.plan_token = 0U;
        s_context.risk_ack_required = false;
        copy_text(s_context.phase, sizeof(s_context.phase), "idle");
        copy_text(s_context.message, sizeof(s_context.message), "Motor preflight was cancelled.");
    } else {
        s_context.state = RS02_WEB_STATE_FAULT;
        s_context.plan_token = 0U;
        s_context.risk_ack_required = false;
        copy_text(s_context.phase, sizeof(s_context.phase), "preflight-failed");
    }
    if (!cancelled && message != NULL) {
        copy_text(s_context.message, sizeof(s_context.message), message);
    }
    xSemaphoreGive(s_lock);
}

void rs02_web_console_set_state(rs02_web_state_t state, const char *message)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_context.state = state;
    s_context.stop_requested = false;
    s_context.prepare_requested = false;
    s_context.start_requested = false;
    s_context.plan_token = 0U;
    s_context.risk_ack_required = false;
    if (message != NULL) {
        copy_text(s_context.message, sizeof(s_context.message), message);
    }
    if (state == RS02_WEB_STATE_COMPLETE) {
        s_context.progress = 1.0f;
        copy_text(s_context.phase, sizeof(s_context.phase), "complete");
    } else if (state == RS02_WEB_STATE_FAULT) {
        copy_text(s_context.phase, sizeof(s_context.phase), "fault");
    }
    xSemaphoreGive(s_lock);
}

void rs02_web_console_update_feedback(const rs02_feedback_t *feedback)
{
    if (feedback == NULL) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_context.feedback = *feedback;
    s_context.has_feedback = true;
    xSemaphoreGive(s_lock);
}

void rs02_web_console_update_bus(float bus_voltage_v, float mechanical_position_rad)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_context.bus_voltage_v = bus_voltage_v;
    s_context.mechanical_position_rad = mechanical_position_rad;
    xSemaphoreGive(s_lock);
}

void rs02_web_console_update_motion_plan(
    float current_angle_deg,
    float target_angle_deg,
    float signed_delta_deg,
    const rs02_motion_config_t *resolved_config)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_context.has_motion_plan = true;
    s_context.motion_plan_current_angle_deg = current_angle_deg;
    s_context.motion_plan_target_angle_deg = target_angle_deg;
    s_context.motion_plan_signed_delta_deg = signed_delta_deg;
    if (resolved_config != NULL) {
        s_context.config = *resolved_config;
        s_context.risk_ack_required =
            resolved_config->torque_limit_nm > RS02_WEB_DEFAULT_TORQUE_LIMIT_NM ||
            resolved_config->speed_limit_rad_s > RS02_WEB_VALIDATED_SPEED_LIMIT_RAD_S;
    }
    xSemaphoreGive(s_lock);
}

void rs02_web_console_update_motion(const char *phase, float target_rad, float progress)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_context.state = RS02_WEB_STATE_RUNNING;
    s_context.target_rad = target_rad;
    s_context.progress = fminf(fmaxf(progress, 0.0f), 1.0f);
    copy_text(s_context.phase, sizeof(s_context.phase), phase);
    copy_text(s_context.message, sizeof(s_context.message),
              "Motor test active. Keep the physical power cutoff ready.");
    xSemaphoreGive(s_lock);
}
