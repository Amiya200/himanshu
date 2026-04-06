#include "web_server.h"
#include "config.h"
#include "motor.h"
#include "ultrasonic.h"
#include "html_page.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "HTTP";

static httpd_handle_t s_server = NULL;

// ================== TAIL LIGHT STATE ==================
static bool s_tail_light = false;
static int s_last_vib = 0;

void web_server_set_vib(int v) { s_last_vib = v; }
bool web_server_tail_state(void) { return s_tail_light; }

// ================== HELPER: get query param ==================

static int get_query_param(httpd_req_t *req, const char *key,
                           char *buf, size_t buf_sz)
{
    char query[256] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)
        return -1;
    if (httpd_query_key_value(query, key, buf, buf_sz) != ESP_OK)
        return -1;
    return 0;
}

// ================== / ==================

static esp_err_t handle_root(httpd_req_t *req)
{
    ESP_LOGI(TAG, "GET /");
    httpd_resp_set_type(req, "text/html");
    // Disable caching so the page always reflects current firmware
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, html_get_page(), HTTPD_RESP_USE_STRLEN);
}

// ================== /move ==================

static esp_err_t handle_move(httpd_req_t *req)
{
    char dir[32] = {0};
    if (get_query_param(req, "dir", dir, sizeof(dir)) != 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing dir");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "/move dir=%s", dir);

    // Accident blocks everything except stop
    if (motor_get_accident() && strcmp(dir, "stop") != 0)
    {
        motor_stop_immediate();
        return httpd_resp_sendstr(req, "BLOCKED:ACCIDENT");
    }

    motor_dir_t cmd = DIR_STOP;
    if (strcmp(dir, "forward") == 0)
        cmd = DIR_FORWARD;
    else if (strcmp(dir, "backward") == 0)
        cmd = DIR_BACKWARD;
    else if (strcmp(dir, "left") == 0)
        cmd = DIR_LEFT;
    else if (strcmp(dir, "right") == 0)
        cmd = DIR_RIGHT;
    else if (strcmp(dir, "drift_left") == 0)
        cmd = DIR_DRIFT_LEFT;
    else if (strcmp(dir, "drift_right") == 0)
        cmd = DIR_DRIFT_RIGHT;
    else if (strcmp(dir, "stop") == 0)
        cmd = DIR_STOP;
    else
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown dir");
        return ESP_OK;
    }

    motor_send_cmd(cmd);
    return httpd_resp_sendstr(req, "OK");
}

// ================== /tail ==================

static esp_err_t handle_tail(httpd_req_t *req)
{
    char state[8] = {0};
    if (get_query_param(req, "state", state, sizeof(state)) != 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing state");
        return ESP_OK;
    }
    if (strcmp(state, "on") == 0)
    {
        s_tail_light = true;
        gpio_set_level(PIN_TAIL_LIGHT, 1);
        ESP_LOGI(TAG, "Tail light ON");
        return httpd_resp_sendstr(req, "Tail ON");
    }
    else if (strcmp(state, "off") == 0)
    {
        s_tail_light = false;
        gpio_set_level(PIN_TAIL_LIGHT, 0);
        ESP_LOGI(TAG, "Tail light OFF");
        return httpd_resp_sendstr(req, "Tail OFF");
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown state");
    return ESP_OK;
}

// ================== /ramming ==================

static esp_err_t handle_ramming(httpd_req_t *req)
{
    char state[8] = {0};
    if (get_query_param(req, "state", state, sizeof(state)) != 0)
    {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing state");
        return ESP_OK;
    }
    motor_set_ramming(strcmp(state, "on") == 0);
    return httpd_resp_sendstr(req, "OK");
}

// ================== /reset_accident ==================

static esp_err_t handle_reset_accident(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/reset_accident");
    motor_set_accident(false);
    s_tail_light = false;
    gpio_set_level(PIN_TAIL_LIGHT, 0);
    return httpd_resp_sendstr(req, "Cleared");
}

// ================== /status ==================
static void auto_square_task(void *arg); // forward declaration

static esp_err_t handle_auto(httpd_req_t *req)
{
    ESP_LOGI(TAG, "/auto triggered");

    if (motor_get_accident())
    {
        return httpd_resp_sendstr(req, "BLOCKED:ACCIDENT");
    }

    xTaskCreate(auto_square_task, "auto_task", 4096, NULL, 5, NULL);

    return httpd_resp_sendstr(req, "AUTO STARTED");
}
static esp_err_t handle_status(httpd_req_t *req)
{
    long fd, bd;
    ultrasonic_get_distances(&fd, &bd);

    char json[256];
    snprintf(json, sizeof(json),
             "{"
             "\"direction\":\"%s\","
             "\"tail\":%d,"
             "\"accident\":%d,"
             "\"vib\":%d,"
             "\"front_dist\":%ld,"
             "\"back_dist\":%ld,"
             "\"front_blocked\":%d,"
             "\"back_blocked\":%d,"
             "\"ramming\":%d"
             "}",
             motor_dir_to_str(motor_get_direction()),
             s_tail_light ? 1 : 0,
             motor_get_accident() ? 1 : 0,
             s_last_vib,
             fd, bd,
             ultrasonic_is_front_blocked() ? 1 : 0,
             ultrasonic_is_back_blocked() ? 1 : 0,
             motor_get_ramming() ? 1 : 0);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

// ================== 404 ==================

static esp_err_t handle_404(httpd_req_t *req, httpd_err_code_t err)
{
    ESP_LOGW(TAG, "404 -> %s", req->uri);
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
}

// ================== START / STOP ==================
// ================== AUTO MOVEMENT TASK ==================

#define MOVE_1M_TIME 900
#define TURN_TIME 400
#define STEP_TIME 200

static void auto_square_task(void *arg)
{
    ESP_LOGI(TAG, "AUTO MODE STARTED");

    int strips = 5;

    for (int i = 0; i < strips; i++)
    {
        // Safety check
        if (motor_get_accident())
        {
            ESP_LOGW(TAG, "AUTO STOPPED: ACCIDENT");
            break;
        }

        // Snake motion
        if (i % 2 == 0)
            motor_move_blocking(DIR_FORWARD, MOVE_1M_TIME);
        else
            motor_move_blocking(DIR_BACKWARD, MOVE_1M_TIME);

        if (i == strips - 1)
            break;

        // Shift lane
        motor_move_blocking(DIR_RIGHT, TURN_TIME);
        motor_move_blocking(DIR_FORWARD, STEP_TIME);
        motor_move_blocking(DIR_RIGHT, TURN_TIME);
    }

    motor_send_cmd(DIR_STOP);

    ESP_LOGI(TAG, "AUTO MODE COMPLETE");

    vTaskDelete(NULL);
}
esp_err_t web_server_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = HTTP_SERVER_PORT;
    cfg.max_uri_handlers = 10;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 3;
    cfg.send_wait_timeout = 3;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", cfg.server_port);
    esp_err_t ret = httpd_start(&s_server, &cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start server: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = handle_root},
        {.uri = "/move", .method = HTTP_GET, .handler = handle_move},
        {.uri = "/tail", .method = HTTP_GET, .handler = handle_tail},
        {.uri = "/ramming", .method = HTTP_GET, .handler = handle_ramming},
        {.uri = "/reset_accident", .method = HTTP_GET, .handler = handle_reset_accident},
        {.uri = "/status", .method = HTTP_GET, .handler = handle_status},
        {.uri = "/auto", .method = HTTP_GET, .handler = handle_auto}, // ✅ ADD THIS
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
    {
        httpd_register_uri_handler(s_server, &uris[i]);
    }
    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, handle_404);

    ESP_LOGI(TAG, "HTTP server started — all routes registered");
    return ESP_OK;
}

void web_server_stop(void)
{
    if (s_server)
    {
        httpd_stop(s_server);
        s_server = NULL;
    }
}