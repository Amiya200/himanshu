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
#include <stdlib.h>

static const char *TAG = "HTTP";
static httpd_handle_t s_server = NULL;

// ================== STATE ==================
static bool s_tail_light = false;
static int  s_last_vib   = 0;

void web_server_set_vib(int v) { s_last_vib = v; }
bool web_server_tail_state(void) { return s_tail_light; }

// ================== GRID CONFIG ==================
typedef struct {
    int  cols;
    int  rows;
    int  panel_w_cm;
    int  panel_h_cm;
    int  gap_cm;
    bool wash_enabled;
    bool blow_enabled;
} grid_config_t;

static grid_config_t s_grid = {
    .cols          = 2,
    .rows          = 2,
    .panel_w_cm    = 100,
    .panel_h_cm    = 170,
    .gap_cm        = 5,
    .wash_enabled  = true,
    .blow_enabled  = false,
};

// ================== HELPER ==================

static int get_query_param(httpd_req_t *req, const char *key,
                            char *buf, size_t buf_sz)
{
    char query[512] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)
        return -1;
    if (httpd_query_key_value(query, key, buf, buf_sz) != ESP_OK)
        return -1;
    return 0;
}

// ================== / ==================

static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, html_get_page(), HTTPD_RESP_USE_STRLEN);
}

// ================== /move ==================

static esp_err_t handle_move(httpd_req_t *req)
{
    char dir[32] = {0};
    if (get_query_param(req, "dir", dir, sizeof(dir)) != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing dir");
        return ESP_OK;
    }
    if (motor_get_accident() && strcmp(dir, "stop") != 0) {
        motor_stop_immediate();
        return httpd_resp_sendstr(req, "BLOCKED:ACCIDENT");
    }
    motor_dir_t cmd = DIR_STOP;
    if      (strcmp(dir, "forward")     == 0) cmd = DIR_FORWARD;
    else if (strcmp(dir, "backward")    == 0) cmd = DIR_BACKWARD;
    else if (strcmp(dir, "left")        == 0) cmd = DIR_LEFT;
    else if (strcmp(dir, "right")       == 0) cmd = DIR_RIGHT;
    else if (strcmp(dir, "drift_left")  == 0) cmd = DIR_DRIFT_LEFT;
    else if (strcmp(dir, "drift_right") == 0) cmd = DIR_DRIFT_RIGHT;
    else if (strcmp(dir, "stop")        == 0) cmd = DIR_STOP;
    else {
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
    if (get_query_param(req, "state", state, sizeof(state)) != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing state");
        return ESP_OK;
    }
    bool on = (strcmp(state, "on") == 0);
    s_tail_light = on;
    gpio_set_level(PIN_TAIL_LIGHT, on ? 1 : 0);
    return httpd_resp_sendstr(req, on ? "Tail ON" : "Tail OFF");
}

// ================== /pump ==================

static esp_err_t handle_pump(httpd_req_t *req)
{
    char state[8] = {0};
    if (get_query_param(req, "state", state, sizeof(state)) != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing state");
        return ESP_OK;
    }
    pump_set(strcmp(state, "on") == 0);
    return httpd_resp_sendstr(req, pump_get() ? "Pump ON" : "Pump OFF");
}

// ================== /blower ==================

static esp_err_t handle_blower(httpd_req_t *req)
{
    char state[8] = {0};
    if (get_query_param(req, "state", state, sizeof(state)) != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing state");
        return ESP_OK;
    }
    blower_set(strcmp(state, "on") == 0);
    return httpd_resp_sendstr(req, blower_get() ? "Blower ON" : "Blower OFF");
}

// ================== /ramming ==================

static esp_err_t handle_ramming(httpd_req_t *req)
{
    char state[8] = {0};
    if (get_query_param(req, "state", state, sizeof(state)) != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing state");
        return ESP_OK;
    }
    motor_set_ramming(strcmp(state, "on") == 0);
    return httpd_resp_sendstr(req, "OK");
}

// ================== /reset_accident ==================

static esp_err_t handle_reset_accident(httpd_req_t *req)
{
    motor_set_accident(false);
    s_tail_light = false;
    gpio_set_level(PIN_TAIL_LIGHT, 0);
    return httpd_resp_sendstr(req, "Cleared");
}

// ================== /set_grid ==================

static esp_err_t handle_set_grid(httpd_req_t *req)
{
    char buf[16];
    if (get_query_param(req, "cols", buf, sizeof(buf)) == 0) s_grid.cols         = atoi(buf);
    if (get_query_param(req, "rows", buf, sizeof(buf)) == 0) s_grid.rows         = atoi(buf);
    if (get_query_param(req, "pw",   buf, sizeof(buf)) == 0) s_grid.panel_w_cm   = atoi(buf);
    if (get_query_param(req, "ph",   buf, sizeof(buf)) == 0) s_grid.panel_h_cm   = atoi(buf);
    if (get_query_param(req, "gap",  buf, sizeof(buf)) == 0) s_grid.gap_cm       = atoi(buf);
    if (get_query_param(req, "wash", buf, sizeof(buf)) == 0) s_grid.wash_enabled = (atoi(buf) == 1);
    if (get_query_param(req, "blow", buf, sizeof(buf)) == 0) s_grid.blow_enabled = (atoi(buf) == 1);

    if (s_grid.cols < 1)  s_grid.cols = 1;
    if (s_grid.rows < 1)  s_grid.rows = 1;
    if (s_grid.cols > 10) s_grid.cols = 10;
    if (s_grid.rows > 10) s_grid.rows = 10;
    if (s_grid.panel_w_cm < 10) s_grid.panel_w_cm = 10;
    if (s_grid.panel_h_cm < 10) s_grid.panel_h_cm = 10;
    if (s_grid.gap_cm     < 0)  s_grid.gap_cm     = 0;

    ESP_LOGI(TAG, "Grid: %dx%d panel=%dx%dcm gap=%dcm wash=%d blow=%d",
             s_grid.cols, s_grid.rows,
             s_grid.panel_w_cm, s_grid.panel_h_cm, s_grid.gap_cm,
             s_grid.wash_enabled, s_grid.blow_enabled);

    char resp[128];
    snprintf(resp, sizeof(resp),
             "{\"ok\":1,\"cols\":%d,\"rows\":%d,\"pw\":%d,\"ph\":%d,\"gap\":%d,\"wash\":%d,\"blow\":%d}",
             s_grid.cols, s_grid.rows,
             s_grid.panel_w_cm, s_grid.panel_h_cm, s_grid.gap_cm,
             s_grid.wash_enabled ? 1 : 0, s_grid.blow_enabled ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

// ================== /get_grid ==================

static esp_err_t handle_get_grid(httpd_req_t *req)
{
    char resp[128];
    snprintf(resp, sizeof(resp),
             "{\"cols\":%d,\"rows\":%d,\"pw\":%d,\"ph\":%d,\"gap\":%d,\"wash\":%d,\"blow\":%d}",
             s_grid.cols, s_grid.rows,
             s_grid.panel_w_cm, s_grid.panel_h_cm, s_grid.gap_cm,
             s_grid.wash_enabled ? 1 : 0, s_grid.blow_enabled ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, resp);
}

// ================== /status ==================

static esp_err_t handle_status(httpd_req_t *req)
{
    // ultrasonic_get_distances always works:
    //   ULTRASONIC_ENABLED=0 → returns 999, 999 (no obstacle)
    //   ULTRASONIC_ENABLED=1 → returns real sensor values
    long fd, bd;
    ultrasonic_get_distances(&fd, &bd);

    char json[320];
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
             "\"ramming\":%d,"
             "\"pump\":%d,"
             "\"blower\":%d"
             "}",
             motor_dir_to_str(motor_get_direction()),
             s_tail_light ? 1 : 0,
             motor_get_accident() ? 1 : 0,
             s_last_vib,
             fd, bd,
             ultrasonic_is_front_blocked() ? 1 : 0,
             ultrasonic_is_back_blocked()  ? 1 : 0,
             motor_get_ramming() ? 1 : 0,
             pump_get()          ? 1 : 0,
             blower_get()        ? 1 : 0);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

// ================== 404 ==================

static esp_err_t handle_404(httpd_req_t *req, httpd_err_code_t err)
{
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
}

// ================== AUTO CLEAN TASK ==================

static void auto_clean_task(void *arg)
{
    ESP_LOGI(TAG, "AUTO CLEAN started  grid=%dx%d  panel=%dx%dcm  gap=%dcm",
             s_grid.cols, s_grid.rows,
             s_grid.panel_w_cm, s_grid.panel_h_cm, s_grid.gap_cm);

    const int strip_width_cm = 10;
    int passes = (s_grid.panel_h_cm + strip_width_cm - 1) / strip_width_cm;

    if (s_grid.wash_enabled) pump_set(true);
    if (s_grid.blow_enabled) blower_set(true);

    for (int col = 0; col < s_grid.cols; col++) {
        if (motor_get_accident()) break;

        ESP_LOGI(TAG, "Cleaning column %d/%d", col + 1, s_grid.cols);

        for (int pass = 0; pass < passes; pass++) {
            if (motor_get_accident()) goto done;

            motor_dir_t dir = (pass % 2 == 0) ? DIR_FORWARD : DIR_BACKWARD;
            motor_move_blocking(dir, s_grid.panel_w_cm * MS_PER_CM);

            if (pass < passes - 1)
                motor_move_blocking(DIR_RIGHT, strip_width_cm * MS_PER_CM);
        }

        // Return lateral position to start of column
        {
            int return_ms = (passes - 1) * strip_width_cm * MS_PER_CM;
            if (return_ms > 0)
                motor_move_blocking(DIR_LEFT, return_ms);
        }

        // Advance to next panel column
        if (col < s_grid.cols - 1)
            motor_move_blocking(DIR_FORWARD,
                                (s_grid.panel_w_cm + s_grid.gap_cm) * MS_PER_CM);
    }

done:
    pump_set(false);
    blower_set(false);
    motor_send_cmd(DIR_STOP);
    ESP_LOGI(TAG, "AUTO CLEAN complete");
    vTaskDelete(NULL);
}

static esp_err_t handle_auto(httpd_req_t *req)
{
    if (motor_get_accident())
        return httpd_resp_sendstr(req, "BLOCKED:ACCIDENT");

    xTaskCreate(auto_clean_task, "auto_clean", 4096, NULL, 5, NULL);
    return httpd_resp_sendstr(req, "AUTO STARTED");
}

// ================== START / STOP ==================

esp_err_t web_server_start(void)
{
    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = HTTP_SERVER_PORT;
    cfg.max_uri_handlers  = 16;
    cfg.lru_purge_enable  = true;
    cfg.recv_wait_timeout = 3;
    cfg.send_wait_timeout = 3;

    ESP_LOGI(TAG, "Starting HTTP server on port %d", cfg.server_port);
    esp_err_t ret = httpd_start(&s_server, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start: %s", esp_err_to_name(ret));
        return ret;
    }

    httpd_uri_t uris[] = {
        {.uri = "/",              .method = HTTP_GET, .handler = handle_root},
        {.uri = "/move",          .method = HTTP_GET, .handler = handle_move},
        {.uri = "/tail",          .method = HTTP_GET, .handler = handle_tail},
        {.uri = "/pump",          .method = HTTP_GET, .handler = handle_pump},
        {.uri = "/blower",        .method = HTTP_GET, .handler = handle_blower},
        {.uri = "/ramming",       .method = HTTP_GET, .handler = handle_ramming},
        {.uri = "/reset_accident",.method = HTTP_GET, .handler = handle_reset_accident},
        {.uri = "/set_grid",      .method = HTTP_GET, .handler = handle_set_grid},
        {.uri = "/get_grid",      .method = HTTP_GET, .handler = handle_get_grid},
        {.uri = "/status",        .method = HTTP_GET, .handler = handle_status},
        {.uri = "/auto",          .method = HTTP_GET, .handler = handle_auto},
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        httpd_register_uri_handler(s_server, &uris[i]);

    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, handle_404);
    ESP_LOGI(TAG, "HTTP server started — all routes registered");
    return ESP_OK;
}

void web_server_stop(void)
{
    if (s_server) { httpd_stop(s_server); s_server = NULL; }
}