/*
 * web_server.c  —  HTTP control interface
 *
 * CHANGES vs original:
 *  - All accident detection endpoints removed
 *  - auto_clean module integrated (replaces inline task)
 *  - /auto        → calls auto_clean_start()
 *  - /stop_auto   → calls auto_clean_stop()
 *  - /set_grid    → calls auto_clean_set_grid()
 *  - /get_grid    → calls auto_clean_get_grid()
 *  - /status      → includes auto_pct, strips_done, strips_total
 *  - /sensor_check kept for wiring verification
 *  - Removed front_dist / back_dist (never existed, kept from prior fix)
 */

#include "web_server.h"
#include "auto_clean.h"
#include "config.h"
#include "motor.h"
#include "ir_sensor.h"
#include "mpu6050.h"
#include "html_page.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "HTTP";
static httpd_handle_t s_server = NULL;

static bool s_tail_light = false;
bool web_server_tail_state(void) { return s_tail_light; }

/* ─── Query helper ────────────────────────────────────────── */
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

/* ─── Handlers ────────────────────────────────────────────── */

static esp_err_t handle_root(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, html_get_page(), HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_move(httpd_req_t *req)
{
    char dir[32] = {0};
    if (get_query_param(req, "dir", dir, sizeof(dir)) != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing dir");
        return ESP_OK;
    }
    if (auto_clean_is_running()) {
        return httpd_resp_sendstr(req, "BLOCKED:AUTO_RUNNING");
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

static esp_err_t handle_set_grid(httpd_req_t *req)
{
    clean_grid_t g;
    auto_clean_get_grid(&g);

    char buf[16];
    if (get_query_param(req, "cols", buf, sizeof(buf)) == 0) g.panel_cols     = atoi(buf);
    if (get_query_param(req, "rows", buf, sizeof(buf)) == 0) g.panel_rows     = atoi(buf);
    if (get_query_param(req, "pw",   buf, sizeof(buf)) == 0) g.panel_w_cm     = atoi(buf);
    if (get_query_param(req, "ph",   buf, sizeof(buf)) == 0) g.panel_h_cm     = atoi(buf);
    if (get_query_param(req, "gap",  buf, sizeof(buf)) == 0) g.gap_between_cm = atoi(buf);
    if (get_query_param(req, "wash", buf, sizeof(buf)) == 0) g.wash_enabled   = (atoi(buf) == 1);
    if (get_query_param(req, "blow", buf, sizeof(buf)) == 0) g.blow_enabled   = (atoi(buf) == 1);

    auto_clean_set_grid(&g);
    auto_clean_get_grid(&g);   /* read back clamped values */

    int strips = (g.panel_w_cm + ROVER_WIDTH_CM - 1) / ROVER_WIDTH_CM;
    int total  = strips * g.panel_cols * g.panel_rows;

    char resp[256];
    snprintf(resp, sizeof(resp),
             "{\"ok\":1,\"cols\":%d,\"rows\":%d,\"pw\":%d,\"ph\":%d"
             ",\"gap\":%d,\"wash\":%d,\"blow\":%d"
             ",\"strips_per_panel\":%d,\"total_strips\":%d"
             ",\"rover_w\":%d,\"rover_d\":%d}",
             g.panel_cols, g.panel_rows,
             g.panel_w_cm, g.panel_h_cm, g.gap_between_cm,
             g.wash_enabled ? 1 : 0, g.blow_enabled ? 1 : 0,
             strips, total,
             ROVER_WIDTH_CM, ROVER_DEPTH_CM);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t handle_get_grid(httpd_req_t *req)
{
    clean_grid_t g;
    auto_clean_get_grid(&g);

    int strips = (g.panel_w_cm + ROVER_WIDTH_CM - 1) / ROVER_WIDTH_CM;
    int total  = strips * g.panel_cols * g.panel_rows;

    char resp[256];
    snprintf(resp, sizeof(resp),
             "{\"cols\":%d,\"rows\":%d,\"pw\":%d,\"ph\":%d"
             ",\"gap\":%d,\"wash\":%d,\"blow\":%d"
             ",\"strips_per_panel\":%d,\"total_strips\":%d"
             ",\"rover_w\":%d,\"rover_d\":%d}",
             g.panel_cols, g.panel_rows,
             g.panel_w_cm, g.panel_h_cm, g.gap_between_cm,
             g.wash_enabled ? 1 : 0, g.blow_enabled ? 1 : 0,
             strips, total,
             ROVER_WIDTH_CM, ROVER_DEPTH_CM);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t handle_status(httpd_req_t *req)
{
    ir_status_t ir  = ir_get_status();
    mpu_data_t  mpu = mpu_get();

    char json[900];
    snprintf(json, sizeof(json),
             "{"
             "\"direction\":\"%s\","
             "\"tail\":%d,"
             /* IR */
             "\"ir_fl\":%d,\"ir_fr\":%d,\"ir_bl\":%d,\"ir_br\":%d,"
             "\"front_blocked\":%d,\"back_blocked\":%d,"
             /* MPU */
             "\"mpu_ready\":%d,"
             "\"vib\":%.3f,"
             "\"heading\":%.1f,"
             "\"mpu_temp\":%d,"
             "\"accel_x\":%.3f,\"accel_y\":%.3f,\"accel_z\":%.3f,"
             "\"gyro_z\":%.2f,"
             /* peripherals */
             "\"ramming\":%d,\"pump\":%d,\"blower\":%d,"
             /* auto clean progress */
             "\"auto_running\":%d,"
             "\"auto_pct\":%d,"
             "\"auto_strips_done\":%d,"
             "\"auto_strips_total\":%d,"
             "\"auto_state\":\"%s\""
             "}",
             motor_dir_to_str(motor_get_direction()),
             (int)(s_tail_light ? 1 : 0),
             (int)(ir.fl ? 1 : 0), (int)(ir.fr ? 1 : 0),
             (int)(ir.bl ? 1 : 0), (int)(ir.br ? 1 : 0),
             (int)(ir.front_blocked ? 1 : 0),
             (int)(ir.back_blocked  ? 1 : 0),
             (int)(mpu_is_ready()  ? 1 : 0),
             (double)mpu.vib_mag,
             (double)mpu.heading_deg,
             mpu.temp_c,
             (double)mpu.accel_x,
             (double)mpu.accel_y,
             (double)mpu.accel_z,
             (double)mpu.gyro_z,
             (int)(motor_get_ramming() ? 1 : 0),
             (int)(pump_get()    ? 1 : 0),
             (int)(blower_get()  ? 1 : 0),
             (int)(auto_clean_is_running() ? 1 : 0),
             auto_clean_pct(),
             auto_clean_strips_done(),
             auto_clean_strips_total(),
             auto_clean_state() == CLEAN_IDLE    ? "IDLE"    :
             auto_clean_state() == CLEAN_RUNNING ? "RUNNING" :
             auto_clean_state() == CLEAN_DONE    ? "DONE"    : "ERROR");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t handle_sensor_check(httpd_req_t *req)
{
    ir_status_t ir = ir_get_status();
    mpu_data_t  m  = mpu_get();
    bool mpu_ok    = mpu_is_ready();

    char json[512];
    snprintf(json, sizeof(json),
             "{"
             "\"ir_enabled\":%d,"
             "\"ir_fl\":{\"pin\":%d,\"blocked\":%d},"
             "\"ir_fr\":{\"pin\":%d,\"blocked\":%d},"
             "\"ir_bl\":{\"pin\":%d,\"blocked\":%d},"
             "\"ir_br\":{\"pin\":%d,\"blocked\":%d},"
             "\"mpu_enabled\":%d,"
             "\"mpu_ready\":%d,"
             "\"mpu_temp_c\":%d,"
             "\"mpu_accel_x\":%.3f,"
             "\"mpu_accel_y\":%.3f,"
             "\"mpu_accel_z\":%.3f,"
             "\"mpu_vib\":%.3f,"
             "\"rover_w_cm\":%d,"
             "\"rover_d_cm\":%d"
             "}",
             IR_ENABLED,
             (int)PIN_IR_FRONT_LEFT,  (int)(ir.fl ? 1 : 0),
             (int)PIN_IR_FRONT_RIGHT, (int)(ir.fr ? 1 : 0),
             (int)PIN_IR_BACK_LEFT,   (int)(ir.bl ? 1 : 0),
             (int)PIN_IR_BACK_RIGHT,  (int)(ir.br ? 1 : 0),
             MPU_ENABLED,
             (int)(mpu_ok ? 1 : 0),
             m.temp_c,
             (double)m.accel_x,
             (double)m.accel_y,
             (double)m.accel_z,
             (double)m.vib_mag,
             ROVER_WIDTH_CM,
             ROVER_DEPTH_CM);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t handle_auto(httpd_req_t *req)
{
    if (auto_clean_is_running())
        return httpd_resp_sendstr(req, "ALREADY_RUNNING");

    if (auto_clean_start())
        return httpd_resp_sendstr(req, "AUTO_STARTED");
    else
        return httpd_resp_sendstr(req, "ERROR:TASK_CREATE_FAILED");
}

static esp_err_t handle_stop_auto(httpd_req_t *req)
{
    auto_clean_stop();
    return httpd_resp_sendstr(req, "STOPPED");
}

static esp_err_t handle_404(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
}

/* ─── Server start / stop ─────────────────────────────────── */

esp_err_t web_server_start(void)
{
    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = HTTP_SERVER_PORT;
    cfg.max_uri_handlers  = 16;
    cfg.lru_purge_enable  = true;
    cfg.recv_wait_timeout = 5;
    cfg.send_wait_timeout = 5;

    esp_err_t ret = httpd_start(&s_server, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    static const httpd_uri_t uris[] = {
        {.uri = "/",             .method = HTTP_GET, .handler = handle_root},
        {.uri = "/move",         .method = HTTP_GET, .handler = handle_move},
        {.uri = "/tail",         .method = HTTP_GET, .handler = handle_tail},
        {.uri = "/pump",         .method = HTTP_GET, .handler = handle_pump},
        {.uri = "/blower",       .method = HTTP_GET, .handler = handle_blower},
        {.uri = "/ramming",      .method = HTTP_GET, .handler = handle_ramming},
        {.uri = "/set_grid",     .method = HTTP_GET, .handler = handle_set_grid},
        {.uri = "/get_grid",     .method = HTTP_GET, .handler = handle_get_grid},
        {.uri = "/status",       .method = HTTP_GET, .handler = handle_status},
        {.uri = "/sensor_check", .method = HTTP_GET, .handler = handle_sensor_check},
        {.uri = "/auto",         .method = HTTP_GET, .handler = handle_auto},
        {.uri = "/stop_auto",    .method = HTTP_GET, .handler = handle_stop_auto},
    };
    const int n = (int)(sizeof(uris) / sizeof(uris[0]));
    for (int i = 0; i < n; i++)
        httpd_register_uri_handler(s_server, &uris[i]);

    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, handle_404);
    ESP_LOGI(TAG, "HTTP server ready — %d routes on port %d", n, HTTP_SERVER_PORT);
    return ESP_OK;
}

void web_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}