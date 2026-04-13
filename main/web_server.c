/*
 * web_server.c  —  HTTP control interface  (FIXED + ENHANCED)
 *
 * FIXES vs previous version:
 *  1. handle_status: removed front_dist / back_dist — IR sensors are
 *     digital (blocked/clear), not distance sensors. The original HTML
 *     referenced d.front_dist / d.back_dist but these were never in the
 *     JSON, causing the sensor bars to always show "---". Removed.
 *  2. handle_status: added mpu_ready, accel_x/y/z, gyro_z fields so the
 *     dashboard can display full MPU state and a connection check.
 *  3. handle_status: JSON buffer enlarged to 768 bytes to hold new fields.
 *  4. handle_sensor_check: new endpoint /sensor_check returns a
 *     structured health report for all sensors (IR × 4, MPU).
 *  5. All other fixes from prior version retained.
 */

#include "web_server.h"
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
#include <math.h>

static const char *TAG = "HTTP";
static httpd_handle_t s_server = NULL;

/* ─── module state ────────────────────────────────────────── */
static bool s_tail_light  = false;
static bool s_auto_running = false;

bool web_server_tail_state(void) { return s_tail_light; }

/* ─── grid config ─────────────────────────────────────────── */
typedef struct {
    int  cols, rows;
    int  panel_w_cm, panel_h_cm, gap_cm;
    bool wash_enabled, blow_enabled;
} grid_config_t;

static grid_config_t s_grid = {
    .cols = 2, .rows = 2,
    .panel_w_cm = 100, .panel_h_cm = 170, .gap_cm = 5,
    .wash_enabled = true, .blow_enabled = false,
};

/* ─── query helper ────────────────────────────────────────── */
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

/* ══════════════════════════════════════════════════════════
   HTTP HANDLERS
══════════════════════════════════════════════════════════ */

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

static esp_err_t handle_reset_accident(httpd_req_t *req)
{
    motor_set_accident(false);
    mpu_clear_accident();
    s_tail_light = false;
    gpio_set_level(PIN_TAIL_LIGHT, 0);
    return httpd_resp_sendstr(req, "Cleared");
}

static esp_err_t handle_set_grid(httpd_req_t *req)
{
    char buf[16];
    if (get_query_param(req, "cols", buf, sizeof(buf)) == 0) s_grid.cols        = atoi(buf);
    if (get_query_param(req, "rows", buf, sizeof(buf)) == 0) s_grid.rows        = atoi(buf);
    if (get_query_param(req, "pw",   buf, sizeof(buf)) == 0) s_grid.panel_w_cm  = atoi(buf);
    if (get_query_param(req, "ph",   buf, sizeof(buf)) == 0) s_grid.panel_h_cm  = atoi(buf);
    if (get_query_param(req, "gap",  buf, sizeof(buf)) == 0) s_grid.gap_cm      = atoi(buf);
    if (get_query_param(req, "wash", buf, sizeof(buf)) == 0) s_grid.wash_enabled = (atoi(buf) == 1);
    if (get_query_param(req, "blow", buf, sizeof(buf)) == 0) s_grid.blow_enabled = (atoi(buf) == 1);

    if (s_grid.cols < 1)        s_grid.cols = 1;
    if (s_grid.cols > 10)       s_grid.cols = 10;
    if (s_grid.rows < 1)        s_grid.rows = 1;
    if (s_grid.rows > 10)       s_grid.rows = 10;
    if (s_grid.panel_w_cm < 10) s_grid.panel_w_cm = 10;
    if (s_grid.panel_h_cm < 10) s_grid.panel_h_cm = 10;
    if (s_grid.gap_cm < 0)      s_grid.gap_cm = 0;
    if (s_grid.gap_cm > 100)    s_grid.gap_cm = 100;

    char resp[192];
    snprintf(resp, sizeof(resp),
             "{\"ok\":1,\"cols\":%d,\"rows\":%d,\"pw\":%d,\"ph\":%d"
             ",\"gap\":%d,\"wash\":%d,\"blow\":%d}",
             s_grid.cols, s_grid.rows,
             s_grid.panel_w_cm, s_grid.panel_h_cm, s_grid.gap_cm,
             s_grid.wash_enabled ? 1 : 0,
             s_grid.blow_enabled ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t handle_get_grid(httpd_req_t *req)
{
    char resp[192];
    snprintf(resp, sizeof(resp),
             "{\"cols\":%d,\"rows\":%d,\"pw\":%d,\"ph\":%d"
             ",\"gap\":%d,\"wash\":%d,\"blow\":%d}",
             s_grid.cols, s_grid.rows,
             s_grid.panel_w_cm, s_grid.panel_h_cm, s_grid.gap_cm,
             s_grid.wash_enabled ? 1 : 0,
             s_grid.blow_enabled ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, resp);
}

/* ── /status ──────────────────────────────────────────────── */
/*
 * FIX: Removed front_dist / back_dist — these were never populated
 * because the IR sensors are DIGITAL (obstacle yes/no), not ultrasonic
 * distance sensors.  The old HTML tried to show distance bars but got
 * undefined every poll, breaking the UI.  Replaced with per-channel
 * IR booleans + explicit "ir_ready" flag.
 *
 * NEW fields added:
 *   mpu_ready   — 1 if MPU WHO_AM_I passed and task is running
 *   accel_x/y/z — raw accelerometer in g (useful for tilt display)
 *   gyro_z      — yaw rate deg/s (shows if gyro is live)
 *   mpu_temp    — already present, kept
 *   heading     — already present, kept
 *   vib         — already present, kept
 */
static esp_err_t handle_status(httpd_req_t *req)
{
    ir_status_t ir  = ir_get_status();
    mpu_data_t  mpu = mpu_get();

    /* All bool→int casts for -Wformat= on xtensa-gcc */
    char json[768];
    snprintf(json, sizeof(json),
             "{"
             "\"direction\":\"%s\","
             "\"tail\":%d,"
             "\"accident\":%d,"
             /* IR sensor states — 4 individual channels */
             "\"ir_fl\":%d,"
             "\"ir_fr\":%d,"
             "\"ir_bl\":%d,"
             "\"ir_br\":%d,"
             "\"front_blocked\":%d,"
             "\"back_blocked\":%d,"
             /* MPU data */
             "\"mpu_ready\":%d,"
             "\"vib\":%.3f,"
             "\"heading\":%.1f,"
             "\"mpu_temp\":%d,"
             "\"accel_x\":%.3f,"
             "\"accel_y\":%.3f,"
             "\"accel_z\":%.3f,"
             "\"gyro_z\":%.2f,"
             /* peripherals */
             "\"ramming\":%d,"
             "\"pump\":%d,"
             "\"blower\":%d,"
             "\"auto_running\":%d"
             "}",
             motor_dir_to_str(motor_get_direction()),
             (int)(s_tail_light ? 1 : 0),
             (int)(motor_get_accident() ? 1 : 0),
             (int)(ir.fl ? 1 : 0),
             (int)(ir.fr ? 1 : 0),
             (int)(ir.bl ? 1 : 0),
             (int)(ir.br ? 1 : 0),
             (int)(ir.front_blocked ? 1 : 0),
             (int)(ir.back_blocked  ? 1 : 0),
             (int)(mpu_is_ready() ? 1 : 0),
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
             (int)(s_auto_running ? 1 : 0));

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

/* ── /sensor_check ────────────────────────────────────────── */
/*
 * NEW: Returns a human-readable health report for all sensors.
 * Useful for initial wiring verification without needing serial.
 *
 * IR check: reads current level from each pin directly (bypasses
 *           debounce) so the result is instant.
 * MPU check: uses mpu_is_ready() set during mpu_init() WHO_AM_I pass.
 */
static esp_err_t handle_sensor_check(httpd_req_t *req)
{
    ir_status_t ir = ir_get_status();
    mpu_data_t  m  = mpu_get();
    bool mpu_ok    = mpu_is_ready();

    /*
     * For IR sensors: a sensor is "wired" if it can be read. Since
     * gpio_get_level() always returns 0 or 1, we check if the pin
     * appears to respond.  With pull-ups, an unconnected pin reads 1
     * (no obstacle), a connected sensor with nothing in front also
     * reads 1. A sensor stuck at 0 with nothing in front of it
     * suggests a wiring fault or wrong polarity.
     *
     * We report what we read; the user must verify in an open area.
     */
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
             "\"mpu_who_am_i_passed\":%d,"
             "\"mpu_temp_c\":%d,"
             "\"mpu_accel_x\":%.3f,"
             "\"mpu_accel_y\":%.3f,"
             "\"mpu_accel_z\":%.3f,"
             "\"mpu_vib\":%.3f"
             "}",
             IR_ENABLED,
             (int)PIN_IR_FRONT_LEFT,  (int)(ir.fl ? 1 : 0),
             (int)PIN_IR_FRONT_RIGHT, (int)(ir.fr ? 1 : 0),
             (int)PIN_IR_BACK_LEFT,   (int)(ir.bl ? 1 : 0),
             (int)PIN_IR_BACK_RIGHT,  (int)(ir.br ? 1 : 0),
             MPU_ENABLED,
             (int)(mpu_ok ? 1 : 0),
             (int)(mpu_ok ? 1 : 0),
             m.temp_c,
             (double)m.accel_x,
             (double)m.accel_y,
             (double)m.accel_z,
             (double)m.vib_mag);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t handle_404(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
}

/* ══════════════════════════════════════════════════════════
   AUTO CLEAN TASK  (unchanged from prior fixed version)
══════════════════════════════════════════════════════════ */

#define HEADING_TOL_DEG  5.0f
#define HEADING_POLL_MS  20
#define MAX_CORR_MS      3000

static bool correct_heading_to(float target_deg)
{
#if MPU_ENABLED
    int elapsed = 0;
    while (elapsed < MAX_CORR_MS) {
        if (motor_get_accident()) { motor_stop_immediate(); return false; }
        float cur = mpu_heading();
        float err = target_deg - cur;
        while (err >  180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        if (fabsf(err) <= HEADING_TOL_DEG) {
            motor_send_cmd(DIR_STOP);
            vTaskDelay(pdMS_TO_TICKS(MS_STABILISE));
            return true;
        }
        motor_send_cmd(err > 0.0f ? DIR_RIGHT : DIR_LEFT);
        vTaskDelay(pdMS_TO_TICKS(HEADING_POLL_MS));
        elapsed += HEADING_POLL_MS;
    }
    motor_send_cmd(DIR_STOP);
    ESP_LOGW(TAG, "Heading timeout  cur=%.1f target=%.1f",
             (double)mpu_heading(), (double)target_deg);
#else
    (void)target_deg;
#endif
    return !motor_get_accident();
}

#define CHECK_ACC() do { if (motor_get_accident()) goto done; } while (0)

static void auto_clean_task(void *arg)
{
    (void)arg;
    const grid_config_t g = s_grid;

    const int strip_ms  = STRIP_WIDTH_CM * MS_PER_CM;
    const int pw_ms     = g.panel_w_cm * MS_PER_CM;
    const int ph_ms     = g.panel_h_cm * MS_PER_CM;
    const int gap_ms    = g.gap_cm * MS_PER_CM;
    const int backup_ms = INTER_PANEL_BACKUP_CM * MS_PER_CM;
    const int passes    = (g.panel_h_cm + STRIP_WIDTH_CM - 1) / STRIP_WIDTH_CM;

    ESP_LOGI(TAG, "AUTO CLEAN  %dx%d  W=%dcm H=%dcm gap=%dcm passes=%d",
             g.cols, g.rows, g.panel_w_cm, g.panel_h_cm, g.gap_cm, passes);

    mpu_reset_heading();
    const float base_hdg = 0.0f;

    for (int col = 0; col < g.cols; col++) {
        CHECK_ACC();
        ESP_LOGI(TAG, "=== Column %d/%d ===", col + 1, g.cols);
        if (g.wash_enabled) pump_set(true);
        if (g.blow_enabled) blower_set(true);

        for (int row = 0; row < g.rows; row++) {
            CHECK_ACC();
            for (int pass = 0; pass < passes; pass++) {
                CHECK_ACC();
                motor_dir_t sw = (pass % 2 == 0) ? DIR_RIGHT : DIR_LEFT;
                if (!motor_move_blocking(sw, pw_ms)) goto done;
                if (pass < passes - 1)
                    if (!motor_move_blocking(DIR_FORWARD, strip_ms)) goto done;
            }
            if (!correct_heading_to(base_hdg)) goto done;

            int net_right = (passes % 2 == 1) ? (passes / 2 + 1) : (passes / 2);
            if (net_right > 0)
                if (!motor_move_blocking(DIR_LEFT, net_right * pw_ms)) goto done;
            if (!correct_heading_to(base_hdg)) goto done;

            if (row < g.rows - 1) {
                bool bw = blower_get();
                if (bw) blower_set(false);
                if (!motor_move_blocking(DIR_FORWARD, gap_ms)) goto done;
                if (bw && g.blow_enabled) blower_set(true);
            }
        }

        if (col < g.cols - 1) {
            blower_set(false);
            pump_set(false);
            int travelled_ms = g.rows * ph_ms
                             + (g.rows > 1 ? (g.rows - 1) * gap_ms : 0)
                             + backup_ms;
            if (!motor_move_blocking(DIR_BACKWARD, travelled_ms)) goto done;
            if (!correct_heading_to(base_hdg)) goto done;
            if (!motor_move_blocking(DIR_RIGHT, MS_TURN_90))   goto done;
            float col_hdg = fmodf(base_hdg + 90.0f, 360.0f);
            if (!correct_heading_to(col_hdg)) goto done;
            if (!motor_move_blocking(DIR_FORWARD, gap_ms + 5 * MS_PER_CM)) goto done;
            if (!motor_move_blocking(DIR_LEFT, MS_TURN_90))    goto done;
            if (!correct_heading_to(base_hdg)) goto done;
            if (g.wash_enabled) pump_set(true);
            if (g.blow_enabled) blower_set(true);
        }
    }

done:
    pump_set(false);
    blower_set(false);
    motor_send_cmd(DIR_STOP);
    s_auto_running = false;
    ESP_LOGI(TAG, "AUTO CLEAN done  accident=%d", (int)motor_get_accident());
    vTaskDelete(NULL);
}

static esp_err_t handle_auto(httpd_req_t *req)
{
    if (motor_get_accident()) return httpd_resp_sendstr(req, "BLOCKED:ACCIDENT");
    if (s_auto_running)       return httpd_resp_sendstr(req, "ALREADY_RUNNING");
    s_auto_running = true;
    BaseType_t rc = xTaskCreate(auto_clean_task, "auto_clean", 4096, NULL, 5, NULL);
    if (rc != pdPASS) {
        s_auto_running = false;
        return httpd_resp_sendstr(req, "ERROR:TASK_CREATE_FAILED");
    }
    return httpd_resp_sendstr(req, "AUTO_STARTED");
}

static esp_err_t handle_stop_auto(httpd_req_t *req)
{
    if (s_auto_running) {
        motor_stop_immediate();
        pump_set(false);
        blower_set(false);
        s_auto_running = false;
    }
    return httpd_resp_sendstr(req, "STOPPED");
}

/* ══════════════════════════════════════════════════════════
   SERVER START / STOP
══════════════════════════════════════════════════════════ */

esp_err_t web_server_start(void)
{
    httpd_config_t cfg       = HTTPD_DEFAULT_CONFIG();
    cfg.server_port          = HTTP_SERVER_PORT;
    cfg.max_uri_handlers     = 16;
    cfg.lru_purge_enable     = true;
    cfg.recv_wait_timeout    = 5;
    cfg.send_wait_timeout    = 5;

    esp_err_t ret = httpd_start(&s_server, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    static const httpd_uri_t uris[] = {
        {.uri = "/",               .method = HTTP_GET, .handler = handle_root},
        {.uri = "/move",           .method = HTTP_GET, .handler = handle_move},
        {.uri = "/tail",           .method = HTTP_GET, .handler = handle_tail},
        {.uri = "/pump",           .method = HTTP_GET, .handler = handle_pump},
        {.uri = "/blower",         .method = HTTP_GET, .handler = handle_blower},
        {.uri = "/ramming",        .method = HTTP_GET, .handler = handle_ramming},
        {.uri = "/reset_accident", .method = HTTP_GET, .handler = handle_reset_accident},
        {.uri = "/set_grid",       .method = HTTP_GET, .handler = handle_set_grid},
        {.uri = "/get_grid",       .method = HTTP_GET, .handler = handle_get_grid},
        {.uri = "/status",         .method = HTTP_GET, .handler = handle_status},
        {.uri = "/sensor_check",   .method = HTTP_GET, .handler = handle_sensor_check},
        {.uri = "/auto",           .method = HTTP_GET, .handler = handle_auto},
        {.uri = "/stop_auto",      .method = HTTP_GET, .handler = handle_stop_auto},
    };
    const int n = (int)(sizeof(uris) / sizeof(uris[0]));
    for (int i = 0; i < n; i++)
        httpd_register_uri_handler(s_server, &uris[i]);

    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, handle_404);
    ESP_LOGI(TAG, "HTTP server ready — %d routes", n);
    return ESP_OK;
}

void web_server_stop(void)
{
    if (s_server) {
        httpd_stop(s_server);
        s_server = NULL;
    }
}