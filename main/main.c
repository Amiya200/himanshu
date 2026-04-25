/*
 * main.c  —  ESP32 Solar Panel Cleaner  —  application entry point
 *
 * CHANGES vs original:
 *  - Accident detection / Telegram system REMOVED entirely
 *  - accident_monitor_task REMOVED
 *  - auto_clean module integrated (no inline task here)
 *  - Serial commands updated: removed ACC/RESET, added AUTO/STOP/GRID
 *  - Setup log shows rover dimensions and auto-clean config
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "servo.h"
#include "config.h"
#include "motor.h"
#include "ir_sensor.h"
#include "mpu6050.h"
#include "auto_clean.h"
#include "web_server.h"

static const char *TAG = "MAIN";

/* ─── WiFi ────────────────────────────────────────────────── */
static EventGroupHandle_t s_wifi_events;
#define WIFI_STA_CONNECTED_BIT BIT0
#define WIFI_STA_FAIL_BIT BIT1

static int s_sta_retries = 0;

static void wifi_event_handler(void *arg,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_sta_retries < STA_MAX_RETRIES)
        {
            esp_wifi_connect();
            s_sta_retries++;
            ESP_LOGW(TAG, "WiFi STA reconnect %d/%d",
                     s_sta_retries, (int)STA_MAX_RETRIES);
        }
        else
        {
            xEventGroupSetBits(s_wifi_events, WIFI_STA_FAIL_BIT);
            ESP_LOGE(TAG, "STA failed -- AP still active");
        }
    }
    else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        s_sta_retries = 0;
        xEventGroupSetBits(s_wifi_events, WIFI_STA_CONNECTED_BIT);
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "STA IP: " IPSTR, IP2STR(&ev->ip_info.ip));
    }
}

static void wifi_init(void)
{
    s_wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any, inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid, STA_SSID,
            sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, STA_PASSWORD,
            sizeof(sta_cfg.sta.password) - 1);
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    wifi_config_t ap_cfg = {0};
    strncpy((char *)ap_cfg.ap.ssid, AP_SSID,
            sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char *)ap_cfg.ap.password, AP_PASSWORD,
            sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len = (uint8_t)strlen(AP_SSID);
    ap_cfg.ap.channel = AP_CHANNEL;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ap_cfg.ap.max_connection = AP_MAX_CONN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events,
        WIFI_STA_CONNECTED_BIT | WIFI_STA_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(10000));

    if (bits & WIFI_STA_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "STA connected to '%s'", STA_SSID);
    }
    else
    {
        ESP_LOGW(TAG, "STA not connected -- AP '%s' @ 192.168.4.1", AP_SSID);
    }
}

/* ─── Serial command task ─────────────────────────────────── */

static void serial_cmd_task(void *arg)
{
    (void)arg;
    char buf[64];
    int pos = 0;

    ESP_LOGI(TAG, "Serial commands:");
    ESP_LOGI(TAG, "  AUTO    -- start auto-clean");
    ESP_LOGI(TAG, "  STOP    -- stop auto-clean");
    ESP_LOGI(TAG, "  STATUS  -- full system status");
    ESP_LOGI(TAG, "  IR      -- IR sensor status");
    ESP_LOGI(TAG, "  MPU     -- MPU-6050 status");
    ESP_LOGI(TAG, "  HDG     -- current gyro heading");
    ESP_LOGI(TAG, "  HDGRST  -- reset heading to 0");
    ESP_LOGI(TAG, "  GRID    -- show current grid config");

    while (1)
    {
        int c = fgetc(stdin);
        if (c == EOF)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r')
            continue;

        if (c == '\n' || pos >= (int)(sizeof(buf) - 1))
        {
            buf[pos] = '\0';
            pos = 0;

            for (int i = 0; buf[i]; i++)
                buf[i] = (char)toupper((unsigned char)buf[i]);

            /* ── AUTO ── */
            if (strcmp(buf, "AUTO") == 0)
            {
                if (auto_clean_is_running())
                {
                    ESP_LOGW(TAG, "[SERIAL] AUTO -- already running");
                }
                else if (auto_clean_start())
                {
                    ESP_LOGI(TAG, "[SERIAL] AUTO -- started");
                }
                else
                {
                    ESP_LOGE(TAG, "[SERIAL] AUTO -- failed to start");
                }

                /* ── STOP ── */
            }
            else if (strcmp(buf, "STOP") == 0)
            {
                auto_clean_stop();
                ESP_LOGI(TAG, "[SERIAL] STOP -- stop requested");

                /* ── HDG ── */
            }
            else if (strcmp(buf, "HDG") == 0)
            {
                ESP_LOGI(TAG, "[SERIAL] Heading=%.1f deg",
                         (double)mpu_heading());

                /* ── HDGRST ── */
            }
            else if (strcmp(buf, "HDGRST") == 0)
            {
                mpu_reset_heading();
                ESP_LOGI(TAG, "[SERIAL] Heading reset to 0");

                /* ── IR ── */
            }
            else if (strcmp(buf, "IR") == 0)
            {
                ir_status_t ir = ir_get_status();
                ESP_LOGI(TAG, "=== IR STATUS ===");
                ESP_LOGI(TAG, "  Front-Left  (pin %d): %s",
                         (int)PIN_IR_FRONT_LEFT,
                         ir.fl ? "BLOCKED" : "CLEAR");
                ESP_LOGI(TAG, "  Front-Right (pin %d): %s",
                         (int)PIN_IR_FRONT_RIGHT,
                         ir.fr ? "BLOCKED" : "CLEAR");
                ESP_LOGI(TAG, "  Back-Left   (pin %d): %s",
                         (int)PIN_IR_BACK_LEFT,
                         ir.bl ? "BLOCKED" : "CLEAR");
                ESP_LOGI(TAG, "  Back-Right  (pin %d): %s",
                         (int)PIN_IR_BACK_RIGHT,
                         ir.br ? "BLOCKED" : "CLEAR");
                ESP_LOGI(TAG, "  Front zone: %s  |  Back zone: %s",
                         ir.front_blocked ? "BLOCKED" : "CLEAR",
                         ir.back_blocked ? "BLOCKED" : "CLEAR");

                /* ── MPU ── */
            }
            else if (strcmp(buf, "MPU") == 0)
            {
                mpu_data_t d = mpu_get();
                ESP_LOGI(TAG, "=== MPU STATUS ===");
                ESP_LOGI(TAG, "  Ready:   %s", mpu_is_ready() ? "YES" : "NO");
                ESP_LOGI(TAG, "  Accel X: %.4f g", (double)d.accel_x);
                ESP_LOGI(TAG, "  Accel Y: %.4f g", (double)d.accel_y);
                ESP_LOGI(TAG, "  Accel Z: %.4f g (expect ~1.0 flat)",
                         (double)d.accel_z);
                ESP_LOGI(TAG, "  Gyro Z:  %.4f deg/s", (double)d.gyro_z);
                ESP_LOGI(TAG, "  Heading: %.1f deg", (double)d.heading_deg);
                ESP_LOGI(TAG, "  Vib:     %.4f g", (double)d.vib_mag);
                ESP_LOGI(TAG, "  Temp:    %d C", d.temp_c);

                /* ── STATUS ── */
            }
            else if (strcmp(buf, "STATUS") == 0)
            {
                mpu_data_t d = mpu_get();
                ir_status_t ir = ir_get_status();
                clean_grid_t g;
                auto_clean_get_grid(&g);

                ESP_LOGI(TAG, "=== FULL SYSTEM STATUS ===");
                ESP_LOGI(TAG, "  [MOTOR] dir=%s  pump=%d  blower=%d",
                         motor_dir_to_str(motor_get_direction()),
                         (int)pump_get(), (int)blower_get());
#if IR_ENABLED
                ESP_LOGI(TAG, "  [IR]    FL=%d FR=%d BL=%d BR=%d  F=%d B=%d",
                         (int)ir.fl, (int)ir.fr,
                         (int)ir.bl, (int)ir.br,
                         (int)ir.front_blocked, (int)ir.back_blocked);
#else
                ESP_LOGW(TAG, "  [IR]    DISABLED");
#endif
                ESP_LOGI(TAG,
                         "  [MPU]   ready=%d Ax=%.3f Ay=%.3f Az=%.3f "
                         "Gz=%.2f hdg=%.1f vib=%.3f temp=%d",
                         (int)mpu_is_ready(),
                         (double)d.accel_x, (double)d.accel_y, (double)d.accel_z,
                         (double)d.gyro_z, (double)d.heading_deg,
                         (double)d.vib_mag, d.temp_c);
                ESP_LOGI(TAG,
                         "  [AUTO]  state=%d  pct=%d%%  strips=%d/%d",
                         (int)auto_clean_state(),
                         auto_clean_pct(),
                         auto_clean_strips_done(),
                         auto_clean_strips_total());

                /* ── GRID ── */
            }
            else if (strcmp(buf, "GRID") == 0)
            {
                clean_grid_t g;
                auto_clean_get_grid(&g);
                int strips = (g.panel_w_cm + ROVER_WIDTH_CM - 1) / ROVER_WIDTH_CM;
                ESP_LOGI(TAG, "=== GRID CONFIG ===");
                ESP_LOGI(TAG, "  Columns:      %d", g.panel_cols);
                ESP_LOGI(TAG, "  Rows:         %d", g.panel_rows);
                ESP_LOGI(TAG, "  Panel W:      %d cm", g.panel_w_cm);
                ESP_LOGI(TAG, "  Panel H:      %d cm", g.panel_h_cm);
                ESP_LOGI(TAG, "  Gap:          %d cm", g.gap_between_cm);
                ESP_LOGI(TAG, "  Wash:         %s", g.wash_enabled ? "YES" : "NO");
                ESP_LOGI(TAG, "  Blow:         %s", g.blow_enabled ? "YES" : "NO");
                ESP_LOGI(TAG, "  Strips/panel: %d (rover=%d cm wide)", strips, ROVER_WIDTH_CM);
                ESP_LOGI(TAG, "  Total strips: %d", strips * g.panel_cols * g.panel_rows);
            }
            else if (strlen(buf) > 0)
            {
                ESP_LOGW(TAG, "[SERIAL] Unknown: '%s'", buf);
                ESP_LOGI(TAG, "  Available: AUTO | STOP | STATUS | IR | MPU | HDG | HDGRST | GRID");
            }
        }
        else
        {
            buf[pos++] = (char)c;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ─── app_main ────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "========== ESP32 SOLAR CLEANER v2.0 STARTING ==========");
    ESP_LOGI(TAG, "Rover: %d cm wide x %d cm deep",
             ROVER_WIDTH_CM, ROVER_DEPTH_CM);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS erased and reinitialised");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_init();
    ir_init();
    mpu_init();
    wifi_init();
    servo_init();
    ret = web_server_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Web server failed -- continuing without it");
    }

    xTaskCreate(serial_cmd_task, "serial_cmd",
                4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "========== SETUP COMPLETE ==========");
    ESP_LOGI(TAG, "AP SSID: %s  -> http://192.168.4.1", AP_SSID);
    ESP_LOGI(TAG, "MPU ready:  %s", mpu_is_ready() ? "YES" : "NO (check wiring)");
    ESP_LOGI(TAG, "IR enabled: %s", IR_ENABLED ? "YES" : "NO");
    ESP_LOGI(TAG, "Clean pattern: boustrophedon (snake across panel width)");
    ESP_LOGI(TAG, "  FORWARD = panel top -> bottom (cleaning)");
    ESP_LOGI(TAG, "  BACKWARD = panel bottom -> top (rewind only)");
    ESP_LOGI(TAG, "Place rover at TOP-LEFT of first panel, facing DOWN, then press AUTO CLEAN");
}