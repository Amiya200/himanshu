/*
 * main.c  —  ESP32 Solar Panel Cleaner  —  application entry point
 *
 * FIX: serial STATUS command now explicitly calls ir_print_status()
 * and prints full MPU data so both sensors are visible in one command.
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

#include "config.h"
#include "motor.h"
#include "ir_sensor.h"
#include "mpu6050.h"
#include "telegram.h"
#include "web_server.h"

static const char *TAG = "MAIN";

/* ─── WiFi ────────────────────────────────────────────────── */
static EventGroupHandle_t s_wifi_events;
#define WIFI_STA_CONNECTED_BIT  BIT0
#define WIFI_STA_FAIL_BIT       BIT1

static int s_sta_retries = 0;

static void wifi_event_handler(void *arg,
                                esp_event_base_t base,
                                int32_t          event_id,
                                void            *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_sta_retries < STA_MAX_RETRIES) {
            esp_wifi_connect();
            s_sta_retries++;
            ESP_LOGW(TAG, "WiFi STA reconnect attempt %d/%d",
                     s_sta_retries, (int)STA_MAX_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_events, WIFI_STA_FAIL_BIT);
            ESP_LOGE(TAG,
                     "WiFi STA failed after %d retries -- AP still active",
                     (int)STA_MAX_RETRIES);
        }

    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
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
        WIFI_EVENT, ESP_EVENT_ANY_ID,    wifi_event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,  IP_EVENT_STA_GOT_IP,  wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid,     STA_SSID,
            sizeof(sta_cfg.sta.ssid)     - 1);
    strncpy((char *)sta_cfg.sta.password, STA_PASSWORD,
            sizeof(sta_cfg.sta.password) - 1);
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    wifi_config_t ap_cfg = {0};
    strncpy((char *)ap_cfg.ap.ssid,     AP_SSID,
            sizeof(ap_cfg.ap.ssid)     - 1);
    strncpy((char *)ap_cfg.ap.password, AP_PASSWORD,
            sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len       = (uint8_t)strlen(AP_SSID);
    ap_cfg.ap.channel        = AP_CHANNEL;
    ap_cfg.ap.authmode       = WIFI_AUTH_WPA_WPA2_PSK;
    ap_cfg.ap.max_connection = AP_MAX_CONN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,  &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
                           s_wifi_events,
                           WIFI_STA_CONNECTED_BIT | WIFI_STA_FAIL_BIT,
                           pdFALSE, pdFALSE,
                           pdMS_TO_TICKS(10000));

    if (bits & WIFI_STA_CONNECTED_BIT) {
        ESP_LOGI(TAG, "STA connected to '%s'", STA_SSID);
    } else {
        ESP_LOGW(TAG, "STA not connected -- AP '%s' available on 192.168.4.1",
                 AP_SSID);
    }
}

/* ─── Accident monitor task ───────────────────────────────── */

static volatile bool s_telegram_sent = false;

static void accident_monitor_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Accident monitor active  vib_thr=%.2fg",
             (double)MPU_VIB_THRESHOLD_G);

    while (1) {
        bool acc = motor_get_accident();

        if (acc && !s_telegram_sent) {
            mpu_data_t d = mpu_get();
            char msg[512];
            snprintf(msg, sizeof(msg),
                     "*EMERGENCY: Accident Detected!*"
                     "\n\n*Source:* MPU-6050 Vibration"
                     "\n*Vib deviation:* %.3f g"
                     "\n*Heading:* %.1f deg"
                     "\n*Temp:* %d C"
                     "\n*Location:* %s",
                     (double)d.vib_mag,
                     (double)d.heading_deg,
                     d.temp_c,
                     CAR_LOCATION);
            telegram_send(msg);
            s_telegram_sent = true;
            gpio_set_level(PIN_TAIL_LIGHT, 1);
            ESP_LOGW(TAG, "Accident alert sent via Telegram");
        }

        if (!acc && s_telegram_sent) {
            s_telegram_sent = false;
            ESP_LOGI(TAG, "Accident cleared -- Telegram flag reset");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ─── Serial command task ─────────────────────────────────── */

static void serial_cmd_task(void *arg)
{
    (void)arg;
    char buf[64];
    int  pos = 0;
    ESP_LOGI(TAG, "Serial commands: ACC | RESET | HDG | STATUS | IR | MPU");

    while (1) {
        int c = fgetc(stdin);
        if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        if (c == '\r') continue;

        if (c == '\n' || pos >= (int)(sizeof(buf) - 1)) {
            buf[pos] = '\0';
            pos = 0;

            for (int i = 0; buf[i]; i++)
                buf[i] = (char)toupper((unsigned char)buf[i]);

            if (strcmp(buf, "ACC") == 0) {
                motor_set_accident(true);
                ESP_LOGI(TAG, "[SERIAL] ACC -- accident simulated");

            } else if (strcmp(buf, "RESET") == 0) {
                motor_set_accident(false);
                mpu_clear_accident();
                s_telegram_sent = false;
                gpio_set_level(PIN_TAIL_LIGHT, 0);
                ESP_LOGI(TAG, "[SERIAL] RESET -- accident cleared");

            } else if (strcmp(buf, "HDG") == 0) {
                ESP_LOGI(TAG, "[SERIAL] Heading=%.1f deg",
                         (double)mpu_heading());

            } else if (strcmp(buf, "IR") == 0) {
                /*
                 * New dedicated IR command: print raw pin levels AND
                 * debounced state so wiring can be verified instantly.
                 */
                ir_status_t ir = ir_get_status();
                ESP_LOGI(TAG, "=== IR SENSOR STATUS ===");
                ESP_LOGI(TAG, "  Front-Left  (pin %d): %s",
                         (int)PIN_IR_FRONT_LEFT,
                         ir.fl ? "BLOCKED (obstacle)" : "CLEAR");
                ESP_LOGI(TAG, "  Front-Right (pin %d): %s",
                         (int)PIN_IR_FRONT_RIGHT,
                         ir.fr ? "BLOCKED (obstacle)" : "CLEAR");
                ESP_LOGI(TAG, "  Back-Left   (pin %d): %s",
                         (int)PIN_IR_BACK_LEFT,
                         ir.bl ? "BLOCKED (obstacle)" : "CLEAR");
                ESP_LOGI(TAG, "  Back-Right  (pin %d): %s",
                         (int)PIN_IR_BACK_RIGHT,
                         ir.br ? "BLOCKED (obstacle)" : "CLEAR");
                ESP_LOGI(TAG, "  Front zone: %s  |  Back zone: %s",
                         ir.front_blocked ? "BLOCKED" : "CLEAR",
                         ir.back_blocked  ? "BLOCKED" : "CLEAR");
#if IR_ENABLED
                ESP_LOGI(TAG, "  IR_ENABLED=1 (active)");
#else
                ESP_LOGW(TAG, "  IR_ENABLED=0 in config.h -- sensors disabled!");
#endif

            } else if (strcmp(buf, "MPU") == 0) {
                /*
                 * New dedicated MPU command: print all MPU fields.
                 */
                mpu_data_t d = mpu_get();
                ESP_LOGI(TAG, "=== MPU-6050 STATUS ===");
                ESP_LOGI(TAG, "  Ready:     %s", mpu_is_ready() ? "YES" : "NO (check wiring)");
                ESP_LOGI(TAG, "  Accel X:   %.4f g", (double)d.accel_x);
                ESP_LOGI(TAG, "  Accel Y:   %.4f g", (double)d.accel_y);
                ESP_LOGI(TAG, "  Accel Z:   %.4f g  (expect ~1.0 on flat surface)",
                         (double)d.accel_z);
                ESP_LOGI(TAG, "  Gyro Z:    %.4f deg/s", (double)d.gyro_z);
                ESP_LOGI(TAG, "  Heading:   %.1f deg", (double)d.heading_deg);
                ESP_LOGI(TAG, "  Vibration: %.4f g  (threshold=%.2f)",
                         (double)d.vib_mag, (double)MPU_VIB_THRESHOLD_G);
                ESP_LOGI(TAG, "  Temp:      %d C", d.temp_c);
                ESP_LOGI(TAG, "  Accident:  %d", (int)d.accident);

            } else if (strcmp(buf, "STATUS") == 0) {
                /*
                 * Full combined status: motor + IR + MPU.
                 */
                mpu_data_t d   = mpu_get();
                ir_status_t ir = ir_get_status();

                ESP_LOGI(TAG, "=== FULL SYSTEM STATUS ===");

                /* Motor */
                ESP_LOGI(TAG, "  [MOTOR] dir=%s  accident=%d  ramming=%d  "
                         "pump=%d  blower=%d",
                         motor_dir_to_str(motor_get_direction()),
                         (int)motor_get_accident(),
                         (int)motor_get_ramming(),
                         (int)pump_get(),
                         (int)blower_get());

                /* IR */
#if IR_ENABLED
                ESP_LOGI(TAG, "  [IR]    FL=%d FR=%d BL=%d BR=%d  "
                         "F_blocked=%d B_blocked=%d",
                         (int)ir.fl, (int)ir.fr,
                         (int)ir.bl, (int)ir.br,
                         (int)ir.front_blocked, (int)ir.back_blocked);
#else
                ESP_LOGW(TAG, "  [IR]    DISABLED (IR_ENABLED=0 in config.h)");
#endif

                /* MPU */
                ESP_LOGI(TAG, "  [MPU]   ready=%d  Ax=%.3f Ay=%.3f Az=%.3f g  "
                         "Gz=%.2f deg/s  hdg=%.1f  vib=%.3f g  temp=%d C",
                         (int)mpu_is_ready(),
                         (double)d.accel_x, (double)d.accel_y, (double)d.accel_z,
                         (double)d.gyro_z, (double)d.heading_deg,
                         (double)d.vib_mag, d.temp_c);

            } else if (strlen(buf) > 0) {
                ESP_LOGW(TAG, "[SERIAL] Unknown command: '%s'", buf);
                ESP_LOGI(TAG, "  Available: ACC | RESET | HDG | IR | MPU | STATUS");
            }
        } else {
            buf[pos++] = (char)c;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/* ─── app_main ────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "========== ESP32 SOLAR CLEANER STARTING ==========");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased and reinitialised");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    motor_init();
    ir_init();
    mpu_init();
    wifi_init();
    telegram_init();

    ret = web_server_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Web server failed to start -- continuing without it");
    }

    xTaskCreate(accident_monitor_task, "acc_monitor",
                3072, NULL, 6, NULL);
    xTaskCreate(serial_cmd_task,       "serial_cmd",
                4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "========== SETUP COMPLETE ==========");
    ESP_LOGI(TAG, "AP SSID: %s  -> http://192.168.4.1", AP_SSID);
    ESP_LOGI(TAG, "MPU ready: %s",
             mpu_is_ready() ? "YES" : "NO (check wiring)");
    ESP_LOGI(TAG, "IR enabled: %s",
             IR_ENABLED ? "YES" : "NO (IR_ENABLED=0 in config.h)");
}