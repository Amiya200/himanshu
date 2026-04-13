/*
 * main.c  —  ESP32 Solar Panel Cleaner  —  application entry point
 *
 * FIXES vs original:
 *  1. WiFi event handler: STA_FAIL_BIT is set only when STA retries
 *     are exhausted; AP continues to operate independently regardless.
 *     A log warning is printed but AP is not torn down.
 *  2. accident_monitor_task: accesses motor_get_accident() which is
 *     a volatile bool read — safe without mutex on Xtensa.
 *     s_telegram_sent declared static volatile to prevent optimisation
 *     across the loop body.
 *  3. serial_cmd_task: stack kept at 4096; toupper() used instead of
 *     manual ASCII arithmetic (cleaner, locale-independent for ASCII).
 *  4. app_main: nvs_flash_init() result handled for all error codes.
 *  5. GPIO for PIN_TAIL_LIGHT configured in motor_init() — no need
 *     to repeat here; guard removed.
 *  6. xEventGroupWaitBits: pdFALSE (don't clear bits) kept so that
 *     WiFi status can be inspected after boot.
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
                     "WiFi STA failed after %d retries — AP still active",
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

    /* Both interfaces must be created before esp_wifi_start() */
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any, inst_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,    wifi_event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,  IP_EVENT_STA_GOT_IP,  wifi_event_handler, NULL, &inst_got_ip));

    /* STA config */
    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid,     STA_SSID,
            sizeof(sta_cfg.sta.ssid)     - 1);
    strncpy((char *)sta_cfg.sta.password, STA_PASSWORD,
            sizeof(sta_cfg.sta.password) - 1);
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    /* AP config */
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

    /* Wait up to 10 s for STA result; AP is available regardless */
    EventBits_t bits = xEventGroupWaitBits(
                           s_wifi_events,
                           WIFI_STA_CONNECTED_BIT | WIFI_STA_FAIL_BIT,
                           pdFALSE,   /* don't clear bits */
                           pdFALSE,   /* wait for any */
                           pdMS_TO_TICKS(10000));

    if (bits & WIFI_STA_CONNECTED_BIT) {
        ESP_LOGI(TAG, "STA connected to '%s'", STA_SSID);
    } else {
        ESP_LOGW(TAG, "STA not connected — AP '%s' available on 192.168.4.1",
                 AP_SSID);
    }
}

/* ─── Accident monitor task ───────────────────────────────── */

/*
 * Monitors the accident flag set by MPU vibration detection or
 * serial command.  Sends a single Telegram alert per accident
 * event.  Resets the sent flag when the accident is cleared.
 *
 * s_telegram_sent is volatile: accessed from this task only, but
 * the accident flag is set from mpu_task (different core) and we
 * must not cache it in a register.
 */
static volatile bool s_telegram_sent = false;

static void accident_monitor_task(void *arg)
{
    (void)arg;
    /* Give MPU task time to calibrate before we start monitoring */
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

        /* Reset sent flag when accident is cleared */
        if (!acc && s_telegram_sent) {
            s_telegram_sent = false;
            ESP_LOGI(TAG, "Accident cleared — Telegram flag reset");
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
    ESP_LOGI(TAG, "Serial commands: ACC | RESET | HDG | STATUS");

    while (1) {
        int c = fgetc(stdin);
        if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        if (c == '\r') continue;

        if (c == '\n' || pos >= (int)(sizeof(buf) - 1)) {
            buf[pos] = '\0';
            pos = 0;

            /* Convert to uppercase in-place */
            for (int i = 0; buf[i]; i++) {
                buf[i] = (char)toupper((unsigned char)buf[i]);
            }

            if (strcmp(buf, "ACC") == 0) {
                motor_set_accident(true);
                ESP_LOGI(TAG, "[SERIAL] ACC — accident simulated");

            } else if (strcmp(buf, "RESET") == 0) {
                motor_set_accident(false);
                mpu_clear_accident();
                s_telegram_sent = false;
                gpio_set_level(PIN_TAIL_LIGHT, 0);
                ESP_LOGI(TAG, "[SERIAL] RESET — accident cleared");

            } else if (strcmp(buf, "HDG") == 0) {
                ESP_LOGI(TAG, "[SERIAL] Heading=%.1f deg",
                         (double)mpu_heading());

            } else if (strcmp(buf, "STATUS") == 0) {
                mpu_data_t d = mpu_get();
                ir_status_t ir = ir_get_status();
                ESP_LOGI(TAG,
                         "[STATUS] acc=%d vib=%.3f hdg=%.1f temp=%d "
                         "IR_F=%d/%d IR_B=%d/%d pump=%d blow=%d",
                         (int)motor_get_accident(),
                         (double)d.vib_mag, (double)d.heading_deg, d.temp_c,
                         (int)ir.fl, (int)ir.fr,
                         (int)ir.bl, (int)ir.br,
                         (int)pump_get(), (int)blower_get());

            } else if (strlen(buf) > 0) {
                ESP_LOGW(TAG, "[SERIAL] Unknown: '%s'", buf);
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

    /*
     * NVS init — required by WiFi stack.
     * Erase and re-init if the partition is full or version-mismatched.
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition erased and reinitialised");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /*
     * BOOT ORDER — safe state enforced before any task starts:
     *
     *  1. motor_init()       → all drive pins LOW; relays OFF
     *  2. ir_init()          → configure IR GPIO inputs + task
     *  3. mpu_init()         → I2C + MPU-6050 setup + calibration task
     *  4. wifi_init()        → APSTA mode; blocks up to 10 s for STA
     *  5. telegram_init()    → HTTP client pool
     *  6. web_server_start() → HTTP server + URI handlers
     *  7. accident_monitor_task (waits 5 s before first check)
     *  8. serial_cmd_task
     */
    motor_init();
    ir_init();
    mpu_init();
    wifi_init();
    telegram_init();

    ret = web_server_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Web server failed to start — continuing without it");
    }

    xTaskCreate(accident_monitor_task, "acc_monitor",
                3072, NULL, 6, NULL);
    xTaskCreate(serial_cmd_task,       "serial_cmd",
                4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "========== SETUP COMPLETE ==========");
    ESP_LOGI(TAG, "AP SSID: %s  →  http://192.168.4.1", AP_SSID);
    ESP_LOGI(TAG, "MPU ready: %s", mpu_is_ready() ? "YES" : "NO (check wiring)");
}