#include <stdio.h>
#include <string.h>
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
#include "driver/adc.h"

#include "config.h"
#include "motor.h"
#include "ultrasonic.h"
#include "telegram.h"
#include "web_server.h"

static const char *TAG = "MAIN";

// ================== WIFI EVENT GROUP ==================
static EventGroupHandle_t s_wifi_events;
#define WIFI_STA_CONNECTED_BIT  BIT0
#define WIFI_STA_FAIL_BIT       BIT1

static int  s_sta_retries   = 0;
static bool s_telegram_sent = false;

extern void web_server_set_vib(int v);

// ================== WIFI EVENT HANDLER ==================

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_sta_retries < STA_MAX_RETRIES) {
            esp_wifi_connect();
            s_sta_retries++;
            ESP_LOGW(TAG, "WiFi STA reconnecting... (%d/%d)", s_sta_retries, STA_MAX_RETRIES);
        } else {
            xEventGroupSetBits(s_wifi_events, WIFI_STA_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi STA failed after %d retries", STA_MAX_RETRIES);
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi STA IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_sta_retries = 0;
        xEventGroupSetBits(s_wifi_events, WIFI_STA_CONNECTED_BIT);
    }
}

// ================== WIFI INIT ==================

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
        WIFI_EVENT, ESP_EVENT_ANY_ID,   wifi_event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, &inst_got_ip));

    wifi_config_t sta_cfg = { 0 };
    strncpy((char*)sta_cfg.sta.ssid,     STA_SSID,     sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char*)sta_cfg.sta.password, STA_PASSWORD, sizeof(sta_cfg.sta.password) - 1);
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    wifi_config_t ap_cfg = { 0 };
    strncpy((char*)ap_cfg.ap.ssid,     AP_SSID,     sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char*)ap_cfg.ap.password, AP_PASSWORD, sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len       = strlen(AP_SSID);
    ap_cfg.ap.channel        = AP_CHANNEL;
    ap_cfg.ap.authmode       = WIFI_AUTH_WPA_WPA2_PSK;
    ap_cfg.ap.max_connection = AP_MAX_CONN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,  &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP+STA started");
    ESP_LOGI(TAG, "AP SSID: %s  PASS: %s", AP_SSID, AP_PASSWORD);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events,
        WIFI_STA_CONNECTED_BIT | WIFI_STA_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(10000));

    if (bits & WIFI_STA_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi STA connected to '%s'", STA_SSID);
    } else {
        ESP_LOGW(TAG, "WiFi STA not connected — Telegram unavailable");
    }
}

// ================== VIBRATION / ACCIDENT TASK ==================

static void accident_task(void *arg)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

    // Wait for ADC and power rails to settle before monitoring
    ESP_LOGI(TAG, "Vibration monitor: settling for 3 s...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "Vibration monitor active (threshold=%d)", VIB_THRESHOLD);

    int consecutive = 0;

    while (1) {
        int raw = adc1_get_raw(ADC1_CHANNEL_7);
        web_server_set_vib(raw);

        if (raw > VIB_THRESHOLD) {
            consecutive++;
        } else {
            consecutive = 0;
        }

        // Require VIB_DEBOUNCE_COUNT sustained readings to avoid false triggers
        if (consecutive >= VIB_DEBOUNCE_COUNT && !motor_get_accident()) {
            ESP_LOGW(TAG, "ACCIDENT! vib=%d (x%d readings)", raw, consecutive);
            motor_set_accident(true);
            gpio_set_level(PIN_TAIL_LIGHT, 1);

            if (!s_telegram_sent) {
                char msg[512];
                snprintf(msg, sizeof(msg),
                    "*EMERGENCY ALERT: Car Accident Detected!*"
                    "\n\n*Source:* Vibration Sensor"
                    "\n*Vibration:* %d (Threshold: %d)"
                    "\n*State:* Movement blocked, Tail light ON"
                    "\n*Location:* %s",
                    raw, VIB_THRESHOLD, CAR_LOCATION);
                telegram_send(msg);
                s_telegram_sent = true;
            }
            consecutive = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(VIB_CHECK_INTERVAL_MS));
    }
}

// ================== SERIAL COMMAND TASK ==================

static void serial_cmd_task(void *arg)
{
    char buf[64];
    int  pos = 0;

    ESP_LOGI(TAG, "Serial: ACC = simulate accident | RESET = clear accident");

    while (1) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (c == '\r') continue;
        if (c == '\n' || pos >= (int)sizeof(buf) - 1) {
            buf[pos] = '\0';
            pos = 0;

            for (int i = 0; buf[i]; i++) {
                if (buf[i] >= 'a' && buf[i] <= 'z') buf[i] -= 32;
            }

            if (strcmp(buf, "ACC") == 0) {
                ESP_LOGI(TAG, "[SERIAL] ACC -> simulating accident");
                if (!motor_get_accident() && !s_telegram_sent) {
                    char msg[256];
                    snprintf(msg, sizeof(msg),
                        "*EMERGENCY ALERT: Car Accident (Manual Trigger)!*"
                        "\n\n*Source:* Serial Command"
                        "\n*Location:* %s", CAR_LOCATION);
                    telegram_send(msg);
                    s_telegram_sent = true;
                }
                motor_set_accident(true);
                gpio_set_level(PIN_TAIL_LIGHT, 1);

            } else if (strcmp(buf, "RESET") == 0) {
                ESP_LOGI(TAG, "[SERIAL] RESET -> clearing accident");
                motor_set_accident(false);
                s_telegram_sent = false;
                gpio_set_level(PIN_TAIL_LIGHT, 0);

            } else if (strlen(buf) > 0) {
                ESP_LOGW(TAG, "[SERIAL] Unknown: '%s' (use ACC or RESET)", buf);
            }
        } else {
            buf[pos++] = (char)c;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ================== APP MAIN ==================

void app_main(void)
{
    ESP_LOGI(TAG, "========== ESP32 SOLAR CLEANER STARTING ==========");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Motor driver first — puts all drive pins in safe state
    motor_init();

    // 2. Ultrasonic — safe to call always.
    //    When ULTRASONIC_ENABLED=0 in config.h it just logs a warning
    //    and returns immediately without touching any GPIO.
    ultrasonic_init();

    // 3. WiFi
    wifi_init();

    // 4. Telegram
    telegram_init();

    // 5. HTTP web server
    ESP_ERROR_CHECK(web_server_start());

    // 6. Vibration / accident task
    xTaskCreate(accident_task,   "accident_task",   3072, NULL, 6, NULL);

    // 7. Serial command task — needs 4096, not 2048 (fgetc pulls stdio overhead)
    xTaskCreate(serial_cmd_task, "serial_cmd_task", 4096, NULL, 2, NULL);

    ESP_LOGI(TAG, "========== SETUP COMPLETE ==========");
    ESP_LOGI(TAG, "AP SSID: '%s'  password: '%s'", AP_SSID, AP_PASSWORD);
    ESP_LOGI(TAG, "Open browser -> http://192.168.4.1");
}