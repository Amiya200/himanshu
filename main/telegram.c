// #include "telegram.h"
// #include "config.h"
// #include "esp_http_client.h"
// #include "esp_tls.h"
// #include "esp_log.h"
// #include "esp_wifi.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include <string.h>
// #include <stdio.h>

// static const char *TAG = "TELEGRAM";

// #define MAX_MSG_LEN     512
// #define QUEUE_DEPTH     4

// typedef struct {
//     char msg[MAX_MSG_LEN];
// } tg_item_t;

// static QueueHandle_t s_tg_queue = NULL;

// // ================== INTERNAL SEND ==================

// static void do_send(const char *message)
// {
//     // Check WiFi connectivity
//     wifi_ap_record_t ap;
//     if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
//         ESP_LOGW(TAG, "WiFi STA not connected - skipping Telegram");
//         return;
//     }

//     char url[256];
//     snprintf(url, sizeof(url),
//              TELEGRAM_API_URL "%s/sendMessage", TELEGRAM_BOT_TOKEN);

//     char post_data[MAX_MSG_LEN + 128];
//     snprintf(post_data, sizeof(post_data),
//              "chat_id=%s&text=%s&parse_mode=Markdown",
//              TELEGRAM_CHAT_ID, message);

//     esp_http_client_config_t cfg = {
//         .url              = url,
//         .method           = HTTP_METHOD_POST,
//         .transport_type   = HTTP_TRANSPORT_OVER_SSL,
//         .skip_cert_common_name_check = true,
//         .timeout_ms       = 10000,
//     };

//     esp_http_client_handle_t client = esp_http_client_init(&cfg);
//     if (!client) {
//         ESP_LOGE(TAG, "Failed to init HTTP client");
//         return;
//     }

//     esp_http_client_set_header(client,
//                                "Content-Type",
//                                "application/x-www-form-urlencoded");
//     esp_http_client_set_post_field(client, post_data, strlen(post_data));

//     esp_err_t err = esp_http_client_perform(client);
//     if (err == ESP_OK) {
//         int status = esp_http_client_get_status_code(client);
//         ESP_LOGI(TAG, "HTTP Status: %d", status);
//     } else {
//         ESP_LOGE(TAG, "HTTP request failed: %s", esp_err_to_name(err));
//     }

//     esp_http_client_cleanup(client);
// }

// // ================== TASK ==================

// static void telegram_task(void *arg)
// {
//     tg_item_t item;
//     while (1) {
//         if (xQueueReceive(s_tg_queue, &item, portMAX_DELAY) == pdTRUE) {
//             do_send(item.msg);
//         }
//     }
// }

// // ================== PUBLIC API ==================

// void telegram_init(void)
// {
//     s_tg_queue = xQueueCreate(QUEUE_DEPTH, sizeof(tg_item_t));
//     xTaskCreate(telegram_task, "telegram_task", 6144, NULL, 3, NULL);
//     ESP_LOGI(TAG, "Telegram module initialized");
// }

// void telegram_send(const char *message)
// {
//     if (!s_tg_queue || !message) return;
//     tg_item_t item;
//     strncpy(item.msg, message, MAX_MSG_LEN - 1);
//     item.msg[MAX_MSG_LEN - 1] = '\0';

//     if (xQueueSend(s_tg_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
//         ESP_LOGW(TAG, "Queue full - message dropped");
//     }
// }