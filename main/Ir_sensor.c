/*
 * ir_sensor.c  —  4-channel IR obstacle sensor with debounce
 *
 * Unchanged from last working version.
 * Accident detection was never part of this module.
 */

#include "ir_sensor.h"
#include "config.h"
#include "motor.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "IR";

#define IR_STATUS_PRINT_INTERVAL  100   /* ~5 s at 50 ms poll */

/* ─── state ───────────────────────────────────────────────── */
static volatile int16_t s_cnt_fl = 0, s_cnt_fr = 0;
static volatile int16_t s_cnt_bl = 0, s_cnt_br = 0;
static volatile bool    s_fl = false, s_fr = false;
static volatile bool    s_bl = false, s_br = false;
static volatile bool    s_front_blocked = false;
static volatile bool    s_back_blocked  = false;

/* ─── helpers ─────────────────────────────────────────────── */

static esp_err_t cfg_input(int pin)
{
    gpio_config_t c = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&c);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "gpio_config failed pin %d: %s",
                 (int)pin, esp_err_to_name(err));
    return err;
}

static inline bool read_ir(int pin)
{
    return gpio_get_level(pin) == 0;   /* LOW = obstacle */
}

static void debounce_update(int pin,
                             volatile int16_t *cnt,
                             volatile bool    *state)
{
    if (read_ir(pin)) {
        if (*cnt < (int16_t)IR_DEBOUNCE_COUNT) (*cnt)++;
        if (*cnt >= (int16_t)IR_DEBOUNCE_COUNT) *state = true;
    } else {
        *cnt   = 0;
        *state = false;
    }
}

/* ─── public print ────────────────────────────────────────── */

void ir_print_status(void)
{
    ESP_LOGI(TAG,
             "[IR] FL=%d FR=%d BL=%d BR=%d | F=%d B=%d",
             (int)s_fl, (int)s_fr, (int)s_bl, (int)s_br,
             (int)s_front_blocked, (int)s_back_blocked);
}

/* ─── task ────────────────────────────────────────────────── */

#if IR_ENABLED

static void ir_task(void *arg)
{
    (void)arg;
    int print_cnt = 0;

    ESP_LOGI(TAG,
             "IR task  FL=%d FR=%d BL=%d BR=%d  poll=%d ms  deb=%d",
             (int)PIN_IR_FRONT_LEFT,  (int)PIN_IR_FRONT_RIGHT,
             (int)PIN_IR_BACK_LEFT,   (int)PIN_IR_BACK_RIGHT,
             (int)IR_POLL_INTERVAL_MS, (int)IR_DEBOUNCE_COUNT);

    ir_print_status();

    while (1) {
        debounce_update(PIN_IR_FRONT_LEFT,  &s_cnt_fl, &s_fl);
        debounce_update(PIN_IR_FRONT_RIGHT, &s_cnt_fr, &s_fr);
        debounce_update(PIN_IR_BACK_LEFT,   &s_cnt_bl, &s_bl);
        debounce_update(PIN_IR_BACK_RIGHT,  &s_cnt_br, &s_br);

        bool fb = s_fl || s_fr;
        bool bb = s_bl || s_br;

        if (fb != s_front_blocked || bb != s_back_blocked) {
            s_front_blocked = fb;
            s_back_blocked  = bb;
            motor_set_obstacles(fb, bb);

            ESP_LOGI(TAG,
                     "[IR CHANGE] FL=%d FR=%d BL=%d BR=%d F=%d B=%d",
                     (int)s_fl, (int)s_fr, (int)s_bl, (int)s_br,
                     (int)fb, (int)bb);

            if (fb) ESP_LOGW(TAG, "FRONT OBSTACLE (FL=%d FR=%d)",
                             (int)s_fl, (int)s_fr);
            if (bb) ESP_LOGW(TAG, "BACK OBSTACLE  (BL=%d BR=%d)",
                             (int)s_bl, (int)s_br);
        }

        if (++print_cnt >= IR_STATUS_PRINT_INTERVAL) {
            print_cnt = 0;
            ir_print_status();
        }

        vTaskDelay(pdMS_TO_TICKS(IR_POLL_INTERVAL_MS));
    }
}

#endif /* IR_ENABLED */

/* ─── Public API ──────────────────────────────────────────── */

void ir_init(void)
{
#if IR_ENABLED
    bool ok = true;
    ok &= (cfg_input(PIN_IR_FRONT_LEFT)  == ESP_OK);
    ok &= (cfg_input(PIN_IR_FRONT_RIGHT) == ESP_OK);
    ok &= (cfg_input(PIN_IR_BACK_LEFT)   == ESP_OK);
    ok &= (cfg_input(PIN_IR_BACK_RIGHT)  == ESP_OK);

    if (!ok) ESP_LOGE(TAG, "One or more IR pins failed!");

    ESP_LOGI(TAG,
             "IR boot levels: FL(pin%d)=%d  FR(pin%d)=%d  "
             "BL(pin%d)=%d  BR(pin%d)=%d",
             (int)PIN_IR_FRONT_LEFT,  gpio_get_level(PIN_IR_FRONT_LEFT),
             (int)PIN_IR_FRONT_RIGHT, gpio_get_level(PIN_IR_FRONT_RIGHT),
             (int)PIN_IR_BACK_LEFT,   gpio_get_level(PIN_IR_BACK_LEFT),
             (int)PIN_IR_BACK_RIGHT,  gpio_get_level(PIN_IR_BACK_RIGHT));

    xTaskCreatePinnedToCore(ir_task, "ir_task",
                            2048, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "IR ENABLED  FL=%d FR=%d BL=%d BR=%d",
             (int)PIN_IR_FRONT_LEFT,  (int)PIN_IR_FRONT_RIGHT,
             (int)PIN_IR_BACK_LEFT,   (int)PIN_IR_BACK_RIGHT);
#else
    ESP_LOGW(TAG, "IR DISABLED (IR_ENABLED=0)");
#endif
}

bool ir_front_blocked(void) { return s_front_blocked; }
bool ir_back_blocked(void)  { return s_back_blocked;  }
bool ir_front_left(void)    { return s_fl; }
bool ir_front_right(void)   { return s_fr; }
bool ir_back_left(void)     { return s_bl; }
bool ir_back_right(void)    { return s_br; }

ir_status_t ir_get_status(void)
{
    return (ir_status_t){
        .fl            = s_fl,
        .fr            = s_fr,
        .bl            = s_bl,
        .br            = s_br,
        .front_blocked = s_front_blocked,
        .back_blocked  = s_back_blocked,
    };
}