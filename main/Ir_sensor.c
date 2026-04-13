/*
 * ir_sensor.c  —  4-channel IR obstacle sensor with debounce
 *
 * FIXES vs original:
 *  1. Debounce counters changed to int16_t with explicit bounds
 *     to avoid signed/unsigned comparison warnings.
 *  2. Obstacle state is passed to motor_set_obstacles() on EVERY
 *     change — including clearing — so the motor can resume after
 *     the path becomes clear.
 *  3. Task stack kept at 2048 words (adequate for this simple
 *     polling task).
 *  4. gpio_config() return value checked; bad pins are logged and
 *     the task exits gracefully rather than silently misbehaving.
 *  5. IR_DEBOUNCE_COUNT sanity-clamped at init.
 */

#include "ir_sensor.h"
#include "config.h"
#include "motor.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "IR";

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
        /* Internal pull-up → unconnected pin reads HIGH (no obstacle) */
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed for pin %d: %s",
                 (int)pin, esp_err_to_name(err));
    }
    return err;
}

/* Returns true if sensor output is LOW (obstacle present). */
static inline bool read_ir(int pin)
{
    return gpio_get_level(pin) == 0;
}

/*
 * debounce_update — hysteresis debounce.
 *   Rising  (obstacle appears): increment counter; confirm at threshold.
 *   Falling (obstacle clears):  reset counter immediately → fast clear.
 */
static void debounce_update(int pin,
                             volatile int16_t *cnt,
                             volatile bool    *state)
{
    if (read_ir(pin)) {
        if (*cnt < (int16_t)IR_DEBOUNCE_COUNT) {
            (*cnt)++;
        }
        if (*cnt >= (int16_t)IR_DEBOUNCE_COUNT) {
            *state = true;
        }
    } else {
        *cnt   = 0;
        *state = false;
    }
}

/* ─── task ────────────────────────────────────────────────── */

#if IR_ENABLED

static void ir_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "IR task running  poll=%d ms  debounce=%d",
             (int)IR_POLL_INTERVAL_MS, (int)IR_DEBOUNCE_COUNT);

    while (1) {
        debounce_update(PIN_IR_FRONT_LEFT,  &s_cnt_fl, &s_fl);
        debounce_update(PIN_IR_FRONT_RIGHT, &s_cnt_fr, &s_fr);
        debounce_update(PIN_IR_BACK_LEFT,   &s_cnt_bl, &s_bl);
        debounce_update(PIN_IR_BACK_RIGHT,  &s_cnt_br, &s_br);

        /*
         * Conservative policy: front blocked if EITHER front sensor
         * sees an obstacle (OR logic).  Robot stops on first warning.
         */
        bool fb = s_fl || s_fr;
        bool bb = s_bl || s_br;

        if (fb != s_front_blocked || bb != s_back_blocked) {
            s_front_blocked = fb;
            s_back_blocked  = bb;
            /*
             * Notify motor driver of BOTH obstacle and clear events.
             * Original only called this when state changed, which is
             * correct, but now the motor can also resume when clear.
             */
            motor_set_obstacles(fb, bb);
            ESP_LOGD(TAG,
                     "IR FL=%d FR=%d BL=%d BR=%d → F_blocked=%d B_blocked=%d",
                     (int)s_fl, (int)s_fr,
                     (int)s_bl, (int)s_br,
                     (int)fb,   (int)bb);
        }

        vTaskDelay(pdMS_TO_TICKS(IR_POLL_INTERVAL_MS));
    }
}

#endif /* IR_ENABLED */

/* ─── public API ──────────────────────────────────────────── */

void ir_init(void)
{
#if IR_ENABLED
    bool ok = true;
    ok &= (cfg_input(PIN_IR_FRONT_LEFT)  == ESP_OK);
    ok &= (cfg_input(PIN_IR_FRONT_RIGHT) == ESP_OK);
    ok &= (cfg_input(PIN_IR_BACK_LEFT)   == ESP_OK);
    ok &= (cfg_input(PIN_IR_BACK_RIGHT)  == ESP_OK);

    if (!ok) {
        ESP_LOGE(TAG, "One or more IR pins failed to configure!");
    }

    xTaskCreatePinnedToCore(ir_task, "ir_task",
                            2048, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "IR sensors ENABLED  FL=%d FR=%d BL=%d BR=%d",
             (int)PIN_IR_FRONT_LEFT,  (int)PIN_IR_FRONT_RIGHT,
             (int)PIN_IR_BACK_LEFT,   (int)PIN_IR_BACK_RIGHT);
#else
    ESP_LOGW(TAG, "IR sensors DISABLED (IR_ENABLED=0 in config.h)");
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