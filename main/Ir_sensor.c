/*
 * ir_sensor.c  —  4-channel IR obstacle sensor with debounce
 *
 * FIX vs previous:
 *   IR_ENABLED was 0 in config.h so this file's task never ran.
 *   Config.h now has IR_ENABLED=1.
 *
 *   Added: periodic serial status print every IR_STATUS_PRINT_INTERVAL
 *   polls so IR state is always visible on serial monitor without
 *   needing to type STATUS.  Also prints immediately on any state change.
 *
 *   Added: ir_print_status() public function so main.c STATUS command
 *   can call it directly for instant output.
 */

#include "ir_sensor.h"
#include "config.h"
#include "motor.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "IR";

/* Print IR state every N poll cycles (~5 s at 50 ms poll) */
#define IR_STATUS_PRINT_INTERVAL    100

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
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed for pin %d: %s",
                 (int)pin, esp_err_to_name(err));
    }
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

/* ─── public print helper ─────────────────────────────────── */

void ir_print_status(void)
{
    ESP_LOGI(TAG,
             "[IR] FL=%d FR=%d BL=%d BR=%d | F_blocked=%d B_blocked=%d",
             (int)s_fl, (int)s_fr, (int)s_bl, (int)s_br,
             (int)s_front_blocked, (int)s_back_blocked);
}

/* ─── task ────────────────────────────────────────────────── */

#if IR_ENABLED

static void ir_task(void *arg)
{
    (void)arg;
    int print_cnt = 0;

    ESP_LOGI(TAG, "IR task running  pins FL=%d FR=%d BL=%d BR=%d  "
             "poll=%d ms  debounce=%d",
             (int)PIN_IR_FRONT_LEFT,  (int)PIN_IR_FRONT_RIGHT,
             (int)PIN_IR_BACK_LEFT,   (int)PIN_IR_BACK_RIGHT,
             (int)IR_POLL_INTERVAL_MS, (int)IR_DEBOUNCE_COUNT);

    /* Print initial state immediately */
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

            /* Print immediately on every state change */
            ESP_LOGI(TAG,
                     "[IR CHANGE] FL=%d FR=%d BL=%d BR=%d | "
                     "F_blocked=%d B_blocked=%d",
                     (int)s_fl, (int)s_fr, (int)s_bl, (int)s_br,
                     (int)fb, (int)bb);

            if (fb) ESP_LOGW(TAG, "FRONT OBSTACLE DETECTED (FL=%d FR=%d)",
                             (int)s_fl, (int)s_fr);
            if (bb) ESP_LOGW(TAG, "BACK OBSTACLE DETECTED  (BL=%d BR=%d)",
                             (int)s_bl, (int)s_br);
        }

        /* Periodic print so serial always shows current IR state */
        print_cnt++;
        if (print_cnt >= IR_STATUS_PRINT_INTERVAL) {
            print_cnt = 0;
            ir_print_status();
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

    /* Raw pin state at boot before debounce -- helps diagnose wiring */
    ESP_LOGI(TAG, "IR raw pin levels at boot:  FL(pin%d)=%d  FR(pin%d)=%d  "
             "BL(pin%d)=%d  BR(pin%d)=%d",
             (int)PIN_IR_FRONT_LEFT,  gpio_get_level(PIN_IR_FRONT_LEFT),
             (int)PIN_IR_FRONT_RIGHT, gpio_get_level(PIN_IR_FRONT_RIGHT),
             (int)PIN_IR_BACK_LEFT,   gpio_get_level(PIN_IR_BACK_LEFT),
             (int)PIN_IR_BACK_RIGHT,  gpio_get_level(PIN_IR_BACK_RIGHT));
    ESP_LOGI(TAG, "  Expected in open area: all 1 (HIGH = no obstacle)");
    ESP_LOGI(TAG, "  If any pin reads 0 with nothing in front: check wiring");

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