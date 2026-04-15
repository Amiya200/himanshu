/*
 * motor.c  —  L298N motor driver + relay peripherals
 *
 * CHANGES vs original:
 *  - ALL accident detection code removed
 *  - motor_move_straight() added: MPU gyro-assisted straight travel
 *    (differential correction on LEFT/RIGHT channels while moving
 *     FORWARD or BACKWARD to maintain a target heading)
 *  - BACKWARD direction preserved — used only for inter-strip lateral
 *    repositioning in auto_clean; never used for panel cleaning sweeps
 *  - IR obstacle blocking kept (IR_ENABLED compile flag)
 *  - Active-LOW relay logic retained
 *
 * Rover: 58 cm wide × 51 cm deep, L298N 4-wheel drive
 */

#include "motor.h"
#include "config.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MOTOR";

/* ─── state ───────────────────────────────────────────────── */
static volatile motor_dir_t s_current_dir = DIR_STOP;
static volatile bool        s_ramming     = false;
static volatile bool        s_pump_on     = false;
static volatile bool        s_blower_on   = false;

/* Obstacle flags — written only by ir_task via motor_set_obstacles() */
static volatile bool s_front_blocked = false;
static volatile bool s_back_blocked  = false;

static QueueHandle_t s_motor_queue = NULL;

/* ─── GPIO helpers ────────────────────────────────────────── */

static void gpio_out(int pin)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

/* Active-LOW relay: on=true → pin LOW → relay energised */
static inline void relay_write(int pin, bool on)
{
    gpio_set_level(pin, on ? 0 : 1);
}

static inline void set_pins(int in1, int in2, int in3, int in4,
                             int ena, int enb)
{
    gpio_set_level(PIN_IN1, in1);
    gpio_set_level(PIN_IN2, in2);
    gpio_set_level(PIN_IN3, in3);
    gpio_set_level(PIN_IN4, in4);
    gpio_set_level(PIN_ENA, ena);
    gpio_set_level(PIN_ENB, enb);
}

/* ─── raw drive functions (called from motor_task or blocking) ─ */

static void do_stop(void)
{
    set_pins(0, 0, 0, 0, 0, 0);
    s_current_dir = DIR_STOP;
}

static void do_forward(void)
{
#if IR_ENABLED
    if (!s_ramming && s_front_blocked) {
        ESP_LOGW(TAG, "Forward blocked by IR (front)");
        do_stop();
        return;
    }
#endif
    set_pins(0, 1, 1, 0, 1, 1);
    s_current_dir = DIR_FORWARD;
}

static void do_backward(void)
{
#if IR_ENABLED
    if (!s_ramming && s_back_blocked) {
        ESP_LOGW(TAG, "Backward blocked by IR (back)");
        do_stop();
        return;
    }
#endif
    set_pins(1, 0, 0, 1, 1, 1);
    s_current_dir = DIR_BACKWARD;
}

static void do_left(void)
{
    set_pins(1, 0, 1, 0, 1, 1);
    s_current_dir = DIR_LEFT;
}

static void do_right(void)
{
    set_pins(0, 1, 0, 1, 1, 1);
    s_current_dir = DIR_RIGHT;
}

static void do_drift_left(void)
{
    set_pins(0, 0, 1, 0, 1, 1);
    s_current_dir = DIR_DRIFT_LEFT;
}

static void do_drift_right(void)
{
    set_pins(0, 1, 0, 0, 1, 1);
    s_current_dir = DIR_DRIFT_RIGHT;
}

/* ─── motor task ──────────────────────────────────────────── */

static volatile motor_dir_t s_cmd = DIR_STOP;

static void motor_task(void *arg)
{
    (void)arg;

    motor_dir_t last_cmd = DIR_STOP;
    do_stop();

    while (1) {
        motor_dir_t cmd = s_cmd;

        // Only update when command changes (reduces GPIO noise)
        if (cmd != last_cmd) {
            switch (cmd) {
                case DIR_FORWARD:     do_forward();     break;
                case DIR_BACKWARD:    do_backward();    break;
                case DIR_LEFT:        do_left();        break;
                case DIR_RIGHT:       do_right();       break;
                case DIR_DRIFT_LEFT:  do_drift_left();  break;
                case DIR_DRIFT_RIGHT: do_drift_right(); break;
                default:              do_stop();        break;
            }
            last_cmd = cmd;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // non-blocking loop
    }
}
/* ─── public API ──────────────────────────────────────────── */

void motor_init(void)
{
    gpio_out(PIN_ENA);  gpio_out(PIN_ENB);
    gpio_out(PIN_IN1);  gpio_out(PIN_IN2);
    gpio_out(PIN_IN3);  gpio_out(PIN_IN4);
    gpio_out(PIN_TAIL_LIGHT);
    gpio_out(PIN_RELAY_PUMP);
    gpio_out(PIN_RELAY_BLOWER);

    set_pins(0, 0, 0, 0, 0, 0);
    gpio_set_level(PIN_TAIL_LIGHT, 0);
    relay_write(PIN_RELAY_PUMP,   false);
    relay_write(PIN_RELAY_BLOWER, false);

    // INITIAL STATE
    s_cmd = DIR_STOP;

    xTaskCreatePinnedToCore(motor_task, "motor_task",
                            2048, NULL, 10, NULL, 1);

    ESP_LOGI(TAG,
             "Motor init OK  ENA=%d ENB=%d IN1-4=%d,%d,%d,%d  "
             "PUMP=%d BLOW=%d  IR_blocking=%s",
             (int)PIN_ENA, (int)PIN_ENB,
             (int)PIN_IN1, (int)PIN_IN2,
             (int)PIN_IN3, (int)PIN_IN4,
             (int)PIN_RELAY_PUMP, (int)PIN_RELAY_BLOWER,
             IR_ENABLED ? "ON" : "OFF");
}
void motor_send_cmd(motor_dir_t dir)
{
    s_cmd = dir;
}

void motor_stop_immediate(void)
{
    if (s_motor_queue) xQueueReset(s_motor_queue);
    do_stop();
}

motor_dir_t motor_get_direction(void) { return s_current_dir; }

const char *motor_dir_to_str(motor_dir_t dir)
{
    switch (dir) {
        case DIR_FORWARD:     return "FORWARD";
        case DIR_BACKWARD:    return "BACKWARD";
        case DIR_LEFT:        return "LEFT";
        case DIR_RIGHT:       return "RIGHT";
        case DIR_DRIFT_LEFT:  return "DRIFT_LEFT";
        case DIR_DRIFT_RIGHT: return "DRIFT_RIGHT";
        default:              return "STOP";
    }
}

void motor_set_ramming(bool en)
{
    s_ramming = en;
    ESP_LOGI(TAG, "Ramming: %s", en ? "ON" : "OFF");
}
bool motor_get_ramming(void) { return s_ramming; }

void motor_set_obstacles(bool front, bool back)
{
    s_front_blocked = front;
    s_back_blocked  = back;
#if IR_ENABLED
    if (!s_ramming) {
        if (front && s_current_dir == DIR_FORWARD)  motor_stop_immediate();
        if (back  && s_current_dir == DIR_BACKWARD) motor_stop_immediate();
    }
#endif
}

/* ─── relay peripherals ───────────────────────────────────── */

void pump_set(bool on)
{
    s_pump_on = on;
    relay_write(PIN_RELAY_PUMP, on);
    ESP_LOGI(TAG, "Pump: %s", on ? "ON" : "OFF");
}
bool pump_get(void) { return s_pump_on; }

void blower_set(bool on)
{
    s_blower_on = on;
    relay_write(PIN_RELAY_BLOWER, on);
    ESP_LOGI(TAG, "Blower: %s", on ? "ON" : "OFF");
}
bool blower_get(void) { return s_blower_on; }

/* ─── Simple blocking move (time-based) ──────────────────── */

bool motor_move_blocking(motor_dir_t dir, int duration_ms)
{
    motor_send_cmd(dir);

    const int POLL_MS = 10;
    int elapsed = 0;
    while (elapsed < duration_ms) {
#if IR_ENABLED
        if (!s_ramming) {
            if (dir == DIR_FORWARD  && s_front_blocked) {
                motor_stop_immediate();
                ESP_LOGW(TAG, "move_blocking: FRONT IR blocked after %d ms", elapsed);
                return false;
            }
            if (dir == DIR_BACKWARD && s_back_blocked) {
                motor_stop_immediate();
                ESP_LOGW(TAG, "move_blocking: BACK IR blocked after %d ms", elapsed);
                return false;
            }
        }
#endif
        int slice = ((duration_ms - elapsed) < POLL_MS)
                        ? (duration_ms - elapsed) : POLL_MS;
        vTaskDelay(pdMS_TO_TICKS(slice));
        elapsed += slice;
    }
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(MS_STABILISE));
    return true;
}
bool motor_move_straight(motor_dir_t dir,
                          int         duration_ms,
                          float       target_heading_deg,
                          float       correction_kp)
{
#if MPU_ENABLED
    const int   POLL_MS  = 20;
    const float DEADBAND = 3.0f;   // relaxed
    int elapsed = 0;

    motor_send_cmd(dir);

    while (elapsed < duration_ms) {

#if IR_ENABLED
        if (!s_ramming) {
            if (dir == DIR_FORWARD  && s_front_blocked) {
                motor_stop_immediate();
                return false;
            }
            if (dir == DIR_BACKWARD && s_back_blocked) {
                motor_stop_immediate();
                return false;
            }
        }
#endif

        float cur = mpu_heading();
        float err = target_heading_deg - cur;

        while (err > 180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;

        // ✅ SMOOTH CONTINUOUS CORRECTION (NO JERKS)
        if (fabsf(err) > DEADBAND) {

            if (dir == DIR_FORWARD) {
                if (err > 0)
                    motor_send_cmd(DIR_DRIFT_LEFT);
                else
                    motor_send_cmd(DIR_DRIFT_RIGHT);
            } else {
                if (err > 0)
                    motor_send_cmd(DIR_DRIFT_RIGHT);
                else
                    motor_send_cmd(DIR_DRIFT_LEFT);
            }

        } else {
            motor_send_cmd(dir);  // straight
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        elapsed += POLL_MS;
    }

    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(MS_STABILISE));
    return true;

#else
    return motor_move_blocking(dir, duration_ms);
#endif
}