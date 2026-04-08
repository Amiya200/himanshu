#include "motor.h"
#include "config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MOTOR";

// ================== STATE ==================
static volatile motor_dir_t s_current_dir   = DIR_STOP;
static volatile bool        s_ramming       = false;
static volatile bool        s_accident      = false;
static volatile bool        s_front_blocked = false;
static volatile bool        s_back_blocked  = false;
static volatile bool        s_pump_on       = false;
static volatile bool        s_blower_on     = false;

static QueueHandle_t s_motor_queue = NULL;

// ================== GPIO HELPERS ==================

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

// Active-LOW relay: LOW = relay energised = device ON
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

// ================== RAW DRIVE (motor task only) ==================

static void do_stop(void)
{
    set_pins(0, 0, 0, 0, 0, 0);
    s_current_dir = DIR_STOP;
}

/*
 *  L298N wiring (left motor IN1/IN2/ENA, right motor IN3/IN4/ENB):
 *
 *  FORWARD      : Left  CW   (IN1=0, IN2=1)
 *                 Right CCW  (IN3=1, IN4=0)  ← mirrored mounting
 *
 *  BACKWARD     : Left  CCW  (IN1=1, IN2=0)
 *                 Right CW   (IN3=0, IN4=1)
 *
 *  LEFT  (spin) : Left  backward, Right forward
 *  RIGHT (spin) : Left  forward,  Right backward
 *
 *  DRIFT_LEFT   : Right motor only → gentle left curve
 *  DRIFT_RIGHT  : Left  motor only → gentle right curve
 *
 *  Adjust IN1/IN2/IN3/IN4 logic if your motors spin the wrong way.
 */

static void do_forward(void)
{
    if (s_accident) { do_stop(); return; }
    if (!s_ramming && s_front_blocked) {
        ESP_LOGW(TAG, "Front blocked — cannot move forward");
        do_stop(); return;
    }
    set_pins(0, 1, 1, 0, 1, 1);
    s_current_dir = DIR_FORWARD;
    ESP_LOGI(TAG, "FORWARD");
}

static void do_backward(void)
{
    if (s_accident) { do_stop(); return; }
    if (!s_ramming && s_back_blocked) {
        ESP_LOGW(TAG, "Back blocked — cannot move backward");
        do_stop(); return;
    }
    set_pins(1, 0, 0, 1, 1, 1);
    s_current_dir = DIR_BACKWARD;
    ESP_LOGI(TAG, "BACKWARD");
}

static void do_left(void)
{
    if (s_accident) { do_stop(); return; }
    set_pins(1, 0, 1, 0, 1, 1);
    s_current_dir = DIR_LEFT;
    ESP_LOGI(TAG, "LEFT");
}

static void do_right(void)
{
    if (s_accident) { do_stop(); return; }
    set_pins(0, 1, 0, 1, 1, 1);
    s_current_dir = DIR_RIGHT;
    ESP_LOGI(TAG, "RIGHT");
}

static void do_drift_left(void)
{
    if (s_accident) { do_stop(); return; }
    // Right motor forward only
    set_pins(0, 0, 1, 0, 1, 1);
    s_current_dir = DIR_DRIFT_LEFT;
    ESP_LOGI(TAG, "DRIFT LEFT");
}

static void do_drift_right(void)
{
    if (s_accident) { do_stop(); return; }
    // Left motor forward only
    set_pins(0, 1, 0, 0, 1, 1);
    s_current_dir = DIR_DRIFT_RIGHT;
    ESP_LOGI(TAG, "DRIFT RIGHT");
}

// ================== MOTOR TASK ==================

static void motor_task(void *arg)
{
    motor_cmd_t cmd;
    do_stop();

    while (1) {
        if (xQueueReceive(s_motor_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            switch (cmd.dir) {
                case DIR_FORWARD:     do_forward();     break;
                case DIR_BACKWARD:    do_backward();    break;
                case DIR_LEFT:        do_left();        break;
                case DIR_RIGHT:       do_right();       break;
                case DIR_DRIFT_LEFT:  do_drift_left();  break;
                case DIR_DRIFT_RIGHT: do_drift_right(); break;
                default:              do_stop();        break;
            }
        }
    }
}

// ================== PUBLIC API ==================

void motor_init(void)
{
    // Motor driver GPIO outputs
    gpio_out(PIN_ENA);
    gpio_out(PIN_ENB);
    gpio_out(PIN_IN1);
    gpio_out(PIN_IN2);
    gpio_out(PIN_IN3);
    gpio_out(PIN_IN4);
    gpio_out(PIN_TAIL_LIGHT);

    // Relay GPIOs — drive HIGH first (relay OFF = safe default)
    gpio_out(PIN_RELAY_PUMP);
    gpio_out(PIN_RELAY_BLOWER);
    relay_write(PIN_RELAY_PUMP,   false);
    relay_write(PIN_RELAY_BLOWER, false);

    // All motor pins LOW, tail light OFF
    set_pins(0, 0, 0, 0, 0, 0);
    gpio_set_level(PIN_TAIL_LIGHT, 0);

    s_motor_queue = xQueueCreate(1, sizeof(motor_cmd_t));

    xTaskCreatePinnedToCore(
        motor_task, "motor_task", 2048, NULL,
        10, NULL, 1);

    ESP_LOGI(TAG, "Motor driver initialized — ENA=%d ENB=%d IN1=%d IN2=%d IN3=%d IN4=%d",
             PIN_ENA, PIN_ENB, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4);
}

void motor_send_cmd(motor_dir_t dir)
{
    if (s_motor_queue == NULL) return;
    motor_cmd_t cmd = { .dir = dir };
    xQueueOverwrite(s_motor_queue, &cmd);
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
        case DIR_DRIFT_LEFT:  return "DRIFT LEFT";
        case DIR_DRIFT_RIGHT: return "DRIFT RIGHT";
        default:              return "STOP";
    }
}

void motor_set_ramming(bool en)
{
    s_ramming = en;
    ESP_LOGI(TAG, "Ramming mode: %s", en ? "ON" : "OFF");
}
bool motor_get_ramming(void) { return s_ramming; }

void motor_set_accident(bool en)
{
    s_accident = en;
    if (en) {
        motor_stop_immediate();
        ESP_LOGW(TAG, "Accident flag SET - all movement blocked");
    } else {
        ESP_LOGI(TAG, "Accident flag CLEARED");
    }
}
bool motor_get_accident(void) { return s_accident; }

void motor_set_obstacles(bool front, bool back)
{
    s_front_blocked = front;
    s_back_blocked  = back;

    if (!s_ramming) {
        motor_dir_t cur = s_current_dir;
        if (front && cur == DIR_FORWARD)  motor_stop_immediate();
        if (back  && cur == DIR_BACKWARD) motor_stop_immediate();
    }
}

// ================== RELAY PERIPHERAL API ==================

void pump_set(bool on)
{
    s_pump_on = on;
    relay_write(PIN_RELAY_PUMP, on);
    ESP_LOGI(TAG, "Water pump: %s", on ? "ON" : "OFF");
}
bool pump_get(void) { return s_pump_on; }

void blower_set(bool on)
{
    s_blower_on = on;
    relay_write(PIN_RELAY_BLOWER, on);
    ESP_LOGI(TAG, "Air blower: %s", on ? "ON" : "OFF");
}
bool blower_get(void) { return s_blower_on; }

// ================== AUTO MODE ==================

void motor_move_blocking(motor_dir_t dir, int duration_ms)
{
    if (s_accident) {
        ESP_LOGW(TAG, "AUTO MOVE BLOCKED: ACCIDENT");
        motor_stop_immediate();
        return;
    }
    motor_send_cmd(dir);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(MS_STABILISE));
}