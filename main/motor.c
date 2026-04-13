/*
 * motor.c  —  L298N motor driver + relay peripherals
 *
 * Obstacle blocking behaviour is conditional on IR_ENABLED:
 *
 *   IR_ENABLED 0  →  s_front_blocked and s_back_blocked are never
 *                    set (they stay false forever).  motor_set_obstacles()
 *                    becomes a no-op.  Movement is never blocked by IR.
 *
 *   IR_ENABLED 1  →  s_front_blocked / s_back_blocked start as false.
 *                    The ir_task calls motor_set_obstacles() after its
 *                    debounce count is satisfied.  do_forward() /
 *                    do_backward() check the flags and stop if blocked
 *                    (unless ramming mode is on).
 *
 * This means: when IR is disabled in config.h the car moves freely.
 * When IR is enabled the car stops only when a sensor actually sees
 * an obstacle (debounced).  The initial state is "clear" so the car
 * can move immediately at boot; the IR task will raise the flag within
 * IR_DEBOUNCE_COUNT × IR_POLL_INTERVAL_MS (≤ 150 ms) if something is
 * actually there.
 */

#include "motor.h"
#include "config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "MOTOR";

/* ─── state ───────────────────────────────────────────────── */
static volatile motor_dir_t s_current_dir   = DIR_STOP;
static volatile bool        s_ramming       = false;
static volatile bool        s_accident      = false;
static volatile bool        s_pump_on       = false;
static volatile bool        s_blower_on     = false;

/*
 * Obstacle flags — always start false (path assumed clear).
 * Only ever written by motor_set_obstacles(), which is only called
 * from ir_task when IR_ENABLED == 1.  When IR_ENABLED == 0,
 * motor_set_obstacles() is a compile-time no-op so these never change.
 */
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

/* ─── raw drive (called only from motor_task) ─────────────── */

static void do_stop(void)
{
    set_pins(0, 0, 0, 0, 0, 0);
    s_current_dir = DIR_STOP;
}

static void do_forward(void)
{
    if (s_accident) { do_stop(); return; }
#if IR_ENABLED
    if (!s_ramming && s_front_blocked) {
        ESP_LOGW(TAG, "Forward blocked by IR (front)");
        do_stop(); return;
    }
#endif
    set_pins(0, 1, 1, 0, 1, 1);
    s_current_dir = DIR_FORWARD;
}

static void do_backward(void)
{
    if (s_accident) { do_stop(); return; }
#if IR_ENABLED
    if (!s_ramming && s_back_blocked) {
        ESP_LOGW(TAG, "Backward blocked by IR (back)");
        do_stop(); return;
    }
#endif
    set_pins(1, 0, 0, 1, 1, 1);
    s_current_dir = DIR_BACKWARD;
}

static void do_left(void)
{
    if (s_accident) { do_stop(); return; }
    set_pins(1, 0, 1, 0, 1, 1);
    s_current_dir = DIR_LEFT;
}

static void do_right(void)
{
    if (s_accident) { do_stop(); return; }
    set_pins(0, 1, 0, 1, 1, 1);
    s_current_dir = DIR_RIGHT;
}

static void do_drift_left(void)
{
    if (s_accident) { do_stop(); return; }
    set_pins(0, 0, 1, 0, 1, 1);
    s_current_dir = DIR_DRIFT_LEFT;
}

static void do_drift_right(void)
{
    if (s_accident) { do_stop(); return; }
    set_pins(0, 1, 0, 0, 1, 1);
    s_current_dir = DIR_DRIFT_RIGHT;
}

/* ─── motor task ──────────────────────────────────────────── */

static void motor_task(void *arg)
{
    (void)arg;
    do_stop();
    while (1) {
        motor_cmd_t cmd;
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

/* ─── public API ──────────────────────────────────────────── */

void motor_init(void)
{
    gpio_out(PIN_ENA); gpio_out(PIN_ENB);
    gpio_out(PIN_IN1); gpio_out(PIN_IN2);
    gpio_out(PIN_IN3); gpio_out(PIN_IN4);
    gpio_out(PIN_TAIL_LIGHT);
    gpio_out(PIN_RELAY_PUMP);
    gpio_out(PIN_RELAY_BLOWER);

    set_pins(0, 0, 0, 0, 0, 0);
    gpio_set_level(PIN_TAIL_LIGHT, 0);
    relay_write(PIN_RELAY_PUMP,   false);
    relay_write(PIN_RELAY_BLOWER, false);

    s_motor_queue = xQueueCreate(1, sizeof(motor_cmd_t));
    configASSERT(s_motor_queue);

    xTaskCreatePinnedToCore(motor_task, "motor_task",
                            2048, NULL, 10, NULL, 1);

    ESP_LOGI(TAG,
             "Motor init OK  ENA=%d ENB=%d IN1-4=%d,%d,%d,%d  "
             "PUMP=%d BLOW=%d  IR_obstacle_blocking=%s",
             (int)PIN_ENA, (int)PIN_ENB,
             (int)PIN_IN1, (int)PIN_IN2, (int)PIN_IN3, (int)PIN_IN4,
             (int)PIN_RELAY_PUMP, (int)PIN_RELAY_BLOWER,
             IR_ENABLED ? "ENABLED" : "DISABLED");
}

void motor_send_cmd(motor_dir_t dir)
{
    if (!s_motor_queue) return;
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

void motor_set_accident(bool en)
{
    s_accident = en;
    if (en) {
        motor_stop_immediate();
        pump_set(false);
        blower_set(false);
        ESP_LOGW(TAG, "ACCIDENT SET — all motion blocked, peripherals off");
    } else {
        ESP_LOGI(TAG, "Accident cleared");
    }
}
bool motor_get_accident(void) { return s_accident; }

/*
 * motor_set_obstacles() — only called by ir_task when IR_ENABLED == 1.
 * When IR_ENABLED == 0 this function still compiles (no #if guard here)
 * but ir_task itself is compiled out, so it is never called.
 * The do_forward() / do_backward() checks are inside #if IR_ENABLED
 * blocks, so the flags have no effect even if someone calls this manually.
 */
void motor_set_obstacles(bool front, bool back)
{
    s_front_blocked = front;
    s_back_blocked  = back;

#if IR_ENABLED
    if (!s_ramming) {
        if (front && s_current_dir == DIR_FORWARD)  motor_stop_immediate();
        if (back  && s_current_dir == DIR_BACKWARD) motor_stop_immediate();
    }
    ESP_LOGD(TAG, "IR obstacles: front=%d back=%d", (int)front, (int)back);
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

/* ─── blocking move ───────────────────────────────────────── */

bool motor_move_blocking(motor_dir_t dir, int duration_ms)
{
    if (s_accident) {
        ESP_LOGW(TAG, "Blocking move refused — accident set");
        motor_stop_immediate();
        return false;
    }
    motor_send_cmd(dir);

    const int POLL_MS = 10;
    int elapsed = 0;
    while (elapsed < duration_ms) {
        if (s_accident) {
            motor_stop_immediate();
            return false;
        }
        int slice = ((duration_ms - elapsed) < POLL_MS)
                        ? (duration_ms - elapsed) : POLL_MS;
        vTaskDelay(pdMS_TO_TICKS(slice));
        elapsed += slice;
    }
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(MS_STABILISE));
    return !s_accident;
}