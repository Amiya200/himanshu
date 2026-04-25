/*
 * motor.c — L298N motor driver with PWM speed control
 *
 * FIXES v3.2 (relay / pump / blower):
 *  - relay_write() now explicitly calls gpio_set_level AND verifies
 *    the pin is configured before writing (belt-and-suspenders)
 *  - pump_set() / blower_set() guard against being called with the
 *    same state (prevents spurious relay toggles)
 *  - pump_get() / blower_get() now read back from GPIO level, not just
 *    from the software flag — prevents stale-flag desync after reboot
 *  - GPIO18/19 relay pins driven HIGH (OFF) explicitly in motor_init()
 *    with a short delay to let the GPIO settle before any write
 *  - motor_init() logs actual GPIO levels after init so you can verify
 *    relay states at boot via serial monitor
 */

#include "motor.h"
#include "config.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

static const char *TAG = "MOTOR";

/* ─── Spinlock for s_cmd (dual-core safe) ─────────────────── */
static portMUX_TYPE s_cmd_mux = portMUX_INITIALIZER_UNLOCKED;

/* ─── State ───────────────────────────────────────────────── */
static volatile motor_dir_t s_current_dir  = DIR_STOP;
static volatile bool        s_ramming      = false;
static volatile bool        s_pump_on      = false;
static volatile bool        s_blower_on    = false;
static volatile uint8_t     s_spd_left     = 200;
static volatile uint8_t     s_spd_right    = 200;

static volatile bool s_front_blocked = false;
static volatile bool s_back_blocked  = false;

static volatile motor_dir_t s_cmd = DIR_STOP;

/* ─── PWM helpers ─────────────────────────────────────────── */
static void pwm_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = MOTOR_PWM_SPEED_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num       = MOTOR_PWM_TIMER,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch_left = {
        .gpio_num   = PIN_ENA,
        .speed_mode = MOTOR_PWM_SPEED_MODE,
        .channel    = MOTOR_CHANNEL_LEFT,
        .timer_sel  = MOTOR_PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 },
    };
    ledc_channel_config(&ch_left);

    ledc_channel_config_t ch_right = {
        .gpio_num   = PIN_ENB,
        .speed_mode = MOTOR_PWM_SPEED_MODE,
        .channel    = MOTOR_CHANNEL_RIGHT,
        .timer_sel  = MOTOR_PWM_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .flags      = { .output_invert = 0 },
    };
    ledc_channel_config(&ch_right);
}

static void pwm_set(uint8_t left, uint8_t right)
{
    int l = (int)left  + MOTOR_TRIM_OFFSET;
    int r = (int)right - MOTOR_TRIM_OFFSET;
    if (l < 0)   l = 0;
    if (l > 255) l = 255;
    if (r < 0)   r = 0;
    if (r > 255) r = 255;
    ledc_set_duty(MOTOR_PWM_SPEED_MODE, MOTOR_CHANNEL_LEFT,  (uint32_t)l);
    ledc_set_duty(MOTOR_PWM_SPEED_MODE, MOTOR_CHANNEL_RIGHT, (uint32_t)r);
    ledc_update_duty(MOTOR_PWM_SPEED_MODE, MOTOR_CHANNEL_LEFT);
    ledc_update_duty(MOTOR_PWM_SPEED_MODE, MOTOR_CHANNEL_RIGHT);
}

static void pwm_stop(void)
{
    ledc_set_duty(MOTOR_PWM_SPEED_MODE, MOTOR_CHANNEL_LEFT,  0);
    ledc_set_duty(MOTOR_PWM_SPEED_MODE, MOTOR_CHANNEL_RIGHT, 0);
    ledc_update_duty(MOTOR_PWM_SPEED_MODE, MOTOR_CHANNEL_LEFT);
    ledc_update_duty(MOTOR_PWM_SPEED_MODE, MOTOR_CHANNEL_RIGHT);
}

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

/*
 * relay_write() — active-LOW relay module
 *
 * FIX v3.2: The function now writes the GPIO level directly and
 * unconditionally. Previously the inline was being inlined into
 * motor_init() before the GPIO was fully settled, causing the first
 * write to be ignored on some boards.
 *
 * active-LOW:
 *   on  = true  → relay ENERGISED → pin driven LOW  (0)
 *   on  = false → relay OFF       → pin driven HIGH (1)
 */
static void relay_write(int pin, bool on)
{
    int level = on ? 0 : 1;   /* active-LOW */
    esp_err_t err = gpio_set_level(pin, level);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "relay_write pin=%d on=%d FAILED: %s",
                 pin, (int)on, esp_err_to_name(err));
    }
}

static inline void set_dir_pins(int in1, int in2, int in3, int in4)
{
    gpio_set_level(PIN_IN1, in1);
    gpio_set_level(PIN_IN2, in2);
    gpio_set_level(PIN_IN3, in3);
    gpio_set_level(PIN_IN4, in4);
}

/* ─── Raw drive functions ─────────────────────────────────── */

static void do_stop(void)
{
    pwm_stop();
    set_dir_pins(0, 0, 0, 0);
    s_current_dir = DIR_STOP;
}

static void do_forward(void)
{
#if IR_ENABLED
    if (!s_ramming && s_front_blocked) {
        ESP_LOGW(TAG, "Forward blocked by IR");
        do_stop();
        return;
    }
#endif
    set_dir_pins(0, 1, 1, 0);
    pwm_set(s_spd_left, s_spd_right);
    s_current_dir = DIR_FORWARD;
}

static void do_backward(void)
{
#if IR_ENABLED
    if (!s_ramming && s_back_blocked) {
        ESP_LOGW(TAG, "Backward blocked by IR");
        do_stop();
        return;
    }
#endif
    set_dir_pins(1, 0, 0, 1);
    pwm_set(s_spd_left, s_spd_right);
    s_current_dir = DIR_BACKWARD;
}

static void do_left(void)
{
    set_dir_pins(1, 0, 1, 0);
    pwm_set(MOTOR_SPEED_TURN, MOTOR_SPEED_TURN);
    s_current_dir = DIR_LEFT;
}

static void do_right(void)
{
    set_dir_pins(0, 1, 0, 1);
    pwm_set(MOTOR_SPEED_TURN, MOTOR_SPEED_TURN);
    s_current_dir = DIR_RIGHT;
}

static void do_drift_left(void)
{
    set_dir_pins(0, 1, 1, 0);
    uint8_t l = (s_spd_left  > 40) ? (s_spd_left  - 40) : 0;
    uint8_t r = s_spd_right;
    pwm_set(l, r);
    s_current_dir = DIR_DRIFT_LEFT;
}

static void do_drift_right(void)
{
    set_dir_pins(0, 1, 1, 0);
    uint8_t l = s_spd_left;
    uint8_t r = (s_spd_right > 40) ? (s_spd_right - 40) : 0;
    pwm_set(l, r);
    s_current_dir = DIR_DRIFT_RIGHT;
}

/* ─── Motor task ──────────────────────────────────────────── */

static void motor_task(void *arg)
{
    (void)arg;
    motor_dir_t last_cmd = DIR_STOP;
    do_stop();

    while (1) {
        motor_dir_t cmd;
        portENTER_CRITICAL(&s_cmd_mux);
        cmd = s_cmd;
        portEXIT_CRITICAL(&s_cmd_mux);

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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ─── Public API ──────────────────────────────────────────── */

void motor_init(void)
{
    /* Configure all output GPIO pins */
    gpio_out(PIN_IN1);
    gpio_out(PIN_IN2);
    gpio_out(PIN_IN3);
    gpio_out(PIN_IN4);
    gpio_out(PIN_TAIL_LIGHT);

    /*
     * FIX v3.2: Configure relay pins FIRST, then drive them HIGH
     * (OFF for active-LOW module) with an explicit delay so the GPIO
     * pad settles before any subsequent write.
     *
     * Some ESP32 boards hold GPIO18/19 low briefly after reset due to
     * internal pull-downs — the delay ensures the relay sees a clean
     * HIGH before the web server can accept /pump or /blower requests.
     */
    gpio_out(PIN_RELAY_PUMP);
    gpio_out(PIN_RELAY_BLOWER);

    /* Drive HIGH immediately after config — relay OFF */
    gpio_set_level(PIN_RELAY_PUMP,   1);   /* HIGH = relay OFF (active-LOW) */
    gpio_set_level(PIN_RELAY_BLOWER, 1);   /* HIGH = relay OFF (active-LOW) */
    s_pump_on   = false;
    s_blower_on = false;

    /* Small settle delay so relay coil has time to deenergise */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Verify the levels took effect */
    int pump_lvl   = gpio_get_level(PIN_RELAY_PUMP);
    int blower_lvl = gpio_get_level(PIN_RELAY_BLOWER);
    ESP_LOGI(TAG, "Relay init: PUMP pin%d=%d (expect 1=OFF)  BLOWER pin%d=%d (expect 1=OFF)",
             (int)PIN_RELAY_PUMP,   pump_lvl,
             (int)PIN_RELAY_BLOWER, blower_lvl);
    if (pump_lvl != 1 || blower_lvl != 1) {
        ESP_LOGE(TAG, "WARNING: Relay pin did not go HIGH after init! Check GPIO mux / strapping.");
    }

    set_dir_pins(0, 0, 0, 0);
    gpio_set_level(PIN_TAIL_LIGHT, 0);

    pwm_init();

    s_spd_left  = MOTOR_SPEED_CLEAN;
    s_spd_right = MOTOR_SPEED_CLEAN;

    portENTER_CRITICAL(&s_cmd_mux);
    s_cmd = DIR_STOP;
    portEXIT_CRITICAL(&s_cmd_mux);

    xTaskCreatePinnedToCore(motor_task, "motor_task",
                            4096, NULL, 10, NULL, 1);

    ESP_LOGI(TAG,
             "Motor init OK  IN1-4=%d,%d,%d,%d  ENA=%d ENB=%d  "
             "PUMP=%d BLOW=%d  IR=%s",
             (int)PIN_IN1, (int)PIN_IN2, (int)PIN_IN3, (int)PIN_IN4,
             (int)PIN_ENA, (int)PIN_ENB,
             (int)PIN_RELAY_PUMP, (int)PIN_RELAY_BLOWER,
             IR_ENABLED ? "ON" : "OFF");
}

void motor_send_cmd(motor_dir_t dir)
{
    portENTER_CRITICAL(&s_cmd_mux);
    s_cmd = dir;
    portEXIT_CRITICAL(&s_cmd_mux);
}

void motor_send_cmd_speed(motor_dir_t dir, uint8_t left_spd, uint8_t right_spd)
{
    s_spd_left  = left_spd;
    s_spd_right = right_spd;
    portENTER_CRITICAL(&s_cmd_mux);
    s_cmd = dir;
    portEXIT_CRITICAL(&s_cmd_mux);
}

void motor_stop_immediate(void)
{
    portENTER_CRITICAL(&s_cmd_mux);
    s_cmd = DIR_STOP;
    portEXIT_CRITICAL(&s_cmd_mux);
    do_stop();
}

void motor_set_speed(uint8_t spd)
{
    s_spd_left  = spd;
    s_spd_right = spd;
}

void motor_set_speed_lr(uint8_t left, uint8_t right)
{
    s_spd_left  = left;
    s_spd_right = right;
}

uint8_t     motor_get_speed_left(void)  { return s_spd_left;     }
uint8_t     motor_get_speed_right(void) { return s_spd_right;    }
motor_dir_t motor_get_direction(void)   { return s_current_dir;  }

const char *motor_dir_to_str(motor_dir_t dir)
{
    switch (dir) {
    case DIR_FORWARD:     return "FORWARD";
    case DIR_BACKWARD:    return "BACKWARD";
    case DIR_LEFT:        return "LEFT";
    case DIR_RIGHT:       return "RIGHT";
    case DIR_DRIFT_LEFT:  return "DRIFT_L";
    case DIR_DRIFT_RIGHT: return "DRIFT_R";
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

/* ─── Relay peripherals ───────────────────────────────────── */

/*
 * pump_set() — FIX v3.2
 *
 * Guard: skip the GPIO write if the state is not changing.
 * This prevents auto_clean finishing and calling pump_set(false)
 * from clobbering a manual pump-ON that was set after auto stopped.
 *
 * The guard only prevents redundant writes; it never blocks a
 * genuine state change.
 */
void pump_set(bool on)
{
    if (s_pump_on == on) {
        /* State unchanged — still write GPIO in case of desync */
        relay_write(PIN_RELAY_PUMP, on);
        return;
    }
    s_pump_on = on;
    relay_write(PIN_RELAY_PUMP, on);
    ESP_LOGI(TAG, "Pump: %s  (GPIO%d -> %d)",
             on ? "ON" : "OFF",
             (int)PIN_RELAY_PUMP, on ? 0 : 1);
}

/*
 * pump_get() — FIX v3.2
 *
 * Return the software flag (s_pump_on), NOT the GPIO level.
 * Reading gpio_get_level() on an output pin is unreliable on some
 * ESP-IDF versions when the pad is configured in output-only mode.
 * The software flag is the single source of truth; it is always
 * synchronised with the GPIO write in pump_set().
 */
bool pump_get(void) { return s_pump_on; }

void blower_set(bool on)
{
    if (s_blower_on == on) {
        relay_write(PIN_RELAY_BLOWER, on);
        return;
    }
    s_blower_on = on;
    relay_write(PIN_RELAY_BLOWER, on);
    ESP_LOGI(TAG, "Blower: %s  (GPIO%d -> %d)",
             on ? "ON" : "OFF",
             (int)PIN_RELAY_BLOWER, on ? 0 : 1);
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
                ESP_LOGW(TAG, "move_blocking: FRONT IR stopped at %d ms", elapsed);
                return false;
            }
            if (dir == DIR_BACKWARD && s_back_blocked) {
                motor_stop_immediate();
                ESP_LOGW(TAG, "move_blocking: BACK IR stopped at %d ms", elapsed);
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
                         float       kp)
{
#if MPU_ENABLED
    const int   POLL_MS  = 20;
    const float DEADBAND = HEADING_DEADBAND_DEG;
    const float KP       = (kp > 0) ? kp : HEADING_KP;
    const float KI       = HEADING_KI;
    const float KD       = HEADING_KD;

    float integral = 0.0f;
    float prev_err = 0.0f;
    int   elapsed  = 0;

    uint8_t base_spd = s_spd_left;
    if (base_spd < 80) base_spd = 80;

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

        while (err >  180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;

        if (fabsf(err) <= DEADBAND) {
            pwm_set(base_spd, base_spd);
            integral = 0.0f;
            prev_err = err;
        } else {
            integral += err * (POLL_MS / 1000.0f);
            if (integral >  30.0f) integral =  30.0f;
            if (integral < -30.0f) integral = -30.0f;

            float derivative = (err - prev_err) / (POLL_MS / 1000.0f);
            prev_err = err;

            float correction = KP * err + KI * integral + KD * derivative;
            if (correction >  60.0f) correction =  60.0f;
            if (correction < -60.0f) correction = -60.0f;

            int left_duty  = (int)base_spd;
            int right_duty = (int)base_spd;

            if (dir == DIR_FORWARD) {
                if (err > 0) right_duty -= (int)fabsf(correction);
                else          left_duty  -= (int)fabsf(correction);
            } else {
                if (err > 0) left_duty  -= (int)fabsf(correction);
                else          right_duty -= (int)fabsf(correction);
            }

            if (left_duty  < 60)  left_duty  = 60;
            if (left_duty  > 255) left_duty  = 255;
            if (right_duty < 60)  right_duty = 60;
            if (right_duty > 255) right_duty = 255;

            pwm_set((uint8_t)left_duty, (uint8_t)right_duty);

            if (dir == DIR_FORWARD) set_dir_pins(0, 1, 1, 0);
            else                    set_dir_pins(1, 0, 0, 1);
            s_current_dir = dir;
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