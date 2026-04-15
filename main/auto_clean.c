#include "auto_clean.h"
#include "config.h"
#include "motor.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "AUTO";
#define TURN_90_MS 700   // ⚠️ TUNE THIS
/* ─── module state ────────────────────────────────────────── */

static clean_grid_t s_grid = {
    .panel_cols      = 2,
    .panel_rows      = 1,
    .panel_w_cm      = 200,
    .panel_h_cm      = 170,
    .gap_between_cm  = 5,
    .wash_enabled    = true,
    .blow_enabled    = false,
};

static volatile clean_state_t s_state          = CLEAN_IDLE;
static volatile bool          s_stop_requested = false;
static volatile int           s_strips_done    = 0;
static volatile int           s_strips_total   = 0;

static float s_base_heading = 0.0f;  /* heading at start of clean */

/* ─── Grid API ────────────────────────────────────────────── */

void auto_clean_set_grid(const clean_grid_t *g)
{
    s_grid = *g;
    if (s_grid.panel_cols < 1)       s_grid.panel_cols      = 1;
    if (s_grid.panel_cols > 10)      s_grid.panel_cols      = 10;
    if (s_grid.panel_rows < 1)       s_grid.panel_rows      = 1;
    if (s_grid.panel_rows > 10)      s_grid.panel_rows      = 10;
    if (s_grid.panel_w_cm < 20)      s_grid.panel_w_cm      = 20;
    if (s_grid.panel_h_cm < 20)      s_grid.panel_h_cm      = 20;
    if (s_grid.gap_between_cm < 0)   s_grid.gap_between_cm  = 0;
    if (s_grid.gap_between_cm > 200) s_grid.gap_between_cm  = 200;
}

void auto_clean_get_grid(clean_grid_t *g)   { *g = s_grid; }
clean_state_t auto_clean_state(void)        { return s_state; }
bool  auto_clean_is_running(void)           { return s_state == CLEAN_RUNNING; }
int   auto_clean_strips_done(void)          { return s_strips_done; }
int   auto_clean_strips_total(void)         { return s_strips_total; }
int   auto_clean_pct(void)
{
    if (s_strips_total <= 0) return 0;
    int p = (s_strips_done * 100) / s_strips_total;
    return (p > 100) ? 100 : p;
}

/* ─── Heading helpers ─────────────────────────────────────── */

static float norm_heading(float h)
{
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;
    return h;
}

static float heading_error(float target, float current)
{
    float e = target - current;
    while (e >  180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    return e;
}

static bool correct_heading_to(float target_deg)
{
    if (!mpu_is_ready()) return true;

    const int POLL_MS = 30;
    int elapsed = 0;

    while (elapsed < HEADING_CORRECT_MAX_MS) {

        if (s_stop_requested) {
            motor_stop_immediate();
            return false;
        }

        float cur = mpu_heading();
        float err = heading_error(target_deg, cur);

        if (fabsf(err) <= 5.0f) {
            motor_send_cmd(DIR_STOP);
            vTaskDelay(pdMS_TO_TICKS(150));
            return true;
        }

        if (err > 0)
            motor_send_cmd(DIR_RIGHT);
        else
            motor_send_cmd(DIR_LEFT);

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        elapsed += POLL_MS;
    }

    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(150));
    return true;
}

/*
 * timed_turn_then_correct():
 *   Issue a partial timed spin (70% of expected_ms to reduce overshoot),
 *   stop, stabilise, then fine-correct via gyro feedback.
 */
static bool timed_turn_then_correct(motor_dir_t spin_dir,
                                    int expected_ms,
                                    float target_deg)
{
    motor_send_cmd(spin_dir);
    vTaskDelay(pdMS_TO_TICKS((int)(expected_ms * 0.7f)));
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(200));   /* let gyro settle */
    return correct_heading_to(target_deg);
}

/* ─── Stop-aware move wrappers ────────────────────────────── */

static bool safe_move_straight(motor_dir_t dir, int duration_ms, float hdg)
{
    if (s_stop_requested) return false;
    return motor_move_straight(dir, duration_ms, hdg, 5.0f);
}

static bool safe_move_blocking(motor_dir_t dir, int duration_ms)
{
    if (s_stop_requested) return false;
    return motor_move_blocking(dir, duration_ms);
}

static bool move_step_sequence(motor_dir_t dir, int duration)
{
    if (s_stop_requested) return false;

    motor_send_cmd(dir);
    vTaskDelay(pdMS_TO_TICKS(duration));

    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(200));  // stabilize

    return true;
}


static void auto_clean_task(void *arg)
{
    (void)arg;
    const clean_grid_t g = s_grid;

    const int strips =
        (g.panel_h_cm + ROVER_WIDTH_CM - 1) / ROVER_WIDTH_CM;

    const int forward_ms = g.panel_w_cm * MS_PER_CM;
    const int step_ms    = ROVER_WIDTH_CM * MS_PER_CM;

    s_state = CLEAN_RUNNING;

    bool go_right = true;

    for (int i = 0; i < strips && !s_stop_requested; i++) {

        /* ===============================
           1. MAIN FORWARD STRIP
           =============================== */
        if (!move_step_sequence(DIR_FORWARD, forward_ms)) break;

        s_strips_done++;

        if (i == strips - 1) break;

        /* ===============================
           2. EDGE TURN + STEP
           =============================== */

        if (go_right) {
            // RIGHT EDGE LOGIC

            if (!move_step_sequence(DIR_LEFT, TURN_90_MS)) break;
            if (!move_step_sequence(DIR_FORWARD, step_ms)) break;
            if (!move_step_sequence(DIR_LEFT, TURN_90_MS)) break;

        } else {
            // LEFT EDGE LOGIC

            if (!move_step_sequence(DIR_RIGHT, TURN_90_MS)) break;
            if (!move_step_sequence(DIR_FORWARD, step_ms)) break;
            if (!move_step_sequence(DIR_RIGHT, TURN_90_MS)) break;
        }

        /* Flip direction */
        go_right = !go_right;
    }

    motor_send_cmd(DIR_STOP);
    pump_set(false);
    blower_set(false);

    s_state = CLEAN_DONE;
    vTaskDelete(NULL);
}
bool auto_clean_start(void)
{
    if (s_state == CLEAN_RUNNING) {
        ESP_LOGW(TAG, "auto_clean_start: already running");
        return false;
    }

    s_stop_requested = false;
    s_strips_done    = 0;

    mpu_reset_heading();
    s_base_heading = 0.0f;

    gpio_set_level(PIN_TAIL_LIGHT, 1);

    BaseType_t rc = xTaskCreate(auto_clean_task, "auto_clean",
                                5120, NULL, 5, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create auto_clean task");
        s_state = CLEAN_ERROR;
        gpio_set_level(PIN_TAIL_LIGHT, 0);
        return false;
    }

    return true;
}

void auto_clean_stop(void)
{
    if (s_state == CLEAN_RUNNING) {
        s_stop_requested = true;
        motor_stop_immediate();
        pump_set(false);
        blower_set(false);
        ESP_LOGW(TAG, "Stop requested");
    }
}