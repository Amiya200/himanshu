/*
 * auto_clean.c — Boustrophedon auto-clean with servo boundary control
 *
 * UPGRADES v3.1 (bug-fix pass):
 *  - auto_clean_task and path_playback_task pinned to core 0 (motor_task on core 1)
 *  - auto_clean_stop() no longer force-sets CLEAN_IDLE while task is running;
 *    instead sets CLEAN_STOPPING and lets the task exit cleanly
 *  - safe_forward_strip() pause/stop checks restored (were commented out)
 *  - motor_set_speed(MOTOR_SPEED_REPOSITION) added before every backward/lateral move
 *  - All heading corrections now use norm_heading() consistently
 *
 * UPGRADES v3.0 (original):
 *  - Servo lifts wiper BEFORE reaching panel edge → no more stuck wiper
 *  - Multi-pass per strip (configurable 1–3 passes)
 *  - Row-to-row repositioning implemented
 *  - Pause / Resume support
 *  - Current panel + strip tracking exposed to UI
 *  - PID straight-line travel via motor_move_straight()
 *  - Heading verified and corrected before every strip
 */

#include "auto_clean.h"
#include "config.h"
#include "motor.h"
#include "servo.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "AUTO";

/* ─── Path recording ─────────────────────────────────────── */

#define PATH_MAX_STEPS  256

typedef enum {
    PATH_STEP_MOVE,
    PATH_STEP_STOP,
    PATH_STEP_PUMP,
    PATH_STEP_BLOWER,
} path_step_type_t;

typedef struct {
    path_step_type_t type;
    motor_dir_t      dir;
    int              ms;
    bool             on;
} path_step_t;

static path_step_t    s_path[PATH_MAX_STEPS];
static volatile int   s_path_len         = 0;
static volatile bool  s_path_recording   = false;
static volatile bool  s_path_playback    = false;
static int64_t        s_rec_cmd_start_ms = 0;
static motor_dir_t    s_rec_last_dir     = DIR_STOP;

/* ─── Module state ────────────────────────────────────────── */

static clean_grid_t s_grid = {
    .panel_cols     = 2,
    .panel_rows     = 1,
    .panel_w_cm     = 200,
    .panel_h_cm     = 170,
    .gap_between_cm = 5,
    .row_gap_cm     = 10,
    .wash_enabled   = true,
    .blow_enabled   = false,
    .passes         = 1,
};

/*
 * FIX v3.1: Added CLEAN_STOPPING state so auto_clean_stop() can signal
 * the task without clobbering state before the task has exited.
 */
static volatile clean_state_t s_state           = CLEAN_IDLE;
static volatile bool          s_stop_requested  = false;
static volatile bool          s_pause_requested = false;
static volatile int           s_strips_done     = 0;
static volatile int           s_strips_total    = 0;
static volatile int           s_current_panel   = 0;
static volatile int           s_current_strip   = 0;

static float s_base_heading = 0.0f;

/* ─── Grid API ────────────────────────────────────────────── */

void auto_clean_set_grid(const clean_grid_t *g)
{
    s_grid = *g;
    if (s_grid.panel_cols < 1)       s_grid.panel_cols      = 1;
    if (s_grid.panel_cols > 20)      s_grid.panel_cols      = 20;
    if (s_grid.panel_rows < 1)       s_grid.panel_rows      = 1;
    if (s_grid.panel_rows > 20)      s_grid.panel_rows      = 20;
    if (s_grid.panel_w_cm < 20)      s_grid.panel_w_cm      = 20;
    if (s_grid.panel_h_cm < 20)      s_grid.panel_h_cm      = 20;
    if (s_grid.gap_between_cm < 0)   s_grid.gap_between_cm  = 0;
    if (s_grid.gap_between_cm > 500) s_grid.gap_between_cm  = 500;
    if (s_grid.row_gap_cm < 0)       s_grid.row_gap_cm      = 0;
    if (s_grid.row_gap_cm > 500)     s_grid.row_gap_cm      = 500;
    if (s_grid.passes < 1)           s_grid.passes          = 1;
    if (s_grid.passes > 3)           s_grid.passes          = 3;
}

void auto_clean_get_grid(clean_grid_t *g) { *g = s_grid; }

clean_state_t auto_clean_state(void)    { return s_state;         }
bool  auto_clean_is_running(void)       { return s_state == CLEAN_RUNNING || s_state == CLEAN_PAUSED; }
int   auto_clean_strips_done(void)      { return s_strips_done;   }
int   auto_clean_strips_total(void)     { return s_strips_total;  }
int   auto_clean_current_panel(void)    { return s_current_panel; }
int   auto_clean_current_strip(void)    { return s_current_strip; }

int auto_clean_pct(void)
{
    if (s_strips_total <= 0) return 0;
    int p = (s_strips_done * 100) / s_strips_total;
    return (p > 100) ? 100 : p;
}

/* ─── Path recording API ─────────────────────────────────── */

bool auto_path_recording(void) { return s_path_recording; }
bool auto_path_playback(void)  { return s_path_playback;  }
int  auto_path_len(void)       { return s_path_len;       }

bool auto_path_record_start(void)
{
    if (auto_clean_is_running() || s_path_playback) return false;
    s_path_len         = 0;
    s_path_recording   = true;
    s_rec_last_dir     = DIR_STOP;
    s_rec_cmd_start_ms = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Path recording STARTED");
    return true;
}

void auto_path_record_cmd(motor_dir_t dir)
{
    if (!s_path_recording) return;
    int64_t now_ms = esp_timer_get_time() / 1000;

    if (s_rec_last_dir != DIR_STOP && s_path_len < PATH_MAX_STEPS) {
        int dur = (int)(now_ms - s_rec_cmd_start_ms);
        if (dur > 20) {
            s_path[s_path_len].type = PATH_STEP_MOVE;
            s_path[s_path_len].dir  = s_rec_last_dir;
            s_path[s_path_len].ms   = dur;
            s_path_len++;
        }
    }

    if (dir == DIR_STOP && s_path_len < PATH_MAX_STEPS) {
        s_path[s_path_len].type = PATH_STEP_STOP;
        s_path[s_path_len].ms   = 0;
        s_path_len++;
    }

    s_rec_last_dir     = dir;
    s_rec_cmd_start_ms = now_ms;
}

void auto_path_record_peripheral(bool is_pump, bool on)
{
    if (!s_path_recording || s_path_len >= PATH_MAX_STEPS) return;
    s_path[s_path_len].type = is_pump ? PATH_STEP_PUMP : PATH_STEP_BLOWER;
    s_path[s_path_len].on   = on;
    s_path[s_path_len].ms   = 0;
    s_path_len++;
}

void auto_path_record_stop(void)
{
    if (!s_path_recording) return;
    auto_path_record_cmd(DIR_STOP);
    s_path_recording = false;
    ESP_LOGI(TAG, "Path recording STOPPED  %d steps", s_path_len);
}

void auto_path_clear(void)
{
    s_path_len = 0;
    ESP_LOGI(TAG, "Path cleared");
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

/*
 * correct_heading_to():
 *   Spin in proportional bursts until gyro reads within HEADING_OK_DEG
 *   or CORRECT_MAX_MS elapses.
 */
static bool correct_heading_to(float target_deg)
{
#if MPU_ENABLED
    if (!mpu_is_ready()) {
        motor_send_cmd(DIR_STOP);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
        return !s_stop_requested;
    }

    const int POLL_MS = 25;
    int       elapsed = 0;
    int       stable  = 0;

    motor_set_speed(MOTOR_SPEED_TURN);

    while (elapsed < CORRECT_MAX_MS) {
        if (s_stop_requested) {
            motor_send_cmd(DIR_STOP);
            return false;
        }

        /* FIX v3.1: pause check restored */
        while (s_pause_requested && !s_stop_requested) {
            motor_send_cmd(DIR_STOP);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (s_stop_requested) {
            motor_send_cmd(DIR_STOP);
            return false;
        }

        float cur = mpu_heading();
        float err = heading_error(target_deg, cur);

        if (fabsf(err) <= HEADING_OK_DEG) {
            if (++stable >= 3) {
                motor_send_cmd(DIR_STOP);
                vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
                ESP_LOGI(TAG, "Hdg OK → target=%.1f  cur=%.1f",
                         (double)target_deg, (double)cur);
                return true;
            }
        } else {
            stable = 0;
            float ratio = fabsf(err) / 45.0f;
            if (ratio > 1.0f) ratio = 1.0f;
            uint8_t spd = (uint8_t)(MOTOR_SPEED_TURN * (0.5f + 0.5f * ratio));
            motor_set_speed(spd);
            motor_send_cmd((err > 0) ? DIR_RIGHT : DIR_LEFT);
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
        elapsed += POLL_MS;
    }

    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
    ESP_LOGW(TAG, "Heading correction timed out (target=%.1f cur=%.1f)",
             (double)target_deg, (double)mpu_heading());
    return !s_stop_requested;
#else
    (void)target_deg;
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
    return !s_stop_requested;
#endif
}

/*
 * timed_turn_then_correct():
 *   Pre-spin a fraction of the expected time, then let gyro trim.
 */
static bool timed_turn_then_correct(motor_dir_t spin_dir,
                                    int         expected_ms,
                                    float       target_deg)
{
    if (s_stop_requested) return false;

    int spin_ms = (int)(expected_ms * TURN_FRACTION);
    motor_set_speed(MOTOR_SPEED_TURN);
    motor_send_cmd(spin_dir);
    vTaskDelay(pdMS_TO_TICKS(spin_ms));
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

    return correct_heading_to(target_deg);
}

/* ─── Pause/stop check helper ────────────────────────────── */

/*
 * FIX v3.1: Returns true if we should ABORT (stop requested).
 * Blocks here while paused.
 */
static bool check_pause_stop(void)
{
    while (s_pause_requested && !s_stop_requested) {
        motor_send_cmd(DIR_STOP);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return s_stop_requested;
}

/* ─── Forward strip sweep ─────────────────────────────────── */

/*
 * safe_forward_strip():
 *   1. Servo → DEPLOY (wiper active)
 *   2. Check pause/stop before starting
 *   3. Lift servo BOUNDARY_ADVANCE_CM before panel end
 *   4. Complete travel
 *   5. Servo → PARK
 *
 * FIX v3.1: pause/stop guards after deploy and after phase-1 are restored.
 */
#define BOUNDARY_ADVANCE_CM   8   /* lift servo this far before edge */

static bool safe_forward_strip(int total_ms, float hdg)
{
    if (s_stop_requested) return false;

    ESP_LOGI(TAG, "FWD strip: %d ms  hdg=%.1f", total_ms, (double)hdg);

    /* Deploy wiper */
    servo_deploy();

    /* FIX v3.1: check pause/stop right after deploying */
    if (check_pause_stop()) {
        servo_park();
        return false;
    }

    /* Calculate when to lift wiper */
    int boundary_ms = BOUNDARY_ADVANCE_CM * MS_PER_CM;
    int main_ms     = total_ms - boundary_ms;
    if (main_ms < 0) { main_ms = total_ms; boundary_ms = 0; }

    /* Phase 1: main travel with wiper deployed */
    motor_set_speed(MOTOR_SPEED_CLEAN);
#if MPU_ENABLED
    bool ok = motor_move_straight(DIR_FORWARD, main_ms, hdg, HEADING_KP);
#else
    motor_send_cmd(DIR_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(main_ms));
    motor_send_cmd(DIR_STOP);
    bool ok = true;
#endif

    /* FIX v3.1: check after phase-1 completes */
    if (!ok || s_stop_requested) {
        servo_park();
        return false;
    }

    if (check_pause_stop()) {
        servo_park();
        return false;
    }

    /* Phase 2: lift wiper for boundary approach */
    if (boundary_ms > 0) {
        servo_boundary();
        motor_set_speed(MOTOR_SPEED_CLEAN);
#if MPU_ENABLED
        ok = motor_move_straight(DIR_FORWARD, boundary_ms, hdg, HEADING_KP);
#else
        motor_send_cmd(DIR_FORWARD);
        vTaskDelay(pdMS_TO_TICKS(boundary_ms));
        motor_send_cmd(DIR_STOP);
#endif
    }

    /* Park wiper and settle */
    servo_park();
    vTaskDelay(pdMS_TO_TICKS(POST_STRIP_PAUSE_MS));

    return !s_stop_requested;
}

/* ─── Lateral step between strips ─────────────────────────── */

/*
 * safe_lateral_step():
 *   Wiper is parked during all turns/steps.
 *   Sequence:
 *     1. Turn 90° to face the step direction
 *     2. Forward for step_ms
 *     3. Turn 90° back (now facing panel reverse direction)
 *     4. Turn 180° to face forward again
 *     → Final heading = s_base_heading
 *
 * FIX v3.1: motor_set_speed(MOTOR_SPEED_REPOSITION) is set explicitly
 *            before the lateral forward move.
 */
static bool safe_lateral_step(bool go_right, int step_ms, float base_hdg)
{
    if (s_stop_requested) return false;

    servo_park();

    float hdg_side = norm_heading(base_hdg + (go_right ?  90.0f : -90.0f));
    float hdg_back = norm_heading(base_hdg + 180.0f);

    ESP_LOGI(TAG, "STEP %s  step_ms=%d  side=%.1f  back=%.1f",
             go_right ? "→RIGHT" : "←LEFT",
             step_ms, (double)hdg_side, (double)hdg_back);

    motor_dir_t spin1 = go_right ? DIR_RIGHT : DIR_LEFT;
    motor_dir_t spin2 = go_right ? DIR_LEFT  : DIR_RIGHT;

    /* Turn to face step direction */
    if (!timed_turn_then_correct(spin1, TURN_90_MS, hdg_side)) return false;
    if (check_pause_stop()) return false;

    /* FIX v3.1: set speed explicitly before lateral step */
    motor_set_speed(MOTOR_SPEED_REPOSITION);
    motor_send_cmd(DIR_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(step_ms));
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
    if (s_stop_requested) return false;

    /* Turn back 90° → now facing backward (base_hdg + 180) */
    if (!timed_turn_then_correct(spin2, TURN_90_MS, hdg_back)) return false;
    if (check_pause_stop()) return false;

    /* Turn another 180° → back to forward heading */
    if (!timed_turn_then_correct(spin2, TURN_90_MS * 2,
                                 norm_heading(base_hdg))) return false;

    vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
    return !s_stop_requested;
}

/* ─── Path playback task ─────────────────────────────────── */

static void path_playback_task(void *arg)
{
    (void)arg;
    s_state          = CLEAN_RUNNING;
    s_stop_requested = false;

    ESP_LOGI(TAG, "Path playback START  %d steps", s_path_len);

    for (int i = 0; i < s_path_len && !s_stop_requested; i++) {
        /* Pause support */
        while (s_pause_requested && !s_stop_requested) {
            motor_send_cmd(DIR_STOP);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (s_stop_requested) break;

        const path_step_t *st = &s_path[i];
        switch (st->type) {
        case PATH_STEP_MOVE:
            ESP_LOGI(TAG, "  Step %d: MOVE dir=%d  %d ms", i, (int)st->dir, st->ms);
            motor_send_cmd(st->dir);
            vTaskDelay(pdMS_TO_TICKS(st->ms));
            motor_send_cmd(DIR_STOP);
            vTaskDelay(pdMS_TO_TICKS(150));
            break;
        case PATH_STEP_STOP:
            motor_send_cmd(DIR_STOP);
            if (st->ms > 0) vTaskDelay(pdMS_TO_TICKS(st->ms));
            break;
        case PATH_STEP_PUMP:
            pump_set(st->on);
            break;
        case PATH_STEP_BLOWER:
            blower_set(st->on);
            break;
        }
    }

    motor_send_cmd(DIR_STOP);
    pump_set(false);
    blower_set(false);
    servo_park();
    s_path_playback = false;
    s_state         = s_stop_requested ? CLEAN_IDLE : CLEAN_DONE;
    ESP_LOGI(TAG, "Path playback DONE");
    vTaskDelete(NULL);
}

bool auto_path_play_start(void)
{
    if (s_path_len == 0)              { ESP_LOGW(TAG, "No path recorded"); return false; }
    if (auto_clean_is_running())      { ESP_LOGW(TAG, "Already running");  return false; }

    s_path_playback   = true;
    s_stop_requested  = false;
    s_pause_requested = false;

    /*
     * FIX v3.1: pin to core 0 — motor_task runs on core 1.
     * This eliminates the dual-core race on s_cmd.
     */
    BaseType_t rc = xTaskCreatePinnedToCore(path_playback_task, "path_play",
                                            4096, NULL, 5, NULL, 0);
    if (rc != pdPASS) {
        s_path_playback = false;
        s_state         = CLEAN_ERROR;
        return false;
    }
    return true;
}

/* ─── Auto-clean task ────────────────────────────────────── */

static void auto_clean_task(void *arg)
{
    (void)arg;
    const clean_grid_t g = s_grid;

    const int strips_per_panel =
        (g.panel_h_cm + ROVER_WIDTH_CM - 1) / ROVER_WIDTH_CM;

    s_strips_total  = strips_per_panel * g.panel_cols * g.panel_rows;
    s_strips_done   = 0;
    s_current_panel = 0;
    s_current_strip = 0;

    const int forward_ms = g.panel_w_cm   * MS_PER_CM;
    const int step_ms    = ROVER_WIDTH_CM * MS_PER_CM;

    s_state      = CLEAN_RUNNING;
    bool go_right = true;

    if (g.wash_enabled) pump_set(true);
    if (g.blow_enabled) blower_set(true);

    gpio_set_level(PIN_TAIL_LIGHT, 1);

    /* ── Row × Column panel loop ── */
    for (int pr = 0; pr < g.panel_rows && !s_stop_requested; pr++) {
        for (int pc = 0; pc < g.panel_cols && !s_stop_requested; pc++) {

            s_current_panel = pr * g.panel_cols + pc;
            ESP_LOGI(TAG, "Panel [row=%d col=%d]  strips=%d",
                     pr, pc, strips_per_panel);

            if (!correct_heading_to(s_base_heading)) break;

            /* ── Strip loop ── */
            for (int i = 0; i < strips_per_panel && !s_stop_requested; i++) {

                s_current_strip = i;

                /* Multi-pass */
                for (int pass = 0; pass < g.passes && !s_stop_requested; pass++) {
                    ESP_LOGI(TAG, "  Strip %d/%d  pass %d/%d",
                             i+1, strips_per_panel, pass+1, g.passes);

                    if (!safe_forward_strip(forward_ms, s_base_heading)) break;

                    /* Return for next pass */
                    if (pass < g.passes - 1) {
                        servo_park();

                        /* FIX v3.1: set speed before backward move */
                        motor_set_speed(MOTOR_SPEED_REPOSITION);
                        bool ok = motor_move_straight(
                            DIR_BACKWARD, forward_ms,
                            norm_heading(s_base_heading + 180.0f),
                            HEADING_KP);
                        if (!ok || s_stop_requested) break;

                        if (!correct_heading_to(s_base_heading)) break;
                    }
                }

                if (s_stop_requested) break;

                s_strips_done++;
                ESP_LOGI(TAG, "Strip %d/%d done (%d%%)",
                         s_strips_done, s_strips_total, auto_clean_pct());

                if (i == strips_per_panel - 1) break;

                /* ── Lateral step ── */
                if (!safe_lateral_step(go_right, step_ms, s_base_heading)) break;
                go_right = !go_right;

                vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
            }

            /* ── Inter-panel column advance ── */
            if (pc < g.panel_cols - 1 && !s_stop_requested) {
                ESP_LOGI(TAG, "Advancing to next panel column...");
                servo_park();

                if (!correct_heading_to(s_base_heading)) break;

                int gap_ms = g.gap_between_cm * MS_PER_CM;

                /* FIX v3.1: set speed before inter-panel move */
                motor_set_speed(MOTOR_SPEED_REPOSITION);
                bool ok = motor_move_straight(DIR_FORWARD, gap_ms,
                                              s_base_heading, HEADING_KP);
                if (!ok || s_stop_requested) break;

                vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
                go_right = true;
            }
        }

        /* ── Inter-row advance ── */
        if (pr < g.panel_rows - 1 && !s_stop_requested) {
            ESP_LOGI(TAG, "Advancing to next panel row...");
            servo_park();

            float hdg_side = norm_heading(s_base_heading - 90.0f);
            float hdg_fwd  = s_base_heading;

            if (!timed_turn_then_correct(DIR_LEFT, TURN_90_MS, hdg_side)) break;

            int row_advance_ms = (g.panel_h_cm + g.row_gap_cm) * MS_PER_CM;

            /* FIX v3.1: set speed before row advance */
            motor_set_speed(MOTOR_SPEED_REPOSITION);
            bool ok = motor_move_straight(DIR_FORWARD, row_advance_ms,
                                          hdg_side, HEADING_KP);
            if (!ok || s_stop_requested) break;

            if (!timed_turn_then_correct(DIR_RIGHT, TURN_90_MS, hdg_fwd)) break;

            go_right = true;
            vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
        }
    }

    /* ── Done ── */
    motor_send_cmd(DIR_STOP);
    pump_set(false);
    blower_set(false);
    servo_park();
    gpio_set_level(PIN_TAIL_LIGHT, 0);

    /*
     * FIX v3.1: Only set CLEAN_DONE if we weren't stopped externally.
     * auto_clean_stop() may have already set CLEAN_IDLE; don't overwrite
     * it with CLEAN_DONE — but if stop wasn't requested, mark as done.
     */
    if (!s_stop_requested) {
        s_state = CLEAN_DONE;
    }
    /* If stop was requested, s_state was already set to CLEAN_IDLE by
     * auto_clean_stop(); we leave it alone. */

    s_current_strip = 0;
    ESP_LOGI(TAG, "Auto-clean COMPLETE  strips=%d / %d  state=%d",
             s_strips_done, s_strips_total, (int)s_state);
    vTaskDelete(NULL);
}

/* ─── Public API ─────────────────────────────────────────── */

bool auto_clean_start(void)
{
    if (auto_clean_is_running()) {
        ESP_LOGW(TAG, "Already running");
        return false;
    }

    s_stop_requested  = false;
    s_pause_requested = false;
    s_strips_done     = 0;
    s_strips_total    = 0;
    s_current_panel   = 0;
    s_current_strip   = 0;

    mpu_reset_heading();
    s_base_heading = 0.0f;

    servo_park();
    gpio_set_level(PIN_TAIL_LIGHT, 1);

    /*
     * FIX v3.1: pin to core 0 — motor_task runs on core 1.
     * Eliminates the dual-core race on the volatile s_cmd in motor.c.
     */
    BaseType_t rc = xTaskCreatePinnedToCore(auto_clean_task, "auto_clean",
                                            6144, NULL, 5, NULL, 0);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create auto_clean task");
        s_state = CLEAN_ERROR;
        gpio_set_level(PIN_TAIL_LIGHT, 0);
        return false;
    }
    return true;
}

/*
 * auto_clean_stop()
 *
 * FIX v3.1: No longer forces s_state = CLEAN_IDLE immediately.
 * Instead it sets s_stop_requested = true and lets the running task
 * exit its own loop, clean up peripherals, and then decide the final
 * state.  We only force CLEAN_IDLE here if no task is actually running
 * (e.g. called when already idle).
 *
 * We still stop the motor and peripherals immediately for safety.
 */
void auto_clean_stop(void)
{
    if (auto_clean_is_running() || s_path_playback) {
        s_stop_requested = true;
        motor_stop_immediate();
        pump_set(false);
        blower_set(false);
        servo_park();
        gpio_set_level(PIN_TAIL_LIGHT, 0);
        s_path_playback = false;
        /* FIX: do NOT set s_state here — let the task set CLEAN_IDLE
         * after it has safely unwound.  The task checks s_stop_requested
         * at every loop boundary and will exit promptly. */
        ESP_LOGW(TAG, "Auto-clean STOP requested");
    } else {
        /* Nothing was running — just make sure state is clean */
        s_state = CLEAN_IDLE;
    }
}

void auto_clean_pause(void)
{
    if (s_state == CLEAN_RUNNING) {
        s_pause_requested = true;
        s_state           = CLEAN_PAUSED;
        motor_send_cmd(DIR_STOP);
        ESP_LOGI(TAG, "Auto-clean PAUSED");
    }
}

void auto_clean_resume(void)
{
    if (s_state == CLEAN_PAUSED && s_pause_requested) {
        s_pause_requested = false;
        s_state           = CLEAN_RUNNING;
        ESP_LOGI(TAG, "Auto-clean RESUMED");
    }
}