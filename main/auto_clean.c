#include "auto_clean.h"
#include "config.h"
#include "motor.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>
#include "esp_timer.h"

static const char *TAG = "AUTO";

/* ─── Tuning constants ────────────────────────────────────── */

/* Time for a 90-degree in-place spin (milliseconds).
   Increase if rover under-turns, decrease if it over-turns.
   With timed_turn_then_correct() the gyro trims residual error,
   so this just needs to be within ~30 % of the real value.       */
#define TURN_90_MS          750

/* Pause after stopping before we trust the gyro reading (ms).    */
#define SETTLE_MS           350

/* Maximum time the heading-correction loop may run (ms).         */
#define CORRECT_MAX_MS      3000

/* Heading error below which we consider ourselves "on target".   */
#define HEADING_OK_DEG      4.0f

/* How far we spin in the timed-turn phase (fraction of full 90). */
#define TURN_FRACTION       0.65f

/* Extra pause after every lateral step (ms) – lets the chassis
   stop rocking before the next sweep.                            */
#define POST_STEP_SETTLE_MS 500

/* Extra pause between strips to let IMU re-lock (ms).            */
#define POST_STRIP_PAUSE_MS 400

/* ─── Path recording ─────────────────────────────────────── */

#define PATH_MAX_STEPS  256

typedef enum {
    PATH_STEP_MOVE,   /* motor_dir + duration_ms              */
    PATH_STEP_STOP,   /* pause                                */
    PATH_STEP_PUMP,   /* pump on/off                          */
    PATH_STEP_BLOWER, /* blower on/off                        */
} path_step_type_t;

typedef struct {
    path_step_type_t type;
    motor_dir_t      dir;      /* PATH_STEP_MOVE            */
    int              ms;       /* duration (MOVE) or delay  */
    bool             on;       /* PUMP / BLOWER             */
} path_step_t;

static path_step_t    s_path[PATH_MAX_STEPS];
static volatile int   s_path_len         = 0;
static volatile bool  s_path_recording   = false;
static volatile bool  s_path_playback    = false;

/* Timestamp of the last motor command during recording */
static int64_t        s_rec_cmd_start_ms = 0;
static motor_dir_t    s_rec_last_dir     = DIR_STOP;

/* ─── Module state ────────────────────────────────────────── */

static clean_grid_t s_grid = {
    .panel_cols     = 2,
    .panel_rows     = 1,
    .panel_w_cm     = 200,
    .panel_h_cm     = 170,
    .gap_between_cm = 5,
    .wash_enabled   = true,
    .blow_enabled   = false,
};

static volatile clean_state_t s_state          = CLEAN_IDLE;
static volatile bool          s_stop_requested = false;
static volatile int           s_strips_done    = 0;
static volatile int           s_strips_total   = 0;

static float s_base_heading = 0.0f;

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

void auto_clean_get_grid(clean_grid_t *g) { *g = s_grid; }
clean_state_t auto_clean_state(void)      { return s_state; }
bool  auto_clean_is_running(void)         { return s_state == CLEAN_RUNNING; }
int   auto_clean_strips_done(void)        { return s_strips_done; }
int   auto_clean_strips_total(void)       { return s_strips_total; }
int   auto_clean_pct(void)
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
    if (s_state == CLEAN_RUNNING || s_path_playback) return false;
    s_path_len         = 0;
    s_path_recording   = true;
    s_rec_last_dir     = DIR_STOP;
    s_rec_cmd_start_ms = esp_timer_get_time() / 1000;
    ESP_LOGI(TAG, "Path recording STARTED");
    return true;
}

/* Called by web_server whenever a manual move command arrives */
void auto_path_record_cmd(motor_dir_t dir)
{
    if (!s_path_recording) return;
    int64_t now_ms = esp_timer_get_time() / 1000;

    /* Flush the previous segment if it was a MOVE */
    if (s_rec_last_dir != DIR_STOP && s_path_len < PATH_MAX_STEPS) {
        int dur = (int)(now_ms - s_rec_cmd_start_ms);
        if (dur > 20) {                      /* ignore noise <20 ms */
            s_path[s_path_len].type = PATH_STEP_MOVE;
            s_path[s_path_len].dir  = s_rec_last_dir;
            s_path[s_path_len].ms   = dur;
            s_path_len++;
        }
    }

    if (dir == DIR_STOP) {
        /* record explicit stop with 0 duration pause */
        if (s_path_len < PATH_MAX_STEPS) {
            s_path[s_path_len].type = PATH_STEP_STOP;
            s_path[s_path_len].ms   = 0;
            s_path_len++;
        }
    }

    s_rec_last_dir     = dir;
    s_rec_cmd_start_ms = now_ms;
}

/* Called by web_server when pump/blower state changes during recording */
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
    /* flush any ongoing move */
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
 *   Spin in short bursts until gyro reads within HEADING_OK_DEG of
 *   target, or CORRECT_MAX_MS elapses.  Returns false only when
 *   s_stop_requested fires.
 */
static bool correct_heading_to(float target_deg)
{
#if MPU_ENABLED
    if (!mpu_is_ready()) {
        motor_send_cmd(DIR_STOP);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
        return !s_stop_requested;
    }

    const int POLL_MS  = 25;
    int       elapsed  = 0;
    int       stable   = 0;          /* consecutive ticks inside band */

    while (elapsed < CORRECT_MAX_MS) {
        if (s_stop_requested) {
            motor_send_cmd(DIR_STOP);
            return false;
        }

        float cur = mpu_heading();
        float err = heading_error(target_deg, cur);

        if (fabsf(err) <= HEADING_OK_DEG) {
            if (++stable >= 3) {     /* must stay stable for 3 polls  */
                motor_send_cmd(DIR_STOP);
                vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
                ESP_LOGI(TAG, "Heading OK: target=%.1f  cur=%.1f  err=%.1f",
                         (double)target_deg, (double)cur, (double)err);
                return true;
            }
        } else {
            stable = 0;
            /* proportional-ish: large error → full spin, small → short burst */
            motor_dir_t spin = (err > 0) ? DIR_RIGHT : DIR_LEFT;
            motor_send_cmd(spin);
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
 *   Spin for TURN_FRACTION * expected_ms then let the gyro trim the rest.
 */
static bool timed_turn_then_correct(motor_dir_t spin_dir,
                                    int         expected_ms,
                                    float       target_deg)
{
    if (s_stop_requested) return false;

    int spin_ms = (int)(expected_ms * TURN_FRACTION);
    motor_send_cmd(spin_dir);
    vTaskDelay(pdMS_TO_TICKS(spin_ms));
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

    return correct_heading_to(target_deg);
}

/* ─── Stop-aware move wrappers ────────────────────────────── */

/*
 * safe_forward_strip():
 *   Move forward for duration_ms while maintaining hdg via drift
 *   correction.  Pauses POST_STRIP_PAUSE_MS at the end.
 */
static bool safe_forward_strip(int duration_ms, float hdg)
{
    if (s_stop_requested) return false;

    ESP_LOGI(TAG, "FWD strip: %d ms  heading=%.1f", duration_ms, (double)hdg);

#if MPU_ENABLED
    bool ok = motor_move_straight(DIR_FORWARD, duration_ms, hdg, 5.0f);
#else
    motor_send_cmd(DIR_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    motor_send_cmd(DIR_STOP);
    bool ok = true;
#endif

    vTaskDelay(pdMS_TO_TICKS(POST_STRIP_PAUSE_MS));
    return ok && !s_stop_requested;
}

/*
 * safe_lateral_step():
 *   Spin 90°, move step, spin back 180°.  Uses gyro for both turns.
 *   After the step, re-aligns to target heading precisely.
 */
static bool safe_lateral_step(bool go_right, int step_ms, float base_hdg)
{
    if (s_stop_requested) return false;

    float hdg_side = norm_heading(base_hdg + (go_right ?  90.0f : -90.0f));
    float hdg_back = norm_heading(base_hdg + 180.0f);   /* now facing back */

    ESP_LOGI(TAG, "LATERAL STEP: %s  step_ms=%d  side_hdg=%.1f",
             go_right ? "RIGHT" : "LEFT", step_ms, (double)hdg_side);

    motor_dir_t spin1 = go_right ? DIR_RIGHT : DIR_LEFT;
    motor_dir_t spin2 = go_right ? DIR_LEFT  : DIR_RIGHT;

    /* Turn 90° to face the side */
    if (!timed_turn_then_correct(spin1, TURN_90_MS, hdg_side)) return false;

    /* Step across */
    motor_send_cmd(DIR_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(step_ms));
    motor_send_cmd(DIR_STOP);
    vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
    if (s_stop_requested) return false;

    /* Turn 90° to face back (along the panel) — now at base_hdg + 180 */
    if (!timed_turn_then_correct(spin2, TURN_90_MS, hdg_back)) return false;

    /* ── Rewind pass (optional gyro-straight backward) ── */
    /* We do NOT use backward for actual cleaning; only to reach
       next strip start. Could be left out if strips overlap enough.
       For now we just leave the rover at the new strip start facing
       the forward direction; the strip itself calls safe_forward_strip. */

    /* Final re-align to base heading (+ 180 already done above) */
    /* Turn another 180 so we face the original forward direction  */
    if (!timed_turn_then_correct(spin2, TURN_90_MS * 2,
                                 norm_heading(base_hdg))) return false;

    vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
    return !s_stop_requested;
}

/* ─── Path playback task ─────────────────────────────────── */

static void path_playback_task(void *arg)
{
    (void)arg;
    s_state         = CLEAN_RUNNING;
    s_stop_requested = false;

    ESP_LOGI(TAG, "Path playback START  %d steps", s_path_len);

    for (int i = 0; i < s_path_len && !s_stop_requested; i++) {
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
    s_path_playback = false;
    s_state         = CLEAN_DONE;
    ESP_LOGI(TAG, "Path playback DONE");
    vTaskDelete(NULL);
}

bool auto_path_play_start(void)
{
    if (s_path_len == 0) {
        ESP_LOGW(TAG, "No path recorded");
        return false;
    }
    if (s_state == CLEAN_RUNNING) {
        ESP_LOGW(TAG, "Already running");
        return false;
    }
    s_path_playback  = true;
    s_stop_requested = false;

    BaseType_t rc = xTaskCreate(path_playback_task, "path_play",
                                4096, NULL, 5, NULL);
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

    /*
     * Number of strips = ceil(panel_height / rover_width).
     * Each strip runs the full WIDTH of the panel.
     * The rover steps along the HEIGHT direction between strips.
     */
    const int strips_per_panel =
        (g.panel_h_cm + ROVER_WIDTH_CM - 1) / ROVER_WIDTH_CM;

    /* Total strips across all panels */
    s_strips_total = strips_per_panel * g.panel_cols * g.panel_rows;
    s_strips_done  = 0;

    /* Time to traverse one strip (width of panel) */
    const int forward_ms = g.panel_w_cm * MS_PER_CM;

    /* Time to step one rover-width sideways */
    const int step_ms = ROVER_WIDTH_CM * MS_PER_CM;

    s_state      = CLEAN_RUNNING;
    bool go_right = true;

    /* Activate peripherals as configured */
    if (g.wash_enabled)  pump_set(true);
    if (g.blow_enabled)  blower_set(true);

    /* Outer loops: iterate over panel grid (row-major) */
    for (int pr = 0; pr < g.panel_rows && !s_stop_requested; pr++) {
        for (int pc = 0; pc < g.panel_cols && !s_stop_requested; pc++) {

            ESP_LOGI(TAG, "Panel [row=%d col=%d]  strips=%d",
                     pr, pc, strips_per_panel);

            /* ── inner loop: sweep strips across this panel ── */
            for (int i = 0; i < strips_per_panel && !s_stop_requested; i++) {

                /* ── 1. Forward sweep ── */
                if (!safe_forward_strip(forward_ms, s_base_heading)) break;
                s_strips_done++;
                ESP_LOGI(TAG, "Strip done: %d / %d", s_strips_done, s_strips_total);

                if (i == strips_per_panel - 1) break; /* last strip — no step */

                /* ── 2. Lateral step to next strip ── */
                if (!safe_lateral_step(go_right, step_ms, s_base_heading)) break;

                /* After step, the rover is back at base heading. */
                go_right = !go_right;

                vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
            }

            /* ── Move to the next panel column (gap + panel_w) ── */
            if (pc < g.panel_cols - 1 && !s_stop_requested) {
                int gap_ms = (g.panel_w_cm + g.gap_between_cm) * MS_PER_CM;
                ESP_LOGI(TAG, "Moving to next panel col: %d ms", gap_ms);
                /* Realign heading before inter-panel move */
                if (!correct_heading_to(s_base_heading)) break;
                motor_send_cmd(DIR_FORWARD);
                vTaskDelay(pdMS_TO_TICKS(gap_ms));
                motor_send_cmd(DIR_STOP);
                vTaskDelay(pdMS_TO_TICKS(POST_STEP_SETTLE_MS));
            }
        }

        /* ── Move to next panel row ── */
        if (pr < g.panel_rows - 1 && !s_stop_requested) {
            /* TODO: implement row-to-row repositioning if your layout
               requires it — for flat horizontal arrays, this is a
               simple forward step equal to panel_h + gap.           */
        }
    }

    motor_send_cmd(DIR_STOP);
    pump_set(false);
    blower_set(false);
    gpio_set_level(PIN_TAIL_LIGHT, 0);

    s_state = CLEAN_DONE;
    ESP_LOGI(TAG, "Auto-clean COMPLETE  strips=%d", s_strips_done);
    vTaskDelete(NULL);
}

/* ─── Public API ─────────────────────────────────────────── */

bool auto_clean_start(void)
{
    if (s_state == CLEAN_RUNNING) {
        ESP_LOGW(TAG, "auto_clean_start: already running");
        return false;
    }

    s_stop_requested = false;
    s_strips_done    = 0;
    s_strips_total   = 0;

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
    if (s_state == CLEAN_RUNNING || s_path_playback) {
        s_stop_requested = true;
        motor_stop_immediate();
        pump_set(false);
        blower_set(false);
        s_path_playback = false;
        ESP_LOGW(TAG, "Stop requested");
    }
}